/**
 * @file pyro_crashlog.c
 * @brief 崩溃日志实现：RAM 环形缓冲 + HardFault 落 .noinit
 */

#include "pyro_crashlog.h"

#include "stm32h7xx.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/*---------------------------------------------------------------------------*/
/* RAM 环形缓冲（普通 .bss，复位清零） */
/*---------------------------------------------------------------------------*/

static char     s_ring[PYRO_CRASHLOG_LINE_NUM][PYRO_CRASHLOG_LINE_MAX];
static uint32_t s_ring_count = 0; /* 有效条数 */
static uint32_t s_ring_head  = 0; /* 下一个写入位置 */
static uint32_t s_ring_dropped = 0; /* 因环形缓冲满而覆盖的条数 */

/*---------------------------------------------------------------------------*/
/* .noinit 崩溃记录（复位不清零） */
/*---------------------------------------------------------------------------*/

__attribute__((section(".noinit"))) static pyro_crashlog_record_t s_crash_record;

/*---------------------------------------------------------------------------*/
/* .noinit 崩溃循环检测（复位不清零，与 s_crash_record 分离） */
/*---------------------------------------------------------------------------*/

/* 结构布局变化时递增，避免复位后误读旧版本记录 */
#define PYRO_CRASHLOOP_MAGIC 0x434C4F51U /* "CLOQ" */

typedef struct
{
    uint32_t magic;        /* PYRO_CRASHLOOP_MAGIC 表示有效 */
    uint32_t count;        /* 同一现场连续崩溃次数 */
    uint32_t last_pc;      /* 最近一次崩溃 PC */
    uint32_t last_cfsr;    /* 最近一次崩溃 CFSR */
    uint32_t first_pc;     /* 本轮循环第一次崩溃 PC */
    uint32_t first_cfsr;   /* 本轮循环第一次崩溃 CFSR */
    uint32_t logged_pc;    /* 已写入 /crash.log 的 PC */
    uint32_t logged_cfsr;  /* 已写入 /crash.log 的 CFSR */
    uint32_t logged_valid; /* 1 = logged_pc / logged_cfsr 有效 */
    uint32_t total_faults; /* 本次上电累计 HardFault 次数 */
    uint32_t checksum;     /* 除 magic、checksum 外的校验和 */
} pyro_crashloop_t;

__attribute__((section(".noinit"))) static pyro_crashloop_t s_crashloop;

static uint32_t crashloop_checksum(const pyro_crashloop_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t hash = 2166136261UL;
    const size_t begin = offsetof(pyro_crashloop_t, count);
    const size_t end   = offsetof(pyro_crashloop_t, checksum);

    for (size_t i = begin; i < end; ++i)
    {
        hash ^= data[i];
        hash *= 16777619UL;
    }
    return hash;
}

static int crashloop_valid(void)
{
    return s_crashloop.magic == PYRO_CRASHLOOP_MAGIC &&
           s_crashloop.checksum == crashloop_checksum(&s_crashloop);
}

static void crashloop_commit(void)
{
    s_crashloop.checksum = crashloop_checksum(&s_crashloop);
    __DSB();
    s_crashloop.magic = PYRO_CRASHLOOP_MAGIC;
    __DSB();
}

void pyro_crashloop_on_fault(uint32_t pc, uint32_t cfsr)
{
    const int valid  = crashloop_valid();
    const int repeat = valid && s_crashloop.last_pc == pc &&
                       s_crashloop.last_cfsr == cfsr;

    /* 首次上电后 .noinit 内容无效，累计计数从 0 开始 */
    if (!valid)
    {
        s_crashloop.total_faults = 0U;
    }

    if (repeat)
    {
        s_crashloop.count++;
    }
    else
    {
        s_crashloop.count        = 1U;
        s_crashloop.first_pc     = pc;
        s_crashloop.first_cfsr   = cfsr;
        s_crashloop.logged_pc    = 0U;
        s_crashloop.logged_cfsr  = 0U;
        s_crashloop.logged_valid = 0U;
    }

    s_crashloop.last_pc   = pc;
    s_crashloop.last_cfsr = cfsr;
    s_crashloop.total_faults++;
    crashloop_commit();
}

uint32_t pyro_crashloop_get_count(void)
{
    return crashloop_valid() ? s_crashloop.count : 0U;
}

uint32_t pyro_crashloop_get_total_faults(void)
{
    return crashloop_valid() ? s_crashloop.total_faults : 0U;
}

uint32_t pyro_crashloop_get_last_pc(void)
{
    return crashloop_valid() ? s_crashloop.last_pc : 0U;
}

uint32_t pyro_crashloop_get_last_cfsr(void)
{
    return crashloop_valid() ? s_crashloop.last_cfsr : 0U;
}

uint32_t pyro_crashloop_get_first_pc(void)
{
    return crashloop_valid() ? s_crashloop.first_pc : 0U;
}

uint32_t pyro_crashloop_get_first_cfsr(void)
{
    return crashloop_valid() ? s_crashloop.first_cfsr : 0U;
}

int pyro_crashloop_should_log(void)
{
    if (!crashloop_valid())
    {
        return 1;
    }
    if (s_crashloop.logged_valid != 1U)
    {
        return 1;
    }

    return !(s_crashloop.logged_pc == s_crashloop.last_pc &&
             s_crashloop.logged_cfsr == s_crashloop.last_cfsr);
}

void pyro_crashloop_mark_logged(void)
{
    if (!crashloop_valid())
    {
        return;
    }

    s_crashloop.logged_pc    = s_crashloop.last_pc;
    s_crashloop.logged_cfsr  = s_crashloop.last_cfsr;
    s_crashloop.logged_valid = 1U;
    crashloop_commit();
}

static int scheduler_is_running(void)
{
#if (INCLUDE_xTaskGetSchedulerState == 1)
    return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
#else
    return 0;
#endif
}

static void ring_lock(void)
{
    if (scheduler_is_running())
    {
        taskENTER_CRITICAL();
    }
}

static void ring_unlock(void)
{
    if (scheduler_is_running())
    {
        taskEXIT_CRITICAL();
    }
}

static uint32_t crashlog_checksum(const pyro_crashlog_record_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t hash = 2166136261UL;
    const size_t begin = offsetof(pyro_crashlog_record_t, line_count);
    const size_t end = offsetof(pyro_crashlog_record_t, lines);

    /* 固定字段：条数、写位置、丢弃数、异常现场 */
    for (size_t i = begin; i < end; ++i)
    {
        hash ^= data[i];
        hash *= 16777619UL;
    }

    /* 日志快照：只覆盖实际写入的字节，不包含槽位残留 */
    for (uint32_t line = 0; line < record->line_count; ++line)
    {
        const char *text = record->lines[line];
        size_t len = 0;
        while (len < PYRO_CRASHLOG_LINE_MAX && text[len] != '\0')
        {
            ++len;
        }
        for (size_t i = 0; i < len; ++i)
        {
            hash ^= (uint8_t)text[i];
            hash *= 16777619UL;
        }
    }
    return hash;
}

/*---------------------------------------------------------------------------*/
/* 对外实现 */
/*---------------------------------------------------------------------------*/

void pyro_crashlog_push(const char *log, size_t size)
{
    if (log == NULL || size == 0)
    {
        return;
    }

    /* 截断到单行上限（预留结尾 '\0'），并去掉日志末尾换行 */
    if (size >= PYRO_CRASHLOG_LINE_MAX)
    {
        size = PYRO_CRASHLOG_LINE_MAX - 1;
    }
    while (size > 0 && (log[size - 1] == '\n' || log[size - 1] == '\r'))
    {
        --size;
    }

    ring_lock();
    char *dst = s_ring[s_ring_head];
    memcpy(dst, log, size);
    dst[size] = '\0';

    s_ring_head = (s_ring_head + 1) % PYRO_CRASHLOG_LINE_NUM;
    if (s_ring_count < PYRO_CRASHLOG_LINE_NUM)
    {
        s_ring_count++;
    }
    else
    {
        s_ring_dropped++;
    }
    ring_unlock();
}

void pyro_crashlog_save_fault(const pyro_crashlog_fault_t *fault)
{
    /*
     * 仅做内存操作，异常上下文安全：
     * 不取锁、不访问 Flash、不做动态分配。
     */
    if (fault != NULL)
    {
        memcpy(&s_crash_record.fault, fault, sizeof(pyro_crashlog_fault_t));
    }
    else
    {
        memset(&s_crash_record.fault, 0, sizeof(pyro_crashlog_fault_t));
    }

    /* 清空旧快照，避免复位前残留内容混入本次崩溃记录 */
    memset(s_crash_record.lines, 0, sizeof(s_crash_record.lines));

    /*
     * 把环形缓冲按时间顺序（最旧 -> 最新）拷贝到记录中。
     * 若未回绕，从 0 开始；若已回绕，从 head 开始。
     */
    const uint32_t count = s_ring_count;
    uint32_t start       = 0;
    if (count == PYRO_CRASHLOG_LINE_NUM)
    {
        start = s_ring_head; /* 已回绕，最旧的在 head */
    }

    for (uint32_t i = 0; i < count; ++i)
    {
        const uint32_t src_idx = (start + i) % PYRO_CRASHLOG_LINE_NUM;
        const size_t len = strnlen(s_ring[src_idx], PYRO_CRASHLOG_LINE_MAX - 1);
        memcpy(s_crash_record.lines[i], s_ring[src_idx], len);
        s_crash_record.lines[i][len] = '\0';
    }

    s_crash_record.line_count = count;
    s_crash_record.write_idx  = s_ring_head;
    s_crash_record.dropped_count = s_ring_dropped;
    s_crash_record.version    = PYRO_CRASHLOG_VERSION;
    s_crash_record.checksum   = crashlog_checksum(&s_crash_record);

    /* 最后写魔数并同步，保证"魔数有效 => 内容完整" */
    __DSB();
    s_crash_record.magic = PYRO_CRASHLOG_MAGIC;
    __DSB();
}

int pyro_crashlog_has_pending(void)
{
    return pyro_crashlog_check_pending();
}

const pyro_crashlog_record_t *pyro_crashlog_get_pending(void)
{
    if (!pyro_crashlog_has_pending())
    {
        return NULL;
    }
    return &s_crash_record;
}

int pyro_crashlog_check_pending(void)
{
    if (s_crash_record.magic != PYRO_CRASHLOG_MAGIC ||
        s_crash_record.version != PYRO_CRASHLOG_VERSION)
    {
        return 0;
    }

    if (s_crash_record.line_count > PYRO_CRASHLOG_LINE_NUM ||
        s_crash_record.write_idx >= PYRO_CRASHLOG_LINE_NUM ||
        s_crash_record.fault.valid != 1U ||
        s_crash_record.checksum != crashlog_checksum(&s_crash_record))
    {
        return 0;
    }

    return 1;
}

uint32_t pyro_crashlog_get_dropped_count(void)
{
    return s_ring_dropped;
}

void pyro_crashlog_clear_pending(void)
{
    s_crash_record.magic = 0;
    __DSB();
}
