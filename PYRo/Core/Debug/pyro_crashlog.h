/**
 * @file pyro_crashlog.h
 * @brief 崩溃日志：RAM 环形缓冲 + HardFault 落 .noinit + 复位后落 Flash
 *
 * 数据流：
 *   平时   : 日志写入 RAM 环形缓冲（满了丢弃最旧的），不碰 Flash
 *   HardFault: 把环形缓冲内容 + 异常现场写入 .noinit RAM（安全、快）
 *   复位后 : 应用层检查 .noinit，若有崩溃记录，用文件系统写入 /crash.log
 */

#ifndef __PYRO_CRASHLOG_H__
#define __PYRO_CRASHLOG_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* 环形缓冲单条日志最大长度（含结尾 '\0'） */
#define PYRO_CRASHLOG_LINE_MAX   256
/* 环形缓冲可保存的日志条数 */
#define PYRO_CRASHLOG_LINE_NUM   32
/* 异常现场保存的任务名最大长度（含结尾 '\0'） */
#define PYRO_CRASHLOG_TASK_NAME_MAX 24

/* 崩溃记录魔数，用于判断 .noinit 中是否存在有效记录 */
#define PYRO_CRASHLOG_MAGIC      0x43524C47U /* "CRLG" */
/* 记录结构版本，结构变化时递增，避免复位后误读旧布局 */
#define PYRO_CRASHLOG_VERSION    2U

/* 异常现场 */
typedef struct
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t bfar;
    uint32_t mmar;
    char     task_name[PYRO_CRASHLOG_TASK_NAME_MAX];
    uint32_t valid; /* 1 = 现场有效 */
} pyro_crashlog_fault_t;

/* 崩溃记录（放在 .noinit，复位不清零） */
typedef struct
{
    uint32_t magic;                              /* PYRO_CRASHLOG_MAGIC 表示有效 */
    uint32_t version;                            /* PYRO_CRASHLOG_VERSION */
    uint32_t checksum;                           /* 记录内容校验和 */
    uint32_t line_count;                         /* 有效日志条数 */
    uint32_t write_idx;                          /* 环形缓冲写入位置（用于调试） */
    uint32_t dropped_count;                      /* 异常前因缓冲满而丢弃的条数 */
    pyro_crashlog_fault_t fault;                 /* 异常现场 */
    char     lines[PYRO_CRASHLOG_LINE_NUM][PYRO_CRASHLOG_LINE_MAX]; /* 日志快照 */
} pyro_crashlog_record_t;

/**
 * @brief 向 RAM 环形缓冲追加一条日志（满了覆盖最旧的）
 * @param log  日志内容
 * @param size 长度（超过 PYRO_CRASHLOG_LINE_MAX 会被截断）
 * @note 线程安全由调用方保证（日志后端在锁内调用）
 */
void pyro_crashlog_push(const char *log, size_t size);

/**
 * @brief HardFault 时调用：把环形缓冲 + 异常现场写入 .noinit
 * @param fault 异常现场（由 HardFault 处理器填充）
 * @note 仅做内存操作，异常上下文安全
 */
void pyro_crashlog_save_fault(const pyro_crashlog_fault_t *fault);

/**
 * @brief 检查 .noinit 中是否有待落盘的崩溃记录
 * @return 1 有，0 无
 */
int pyro_crashlog_has_pending(void);

/**
 * @brief 获取待落盘的崩溃记录（只读）
 * @return 记录指针；无记录时返回 NULL
 */
const pyro_crashlog_record_t *pyro_crashlog_get_pending(void);

/**
 * @brief 启动时检查 .noinit 中的崩溃记录是否有效
 * @return 1 有效，0 无记录或记录损坏
 * @note 有效记录保留到成功写入 /crash.log 后清除
 */
int pyro_crashlog_check_pending(void);

/**
 * @brief 获取 RAM 环形缓冲因满而丢弃的日志条数
 * @return 丢弃条数；复位后清零
 */
uint32_t pyro_crashlog_get_dropped_count(void);

/**
 * @brief 清除待落盘标志（落盘完成后调用）
 */
void pyro_crashlog_clear_pending(void);

/*---------------------------------------------------------------------------*/
/* 崩溃循环检测：同一现场反复崩溃时，避免重复擦写 Flash */
/*---------------------------------------------------------------------------*/

/**
 * @brief 记录一次 HardFault 现场（在 pyro_hardfault_handler 中调用）
 * @param pc   异常发生时的 PC
 * @param cfsr 故障状态寄存器 CFSR
 * @note PC 与 CFSR 都和上次相同时，视为同一次故障重复发生，计数累加
 */
void pyro_crashloop_on_fault(uint32_t pc, uint32_t cfsr);

/* @brief 当前连续崩溃次数（同一现场重复发生） */
uint32_t pyro_crashloop_get_count(void);

/* @brief 本次上电以来累计 HardFault 次数（不区分现场，断电清零） */
uint32_t pyro_crashloop_get_total_faults(void);

/* @brief 最近一次崩溃现场 */
uint32_t pyro_crashloop_get_last_pc(void);
uint32_t pyro_crashloop_get_last_cfsr(void);

/* @brief 本轮崩溃循环的首次现场，用于保留根因 */
uint32_t pyro_crashloop_get_first_pc(void);
uint32_t pyro_crashloop_get_first_cfsr(void);

/**
 * @brief 当前现场是否还需要写入 /crash.log
 * @return 1 需要落盘，0 相同现场已经落过盘
 */
int pyro_crashloop_should_log(void);

/**
 * @brief 标记当前现场已成功写入 /crash.log
 */
void pyro_crashloop_mark_logged(void);

#ifdef __cplusplus
}
#endif

#endif /* __PYRO_CRASHLOG_H__ */
