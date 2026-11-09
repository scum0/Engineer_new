/**
 * @file pyro_hardfault.c
 * @brief HardFault 现场捕获与打印实现
 *
 * 设计要点：
 *  1. 用 naked 汇编包装层在进入 C 代码前判断异常使用的是 MSP 还是 PSP，
 *     并把对应的压栈现场指针传给 C 处理函数，避免 C 代码破坏现场。
 *  2. 直接使用 SEGGER RTT 输出，绕开日志锁（异常上下文里日志锁可能死锁）。
 *  3. 打印异常压栈寄存器、故障原因寄存器（CFSR/HFSR/BFAR/MMFAR）以及当前任务名。
 *  4. 把 RAM 环形日志与异常现场写入 .noinit，然后主动复位。
 *  5. 复位后由文件系统把 .noinit 中的记录写入 /crash.log。
 */

#include "pyro_hardfault.h"
#include "pyro_crashlog.h"

#include "elog_cfg.h"
#include "SEGGER_RTT.h"

#include "stm32h7xx.h"
#include "stm32h7xx_hal_cortex.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* 异常压栈的寄存器现场（硬件自动压栈顺序） */
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
} hardfault_stack_frame_t;

/* Cortex-M 故障状态寄存器 */
#define SCB_CFSR   (*(volatile uint32_t *)0xE000ED28UL)
#define SCB_HFSR   (*(volatile uint32_t *)0xE000ED2CUL)
#define SCB_MMFAR  (*(volatile uint32_t *)0xE000ED34UL)
#define SCB_BFAR   (*(volatile uint32_t *)0xE000ED38UL)

/* CFSR 子字段有效位 */
#define CFSR_MMARVALID  (1UL << 7)
#define CFSR_BFARVALID  (1UL << 15)

/* 主动复位前给 RTT 主机读取现场的时间 */
#define HARDFAULT_RTT_TIMEOUT_MS 100U

/*
 * 同一故障最多尝试复位落盘的次数。
 * 超过后不再复位，避免"崩溃 -> 复位 -> 再次崩溃"的无限重启。
 */
#define PYRO_CRASH_RESET_MAX 3U

/*
 * 本次上电累计 HardFault 上限（不区分现场）。
 * 防止不同 PC 的故障交替出现时，绕过同一现场的去重而无限复位。
 */
#define PYRO_CRASH_TOTAL_MAX 5U

static volatile bool s_rtt_ready = false;
volatile uint32_t g_hardfault_rtt_last_status = 0U;
volatile uint32_t g_hardfault_rtt_last_length = 0U;
volatile uint32_t g_hardfault_rtt_avail_before = 0U;
volatile uint32_t g_hardfault_rtt_avail_after = 0U;

static void hardfault_rtt_write_literal(const char *text)
{
    size_t len = 0;
    while (text[len] != '\0')
    {
        ++len;
    }
    g_hardfault_rtt_last_length = (uint32_t)len;
    g_hardfault_rtt_last_status =
        SEGGER_RTT_WriteNoLock(0, text, (unsigned)len);
}

static void hardfault_rtt_write(const char *text)
{
    if (!s_rtt_ready || text == NULL)
    {
        return;
    }

    hardfault_rtt_write_literal(text);
}

static void hardfault_rtt_hex(const char *prefix, uint32_t value,
                              const char *suffix)
{
    static const char hex[] = "0123456789ABCDEF";
    char text[64];
    size_t off = 0;

    if (!s_rtt_ready)
    {
        return;
    }

    while (prefix != NULL && *prefix != '\0' && off < sizeof(text) - 12U)
    {
        text[off++] = *prefix++;
    }

    text[off++] = '0';
    text[off++] = 'x';
    for (int shift = 28; shift >= 0; shift -= 4)
    {
        text[off++] = hex[(value >> shift) & 0x0FU];
    }

    while (suffix != NULL && *suffix != '\0' && off < sizeof(text) - 1U)
    {
        text[off++] = *suffix++;
    }
    text[off] = '\0';

    hardfault_rtt_write(text);
}

/**
 * @brief 打印故障原因寄存器
 * @note 仅在 MMARVALID / BFARVALID 置位时，MMFAR / BFAR 的内容才有意义
 */
static void print_fault_status(void)
{
    const uint32_t cfsr = SCB_CFSR;

    hardfault_rtt_hex("CFSR  = ", cfsr, "\r\n");
    hardfault_rtt_hex("HFSR  = ", SCB_HFSR, "\r\n");

    if (cfsr & CFSR_MMARVALID)
    {
        hardfault_rtt_hex("MMFAR = ", SCB_MMFAR,
                          " (invalid memory access)\r\n");
    }
    if (cfsr & CFSR_BFARVALID)
    {
        hardfault_rtt_hex("BFAR  = ", SCB_BFAR,
                          " (invalid bus access)\r\n");
    }
}

void pyro_hardfault_handler(void *stack_frame)
{
    const hardfault_stack_frame_t *frame =
        (const hardfault_stack_frame_t *)stack_frame;

    /* 记录本次现场：同一 PC + CFSR 反复出现时不再重复擦写 Flash */
    pyro_crashloop_on_fault(frame->pc, SCB_CFSR);

    /* 先保存现场，确保即使 RTT 输出阻塞，崩溃记录也已写入 RAM */
    {
        pyro_crashlog_fault_t fault = {0};
        fault.r0    = frame->r0;
        fault.r1    = frame->r1;
        fault.r2    = frame->r2;
        fault.r3    = frame->r3;
        fault.r12   = frame->r12;
        fault.lr    = frame->lr;
        fault.pc    = frame->pc;
        fault.psr   = frame->psr;
        fault.cfsr  = SCB_CFSR;
        fault.hfsr  = SCB_HFSR;
        fault.bfar  = SCB_BFAR;
        fault.mmar  = SCB_MMFAR;
        fault.valid = 1U;

        /* 异常现场不调用 FreeRTOS API，避免 TCB 无效时触发 configASSERT */
        memcpy(fault.task_name, "(unknown task)", sizeof("(unknown task)"));

        pyro_crashlog_save_fault(&fault);
    }

    /*
     * 调试器未连接时 RTT 缓冲区可能已满。此时跳过打印，直接复位，
     * 避免异常处理被 RTT 阻塞而无法落盘。
     */
    const size_t rtt_text_size = 1024U;
    g_hardfault_rtt_avail_before = SEGGER_RTT_GetAvailWriteSpace(0);
    s_rtt_ready = g_hardfault_rtt_avail_before >= rtt_text_size;
    if (s_rtt_ready)
    {
        hardfault_rtt_write_literal("\r\n");
        hardfault_rtt_write_literal(
            "==================== HARDFAULT ====================\r\n");
    }

    /* 异常压栈的寄存器现场 */
    hardfault_rtt_hex("R0  = ", frame->r0, "\r\n");
    hardfault_rtt_hex("R1  = ", frame->r1, "\r\n");
    hardfault_rtt_hex("R2  = ", frame->r2, "\r\n");
    hardfault_rtt_hex("R3  = ", frame->r3, "\r\n");
    hardfault_rtt_hex("R12 = ", frame->r12, "\r\n");
    hardfault_rtt_hex("LR  = ", frame->lr, "\r\n");
    hardfault_rtt_hex("PC  = ", frame->pc, "\r\n");
    hardfault_rtt_hex("PSR = ", frame->psr, "\r\n");

    /* 崩溃循环信息：重复次数与首次现场（根因） */
    hardfault_rtt_hex("CrashCnt = ", pyro_crashloop_get_count(), "\r\n");
    hardfault_rtt_hex("FirstPC  = ", pyro_crashloop_get_first_pc(), "\r\n");
    hardfault_rtt_hex("FirstCFSR= ", pyro_crashloop_get_first_cfsr(), "\r\n");

    /* 故障原因寄存器 */
    print_fault_status();

    /* 任务名从已保存的异常记录读取，不调用 FreeRTOS API */
    {
        const pyro_crashlog_record_t *record = pyro_crashlog_get_pending();
        if (record != NULL)
        {
            hardfault_rtt_write("Task  = ");
            hardfault_rtt_write(record->fault.task_name);
            hardfault_rtt_write("\r\n");
        }
    }

    hardfault_rtt_write(
        "===================================================\r\n");
    g_hardfault_rtt_avail_after = SEGGER_RTT_GetAvailWriteSpace(0);

    /* 给 RTT 主机少量时间取走现场，不阻塞在 RTT 内部 */
    if (s_rtt_ready)
    {
        for (volatile uint32_t delay = 0;
             delay < (HARDFAULT_RTT_TIMEOUT_MS * 1000U); ++delay)
        {
        }
    }

    /*
     * 只有 Flash 后端需要在复位后把 .noinit 落盘到 /crash.log。
     * RTT / UART 后端已经在现场实时输出，复位只会形成崩溃循环并丢失现场，
     * 因此不复位，直接停在下面死循环等待调试器或断电。
     *
     * Flash 后端也不是无条件复位：
     *  - 本次现场还没落盘，且重试次数未超限：复位，让下次启动写 /crash.log；
     *  - 同一现场已经落过盘，或复位重试次数超限：说明复位后仍会复现，
     *    继续复位只会无限重启，因此停在现场等待调试器或断电。
     */
#if defined(ELOG_OUTPUT_BACKEND_FLASH)
    if (pyro_crashloop_should_log() != 0 &&
        pyro_crashloop_get_count() < PYRO_CRASH_RESET_MAX &&
        pyro_crashloop_get_total_faults() < PYRO_CRASH_TOTAL_MAX)
    {
        __DSB();
        __ISB();
        NVIC_SystemReset();
    }

    hardfault_rtt_write(
        "[crashloop] fault already logged or reset limit reached, stop "
        "resetting.\r\n");
#endif

    while (1)
    {
        __NOP();
    }
}

/*
 * HardFault 入口汇编包装层。
 *
 * 进入异常时硬件把现场压在"当前使用的栈"上：
 *   - EXC_RETURN 的 bit2 = 0 表示使用 MSP
 *   - EXC_RETURN 的 bit2 = 1 表示使用 PSP
 * 这里在调用 C 处理函数之前先判断，并把对应的栈指针作为第一个参数传入。
 *
 * 用 naked 属性保证编译器不插入会破坏寄存器的序言代码。
 *
 * @note 函数名刻意不叫 HardFault_Handler：CubeMX 重新生成 stm32h7xx_it.c 时
 *       会重新生成一个普通 C 版本的 HardFault_Handler。为避免符号冲突，
 *       本函数命名为 pyro_hardfault_entry，并由 startup 的向量表直接指向它；
 *       CubeMX 生成的 HardFault_Handler 因无人引用会被 --gc-sections 丢弃。
 */
__attribute__((naked)) void pyro_hardfault_entry(void)
{
    __asm volatile(
        "tst    lr, #4              \n" /* 测试 EXC_RETURN bit2 */
        "ite    eq                  \n"
        "mrseq  r0, msp             \n" /* bit2=0: 用 MSP */
        "mrsne  r0, psp             \n" /* bit2=1: 用 PSP */
        "b      pyro_hardfault_handler \n");
}

void pyro_hardfault_trigger(void)
{
    /* 临时把地址 0 设为只读，随后写入该地址产生 HardFault */
    MPU_Region_InitTypeDef region = {0};
    region.Enable            = MPU_REGION_ENABLE;
    region.Number            = MPU_REGION_NUMBER15;
    region.BaseAddress       = 0U;
    region.Size              = MPU_REGION_SIZE_32B;
    region.SubRegionDisable  = 0U;
    region.TypeExtField      = MPU_TEX_LEVEL0;
    region.AccessPermission  = MPU_REGION_PRIV_RO;
    region.DisableExec       = MPU_INSTRUCTION_ACCESS_DISABLE;
    region.IsShareable       = MPU_ACCESS_NOT_SHAREABLE;
    region.IsCacheable       = MPU_ACCESS_NOT_CACHEABLE;
    region.IsBufferable      = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&region);
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
    HAL_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

    /* 写入只读地址，由 MPU 产生 MemManage 并升级为 HardFault */
    *(volatile uint32_t *)0x00000000UL = 0U;
    __DSB();
    __ISB();
}
