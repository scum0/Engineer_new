/**
 * @file pyro_hardfault.h
 * @brief HardFault 现场捕获与打印
 *
 * 在 HardFault_Handler 中调用 pyro_hardfault_handler()，
 * 打印异常现场寄存器、故障原因寄存器以及当前任务名。
 */

#ifndef __PYRO_HARDFAULT_H__
#define __PYRO_HARDFAULT_H__

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief HardFault 处理入口
 * @param stack_frame 指向异常压栈现场的指针（由汇编包装层传入）
 * @note 该函数不会返回：现场写入 .noinit 后主动复位
 */
void pyro_hardfault_handler(void *stack_frame);

/**
 * @brief 主动触发一次 HardFault，用于上板验证崩溃日志链路
 * @note 仅用于调试；触发后先写 .noinit，再自动复位并落盘 /crash.log
 */
void pyro_hardfault_trigger(void);

__attribute__((naked)) void pyro_hardfault_entry(void);

#ifdef __cplusplus
}
#endif

#endif /* __PYRO_HARDFAULT_H__ */
