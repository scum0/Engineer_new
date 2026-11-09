#ifndef __PYRO_CORE_CONFIG_H__
#define __PYRO_CORE_CONFIG_H__

/*
 * PYRo 驱动日志开关：
 * 取消下一行注释，或在构建系统中添加 -DUES_LOG，即可输出驱动日志。
 * 保持注释时，驱动日志不会编译进固件，行为与未加日志时一致。
 */
// #define UES_LOG

#define PYRO_UART1 pyro::bsp_uart::get_uart1()
#define PYRO_UART5 pyro::bsp_uart::get_uart5()
#define PYRO_UART7 pyro::bsp_uart::get_uart7()
#define PYRO_UART10 pyro::bsp_uart::get_uart10()

#endif // PYRO_PYRO_CORE_CONFIG_H
