/**
* @file pyro_bsp_uart.cpp
 * @brief Implementation of the Board Support Package for UART.
 * 串口板级支持包实现文件。
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2026-03-25
 */

#include "pyro_bsp_uart.h"
#include "usart.h"
#include "pyro_drv_log.h"

namespace pyro
{

// Using local static variables to ensure a safe initialization order.
// 使用局部静态变量以确保安全的初始化顺序。

uart_drv_t& bsp_uart::get_uart1()
{
    static uart_drv_t instance(&huart1, 512);
    return instance;
}

uart_drv_t& bsp_uart::get_uart5()
{
    // Custom DMA buffer size can be configured here.
    // 可以在此处配置自定义的 DMA 缓冲区大小。
    static uart_drv_t instance(&huart5, 512);
    return instance;
}

uart_drv_t& bsp_uart::get_uart7()
{
    static uart_drv_t instance(&huart7, 512);
    return instance;
}

uart_drv_t& bsp_uart::get_uart10()
{
    static uart_drv_t instance(&huart10, 512);
    return instance;
}

void bsp_uart::init_all()
{
    // Calling get_uartX() triggers the first-time construction of the instance.
    // 调用 get_uartX() 将触发实例的首次构造。
    if (get_uart1().enable_rx_dma() != PYRO_OK)
        PYRO_DRV_LOGW("UART", "uart1 enable rx dma failed");
    if (get_uart5().enable_rx_dma() != PYRO_OK)
        PYRO_DRV_LOGW("UART", "uart5 enable rx dma failed");
    if (get_uart7().enable_rx_dma() != PYRO_OK)
        PYRO_DRV_LOGW("UART", "uart7 enable rx dma failed");
    if (get_uart10().enable_rx_dma() != PYRO_OK)
        PYRO_DRV_LOGW("UART", "uart10 enable rx dma failed");
}

} // namespace pyro
