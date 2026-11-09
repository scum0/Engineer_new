#include "pyro_bsp_uart.h"
#include "pyro_bsp_can.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_image_drv.h"
#include "pyro_ins.h"
#include "pyro_supercap_drv.h"
#include "pyro_referee.h"
#include "pyro_sr04_drv.h"
#include "pyro_sr05_drv.h"
#include "pyro_us100_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_log.h"
#include "pyro_flash_fs.h"
#include "elog.h"
#include "log.h"
#include <cstdio>

/**
 * @brief 库内日志适配器：将 PYRo 库日志回调转发到 EasyLogger
 * @note 库内模块只依赖 pyro_log.h，具体日志实现由应用层在此注入
 */
static void pyro_log_to_elog(pyro_log_level_t level, const char *tag,
                             const char *format, va_list args)
{
    char buf[ELOG_LINE_BUF_SIZE];
    vsnprintf(buf, sizeof(buf), format, args);

    /* 库内等级与 EasyLogger 等级一一对应 */
    elog_output(DEFAULT_TERMINAL, (uint8_t)level, tag, NULL, NULL, 0, "%s", buf);
}

namespace pyro
{
extern "C"
{
    void pyro_init_thread(void *argument)
    {
        /* 注入库内日志实现，必须在驱动初始化之前完成 */
        pyro_log_set_handler(pyro_log_to_elog);

        dwt_drv_t::init(480); // Initialize DWT at 480 MHz
        bsp_can::init_all(); // Initialize all CAN instances
        pyro::bsp_uart::init_all();

        /* 初始化外部 Flash 文件系统（由 ELOG_OUTPUT_BACKEND_FLASH 控制） */
        pyro_flash_fs_init();

#ifdef DR16_UART
        dr16_drv_t::instance().start();
        dr16_drv_t::instance().enable();
        DR16_UART.reset(100000, UART_WORDLENGTH_9B, UART_STOPBITS_2,
                        UART_PARITY_EVEN);
        DR16_UART.enable_rx_dma();
#endif

#ifdef VT03_UART
        vt03_drv_t::instance().start();
        vt03_drv_t::instance().enable();
        VT03_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                        UART_PARITY_NONE);
        VT03_UART.enable_rx_dma();
        rc_drv_t::init_virtual_rc();
#endif

#ifdef REFEREE_UART
        REFEREE_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                           UART_PARITY_NONE);
        REFEREE_UART.enable_rx_dma();
        referee_drv_t::get_instance()->init();
#endif

#ifdef SUPERCAP_UART
        SUPERCAP_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                            UART_PARITY_NONE);
        SUPERCAP_UART.enable_rx_dma();
        supercap_drv_t::get_instance()->start_rx();
#endif

#ifdef AUTOAIM_UART
        AUTOAIM_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                           UART_PARITY_NONE);
        AUTOAIM_UART.enable_rx_dma();
#endif

#ifdef IMAGE_UART
        IMAGE_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                         UART_PARITY_NONE);
        image_drv_t::get_instance().start();
        image_drv_t::get_instance().init();
#endif

#ifdef CUSTOM_UART
        CUSTOM_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                          UART_PARITY_NONE);
#endif

#ifdef SR04_UART
        SR04_UART.reset(9600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                        UART_PARITY_NONE);
        SR04_UART.enable_rx_dma();
        sr04_drv::get_instance().init();
        sr04_drv::get_instance().start();
#endif

#ifdef US100_UART
        US100_UART.reset(9600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                         UART_PARITY_NONE);
        us100_drv::get_instance().init();
        us100_drv::get_instance().start();
#endif

#ifdef SR05_UART
        SR05_UART.set_level_invert(false,true);
        SR05_UART.reset(9600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                                UART_PARITY_NONE);
        SR05_UART.enable_rx_dma();
        sr05_drv::get_instance().init();
        sr05_drv::get_instance().start();
#endif
        vTaskDelete(nullptr);
    }
}

} // namespace pyro
