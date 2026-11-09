#include "cmsis_os.h"
#include "arm_app.h"
#include "arm_interboard_thread.h"
#include "pyro_task.h"

extern "C"
{
    extern void pyro_init_thread(void *argument);
    void start_mission_planer_task(void const *argument)
    {
        /*
         * LittleFS 写文件时调用链较深
         * （lfs_file_write -> lfs_file_write_ -> ... -> lfs_crc），
         * 实测栈用量约 1.25KB，256 words(1KB) 会栈溢出进入 HardFault。
         * 这里给到 1024 words(4KB)，保证崩溃日志落盘时栈足够。
         */
        xTaskCreate(pyro_init_thread, "pyro_thread_start", 1024,
                nullptr, configMAX_PRIORITIES - 1, nullptr);
        vTaskDelay(1500);
        
#ifdef UES_LOG
        pyro::sys_monitor_start();
#endif
        
        pyro::fsm_task().start();
        pyro::arm_rc_task().start();
        vTaskDelay(20);

        pyro::arm_interboard_task().start();
        vTaskDelay(20);

        vTaskDelete(nullptr);
    }
}
