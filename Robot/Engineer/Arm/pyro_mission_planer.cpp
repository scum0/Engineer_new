#include "cmsis_os.h"
#include "arm_app.h"
#include "arm_interboard_thread.h"

extern "C"
{
    extern void pyro_init_thread(void *argument);
    void start_mission_planer_task(void const *argument)
    {
        xTaskCreate(pyro_init_thread, "pyro_thread_start", 512,
        nullptr, configMAX_PRIORITIES - 1, nullptr);
        vTaskDelay(1500);

        pyro::arm_init_task().start();
        pyro::fsm_init_task().start();
        vTaskDelay(20);

        pyro::arm_interboard_task().start();
        vTaskDelay(20);

        vTaskDelete(nullptr);
    }
}
