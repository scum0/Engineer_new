#include "cmsis_os.h"

extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void arm_init(void* args);
    extern void fsm_init(void* args);
    extern void self_control_thread(void* args);
    void start_mission_planer_task(void const *argument)
    {
        xTaskCreate(pyro_init_thread, "pyro_thread_start", 512,
        nullptr, configMAX_PRIORITIES - 1, nullptr);
        vTaskDelay(10);

        xTaskCreate(arm_init, "arm_thread_start", 512,
        nullptr, configMAX_PRIORITIES - 1, nullptr);
        vTaskDelay(20);

        xTaskCreate(fsm_init, "fsm_thread_start", 512,
        nullptr, configMAX_PRIORITIES - 1, nullptr);
        vTaskDelay(20);

        xTaskCreate(self_control_thread, "self_control_thread", 512,
        nullptr, configMAX_PRIORITIES - 2, nullptr);
        vTaskDelay(20);

        vTaskDelete(nullptr);
    }
}