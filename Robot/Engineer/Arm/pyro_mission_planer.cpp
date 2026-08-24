#include "cmsis_os.h"

extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void engineer_arm_init(void* args);
    void start_mission_planer_task(void const *argument)
    {
        pyro_init_thread(nullptr);
        vTaskDelay(10);
        xTaskCreate(engineer_arm_init, "engineer_arm_init", 2048, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelay(20);        
        vTaskDelete(nullptr);
    }
}