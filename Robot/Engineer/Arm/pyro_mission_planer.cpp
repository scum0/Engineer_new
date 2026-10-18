#include "pyro_mission_planer.h"

#include "arm_app.h"
#include "arm_interboard_thread.h"
#include "pyro_init_thread.h"

#include "cmsis_os.h"

namespace pyro
{

mission_planer_task_t::mission_planer_task_t()
    : task_base_t("mission_planer", 512, 512, priority_t::REALTIME)
{
}

status_t mission_planer_task_t::init()
{
    return PYRO_OK;
}

void mission_planer_task_t::run_loop()
{
    // 1. 外设初始化
    pyro_init_task().start();
    vTaskDelay(1500);

    // 2. 机械臂与状态机
    arm_init_task().start();
    fsm_init_task().start();
    vTaskDelay(20);

    // 3. 板间通信
    arm_interboard_task().start();
    vTaskDelay(20);
}

mission_planer_task_t &mission_planer_task()
{
    static mission_planer_task_t instance;
    return instance;
}

} // namespace pyro

extern "C" void start_mission_planer_task(void const *argument)
{
    (void)argument;

    pyro::mission_planer_task().start();

    vTaskDelete(nullptr);
}
