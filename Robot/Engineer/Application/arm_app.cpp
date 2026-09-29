#include "arm_module.h"
#include "pyro_rc_base_drv.h"
#include "fsm/arm_fsm.h"
#include "pyro_rc_core.h"

namespace pyro
{

static TaskHandle_t arm_task_handle                = nullptr;
static TaskHandle_t fsm_task_handle                = nullptr;
static arm_top_fsm_t main_fsm;

arm_module_t *arm_module_ptr = nullptr;
arm_cmd_t *arm_cmd_ptr = nullptr;
arm_deps_t *arm_deps = nullptr;
fsm_ctx_t *fsm_ctx_ptr = nullptr;

constexpr uint32_t EVENT_BIT_RESET                = (1 << 0);
constexpr uint32_t EVENT_BIT_SELF_CTRL            = (1 << 1);

void deps_init();

void arm_rc_thread(void *argument)
{
    for (;;)
    {
        uint32_t notify_val = 0;
        xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

        // 检测按键 R 触发
        if (notify_val & EVENT_BIT_RESET)
        {
            if (eTaskGetState(fsm_task_handle) == eSuspended)
                vTaskResume(fsm_task_handle);

            main_fsm.change_state(&main_fsm.idle_state);
        }
        if (notify_val & EVENT_BIT_SELF_CTRL)
        {
            if (eTaskGetState(fsm_task_handle) == eSuspended)
                vTaskResume(fsm_task_handle);
            
            main_fsm.change_state(&main_fsm.self_ctrl_state);
        }
    }
}

void fsm_thread(void *argument)
{
    for(;;)
    {
        if(fsm_ctx_ptr != nullptr)
        {
            main_fsm.execute(fsm_ctx_ptr);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

extern "C" void arm_init()
{
    arm_cmd_ptr = new arm_cmd_t();
    deps_init();
    arm_module_ptr = arm_module_t::instance();
    arm_module_ptr->configure(*arm_deps);

    auto &vrc = pyro::rc_drv_t::read();
    pyro::btn_broker::subscribe(&vrc.keys.r, pyro::btn_event_t::PRESS_DOWN,
                                    arm_task_handle, EVENT_BIT_RESET);
    pyro::btn_broker::subscribe(&vrc.keys.x, pyro::btn_event_t::PRESS_DOWN,
                                arm_task_handle, EVENT_BIT_SELF_CTRL);

    vTaskDelay(1500);
    arm_module_ptr->start();
    xTaskCreate(arm_rc_thread,"arm_rc_thread",512,nullptr,
                configMAX_PRIORITIES - 2, &arm_task_handle);

    vTaskDelete(nullptr);
}

extern "C" void fsm_init()
{
    fsm_ctx_ptr = new fsm_ctx_t();

    // 设置FSM初始状态：idle_state
    main_fsm.change_state(&main_fsm.idle_state);

    xTaskCreate(fsm_thread,"fsm_thread_start",512,nullptr,
                configMAX_PRIORITIES - 2, &fsm_task_handle);

    vTaskDelete(nullptr);
}

void deps_init()
{
    arm_deps = new arm_deps_t();
    arm_deps->motor[0] = new dm_motor_drv_t(0x2, 0x1, bsp_can::can1);
    arm_deps->motor[1] = new dm_motor_drv_t(0x4, 0x3, bsp_can::can1);
    arm_deps->motor[2] = new dm_motor_drv_t(0x6, 0x5, bsp_can::can3);
    arm_deps->motor[3] = new dm_motor_drv_t(0x8, 0x7, bsp_can::can3);
    arm_deps->motor[4] = new dm_motor_drv_t(0xa, 0x9, bsp_can::can2);
    arm_deps->motor[5] = new dm_motor_drv_t(0xc, 0xb, bsp_can::can2);
    arm_deps->motor[6] = new dm_motor_drv_t(0xe, 0xd, bsp_can::can2);


    arm_deps->motor[0]->set_position_range(-PI,PI);
    arm_deps->motor[0]->set_rotate_range(-52,52);
    arm_deps->motor[0]->set_torque_range(-27,27);

    arm_deps->motor[1]->set_position_range(-PI,PI);
    arm_deps->motor[1]->set_rotate_range(-150,150);
    arm_deps->motor[1]->set_torque_range(-150,150);

    arm_deps->motor[2]->set_position_range(-PI,PI);
    arm_deps->motor[2]->set_rotate_range(-160,160);
    arm_deps->motor[2]->set_torque_range(-40,40);

    arm_deps->motor[3]->set_position_range(-PI,PI);
    arm_deps->motor[3]->set_rotate_range(-200,200);
    arm_deps->motor[3]->set_torque_range(-7,7);

    arm_deps->motor[4]->set_position_range(-PI,PI);
    arm_deps->motor[4]->set_rotate_range(-200,200);
    arm_deps->motor[4]->set_torque_range(-7,7);

    arm_deps->motor[5]->set_position_range(-PI,PI);
    arm_deps->motor[5]->set_rotate_range(-200,200);
    arm_deps->motor[5]->set_torque_range(-7,7);

    arm_deps->motor[6]->set_position_range(-PI,PI);
    arm_deps->motor[6]->set_rotate_range(-200,200);
    arm_deps->motor[6]->set_torque_range(-7,7);


    arm_deps->pos_pid[0] = new pid_t(0,0.0,0.0,0.0,52);
    arm_deps->rot_pid[0] = new pid_t(8.8,12.0,0.0,0.0,27);

    arm_deps->pos_pid[1] = new pid_t(20,0,0.0,20.0,150);
    arm_deps->rot_pid[1] = new pid_t(20,0,0.0,20.0,150);

    arm_deps->pos_pid[2] = new pid_t(15,0.0,0.0,0.0,160);
    arm_deps->rot_pid[2] = new pid_t(11,0.2,0.0,5.0,40);

    arm_deps->pos_pid[3] = new pid_t(15.7,0,0.0,6,200);
    arm_deps->rot_pid[3] = new pid_t(1.0,0.2,0.001,3,7);

    arm_deps->pos_pid[4] = new pid_t(10.3,0.1,0.0,20,200);
    arm_deps->rot_pid[4] = new pid_t(1.0,0.01,0.00,4,7);

    arm_deps->pos_pid[5] = new pid_t(9,0.0,0.0,0.0,200);
    arm_deps->rot_pid[5] = new pid_t(0.8,0.1,0.0,1,7);

    arm_deps->pos_pid[6] = new pid_t(9,0.0,0.0,0,200);
    arm_deps->rot_pid[6] = new pid_t(0.8,0.0,0.0,0,7);


    arm_deps->axis[0] = new arm_deps_t::axis_deps_t(arm_deps->motor[0], arm_deps->pos_pid[0],arm_deps->rot_pid[0]);
    arm_deps->axis[1] = new arm_deps_t::axis_deps_t(arm_deps->motor[1], arm_deps->pos_pid[1],arm_deps->rot_pid[1]);
    arm_deps->axis[2] = new arm_deps_t::axis_deps_t(arm_deps->motor[2], arm_deps->pos_pid[2],arm_deps->rot_pid[2]);
    arm_deps->axis[3] = new arm_deps_t::axis_deps_t(arm_deps->motor[3], arm_deps->pos_pid[3],arm_deps->rot_pid[3]);
    arm_deps->axis[4] = new arm_deps_t::axis_deps_t(arm_deps->motor[4], arm_deps->pos_pid[4],arm_deps->rot_pid[4]);
    arm_deps->axis[5] = new arm_deps_t::axis_deps_t(arm_deps->motor[5], arm_deps->pos_pid[5],arm_deps->rot_pid[5]);
    arm_deps->axis[6] = new arm_deps_t::axis_deps_t(arm_deps->motor[6], arm_deps->pos_pid[6],arm_deps->rot_pid[6]);

    arm_deps->axis[0]->enable_constraint();
    arm_deps->axis[1]->enable_constraint();
    arm_deps->axis[2]->enable_constraint();
    arm_deps->axis[3]->disable_constraint();    // R4 无限位关节
    arm_deps->axis[4]->enable_constraint();
    arm_deps->axis[5]->enable_constraint();
    arm_deps->axis[6]->enable_constraint();

    arm_deps->axis[0]->set_limit(-2.1599f, PI);
    arm_deps->axis[1]->set_limit(-0.3f, 1.0f);
    arm_deps->axis[2]->set_limit(-0.3f, 1.35f);
    arm_deps->axis[3]->set_limit(-0.0f, 0.0f);
    arm_deps->axis[4]->set_limit(-0.5f*PI, 0.5f*PI);
    arm_deps->axis[5]->set_limit(-0.5f*PI, 0.5f*PI);
    arm_deps->axis[6]->set_limit(-PI, 0.0f);

    arm_deps->axis[0]->set_feedback_pos_offset(0.65045f);
    arm_deps->axis[1]->set_feedback_pos_offset(-2.3948f);
    arm_deps->axis[2]->set_feedback_pos_offset(-0.328f);
    arm_deps->axis[3]->set_feedback_pos_offset(2.34739f);
    arm_deps->axis[4]->set_feedback_pos_offset(2.46375f);
    arm_deps->axis[5]->set_feedback_pos_offset(2.74246f);
    arm_deps->axis[6]->set_feedback_pos_offset(1.8587f);
}

}