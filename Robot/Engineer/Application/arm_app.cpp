#include "arm_module.h"
#include "pyro_rc_base_drv.h"
#include "arm_fsm.h"
#include "arm_app.h"
#include "pyro_rc_core.h"
#include "task.h"
namespace pyro
{

static arm_top_fsm_t main_fsm;

arm_cmd_t *arm_cmd_ptr = nullptr;
arm_deps_t *arm_deps = nullptr;
fsm_ctx_t *fsm_ctx_ptr = nullptr;

constexpr uint32_t EVENT_BIT_RESET                = (1 << 0);
constexpr uint32_t EVENT_BIT_NORMAL               = (1 << 1);
constexpr uint32_t EVENT_BIT_SELF_CTRL            = (1 << 2);
constexpr uint32_t EVENT_BIT_TURN_GRIPPER         = (1 << 3);

void deps_init();

// 状态机任务的句柄，供遥控器任务做挂起/恢复判断
static TaskHandle_t s_fsm_task_handle = nullptr;

// ==================== 遥控器事件处理任务 ====================
arm_rc_task_t::arm_rc_task_t()
    : task_base_t("arm_rc_task", 256, 256, priority_t::BELOW_NORMAL)
{
}

status_t arm_rc_task_t::init()
{
    return PYRO_OK;
}

void arm_rc_task_t::run_loop()
{
    // 在任务上下文中获取自身句柄，并订阅遥控器按键事件
    const TaskHandle_t self = xTaskGetCurrentTaskHandle();
    auto &vrc               = pyro::rc_drv_t::read();

    pyro::btn_broker::subscribe(&vrc.keys.r, pyro::btn_event_t::PRESS_DOWN,
                                self, EVENT_BIT_RESET);
    pyro::btn_broker::subscribe(&vrc.keys.b, pyro::btn_event_t::PRESS_DOWN,
                                self, EVENT_BIT_NORMAL);
    pyro::btn_broker::subscribe(&vrc.keys.x, pyro::btn_event_t::PRESS_DOWN,
                                self, EVENT_BIT_SELF_CTRL);
    pyro::btn_broker::subscribe(&vrc.keys.shift, pyro::btn_event_t::PRESS_DOWN,
                                self, EVENT_BIT_TURN_GRIPPER);

    for (;;)
    {
        uint32_t notify_val = 0;
        xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

        // 检测按键 R 触发
        if (notify_val & EVENT_BIT_RESET)
        {
            if (s_fsm_task_handle != nullptr &&
                eTaskGetState(s_fsm_task_handle) == eSuspended)
                vTaskResume(s_fsm_task_handle);

            main_fsm.change_state(&main_fsm.idle_state);
        }
        // 检测按键 B 触发
        if (notify_val & EVENT_BIT_NORMAL)
        {
            if (s_fsm_task_handle != nullptr &&
                eTaskGetState(s_fsm_task_handle) == eSuspended)
                vTaskResume(s_fsm_task_handle);

            main_fsm.change_state(&main_fsm.normal_state);
        }
        // 检测按键 X 触发
        if (notify_val & EVENT_BIT_SELF_CTRL)
        {
            if (s_fsm_task_handle != nullptr &&
                eTaskGetState(s_fsm_task_handle) == eSuspended)
                vTaskResume(s_fsm_task_handle);

            main_fsm.change_state(&main_fsm.self_ctrl_state);
        }
        // 检测按键 Shift 触发
        if (notify_val & EVENT_BIT_TURN_GRIPPER)
        {
            arm_module_t::instance()->_turn_gripper_state();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ==================== 状态机执行任务 ====================
fsm_task_t::fsm_task_t()
    : task_base_t("fsm_task", 512, 512, priority_t::HIGH)
{
}

status_t fsm_task_t::init()
{
    return PYRO_OK;
}

void fsm_task_t::run_loop()
{
    // 记录自身句柄，供遥控器任务做挂起/恢复判断
    s_fsm_task_handle = xTaskGetCurrentTaskHandle();

    for (;;)
    {
        if (fsm_ctx_ptr != nullptr)
        {
            main_fsm.execute(fsm_ctx_ptr);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

arm_rc_task_t &arm_rc_task()
{
    static arm_rc_task_t instance;
    return instance;
}

fsm_task_t &fsm_task()
{
    static fsm_task_t instance;
    return instance;
}

// ==================== 机械臂初始化任务 ====================
arm_init_task_t::arm_init_task_t()
    : task_base_t("arm_init_task", 512, 512, priority_t::REALTIME)
{
}

status_t arm_init_task_t::init()
{
    return PYRO_OK;
}

void arm_init_task_t::run_loop()
{
    arm_cmd_ptr = new arm_cmd_t();
    deps_init();
    arm_module_t::instance()->configure(*arm_deps);

    arm_module_t::instance()->start();
    arm_rc_task().start();
}

// ==================== 状态机初始化任务 ====================
fsm_init_task_t::fsm_init_task_t()
    : task_base_t("fsm_init_task", 512, 512, priority_t::REALTIME)
{
}

status_t fsm_init_task_t::init()
{
    return PYRO_OK;
}

void fsm_init_task_t::run_loop()
{
    fsm_ctx_ptr = new fsm_ctx_t();

    // 设置FSM初始状态：idle_state
    main_fsm.change_state(&main_fsm.idle_state);

    fsm_task().start();
}

arm_init_task_t &arm_init_task()
{
    static arm_init_task_t instance;
    return instance;
}

fsm_init_task_t &fsm_init_task()
{
    static fsm_init_task_t instance;
    return instance;
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
    arm_deps->end_motor = new dm_motor_drv_t(0xe, 0xd, bsp_can::can2);


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

    arm_deps->end_motor->set_position_range(-PI,PI);
    arm_deps->end_motor->set_rotate_range(-200,200);
    arm_deps->end_motor->set_torque_range(-7,7);


    arm_deps->pos_pid[0] = new pid_t(10,0.0,0.0,0.0,52);
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

    arm_deps->end_pos_pid = new pid_t(9,0.0,0.0,0,200);
    arm_deps->end_rot_pid = new pid_t(0.8,0.0,0.0,0,7);


    arm_deps->axis[0] = new arm_deps_t::axis_deps_t(arm_deps->motor[0], arm_deps->pos_pid[0],arm_deps->rot_pid[0]);
    arm_deps->axis[1] = new arm_deps_t::axis_deps_t(arm_deps->motor[1], arm_deps->pos_pid[1],arm_deps->rot_pid[1]);
    arm_deps->axis[2] = new arm_deps_t::axis_deps_t(arm_deps->motor[2], arm_deps->pos_pid[2],arm_deps->rot_pid[2]);
    arm_deps->axis[3] = new arm_deps_t::axis_deps_t(arm_deps->motor[3], arm_deps->pos_pid[3],arm_deps->rot_pid[3]);
    arm_deps->axis[4] = new arm_deps_t::axis_deps_t(arm_deps->motor[4], arm_deps->pos_pid[4],arm_deps->rot_pid[4]);
    arm_deps->axis[5] = new arm_deps_t::axis_deps_t(arm_deps->motor[5], arm_deps->pos_pid[5],arm_deps->rot_pid[5]);
    arm_deps->end_gripper = new arm_deps_t::axis_deps_t(arm_deps->end_motor, arm_deps->end_pos_pid, arm_deps->end_rot_pid);

    arm_deps->axis[0]->enable_constraint();
    arm_deps->axis[1]->enable_constraint();
    arm_deps->axis[2]->enable_constraint();
    arm_deps->axis[3]->disable_constraint();    // R4 无限位关节
    arm_deps->axis[4]->enable_constraint();
    arm_deps->axis[5]->enable_constraint();
    arm_deps->end_gripper->enable_constraint();

    arm_deps->axis[0]->set_limit(-2.1599f, PI);
    arm_deps->axis[1]->set_limit(-0.3f, 1.0f);
    arm_deps->axis[2]->set_limit(-0.3f, 1.35f);
    arm_deps->axis[3]->set_limit(-0.0f, 0.0f);
    arm_deps->axis[4]->set_limit(-0.5f*PI, 0.5f*PI);
    arm_deps->axis[5]->set_limit(-0.5f*PI, 0.5f*PI);
    arm_deps->end_gripper->set_limit(-2.0f, 0.0f);

    arm_deps->axis[0]->set_feedback_pos_offset(-1.9206f);
    arm_deps->axis[1]->set_feedback_pos_offset(-2.3948f);
    arm_deps->axis[2]->set_feedback_pos_offset(-0.328f);
    arm_deps->axis[3]->set_feedback_pos_offset(2.34739f);
    arm_deps->axis[4]->set_feedback_pos_offset(2.46375f);
    arm_deps->axis[5]->set_feedback_pos_offset(2.74246f);
    arm_deps->end_gripper->set_feedback_pos_offset(1.8587f);
}

}
