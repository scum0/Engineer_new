/**
 * @file arm_interboard_thread.cpp
 * @brief 机械臂板与底盘板的板间通信任务
 *
 * 接收遥控器数据（使用 PYRo/Component/RC 下的 RC 驱动），组包后经板间串口
 * (INTERBOARD_UART) 发送给底盘；同时接收底盘回传的状态帧。
 */

#include "arm_interboard_thread.h"

#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"
#include "pyro_crc.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_rc_core.h"
#include "pyro_rw_lock.h"
#include "pyro_task.h"
#include "pyro_vt03_rc_drv.h"

#include "FreeRTOS.h"
#include "task.h"

#include <cstring>

using namespace pyro;

namespace
{

// ==================== 常量 ====================
static constexpr uint16_t FRAME_HEADER    = 0xFFA5;
static constexpr uint32_t CALLBACK_OWNER  = 0xAA;
static constexpr TickType_t TASK_PERIOD   = pdMS_TO_TICKS(20);

// 摇杆放大系数
static constexpr float AXIS_SCALE = 3.0f;

// ==================== 帧结构 ====================
#pragma pack(push, 1)
struct upper_to_lower_frame_t
{
    uint16_t frame_header;
    uint16_t enable;
    float vx;
    float vy;
    float wz;
    uint16_t magazine_pos;
    uint16_t lift_control_mod;
    uint16_t lift_mannual;
    uint16_t lift_auto;
    uint16_t lift_calib_trigger;
    uint16_t crc16;
};

struct lower_to_upper_frame_t
{
    uint16_t frame_header;
    uint16_t chassis_online;
    uint16_t magazine_pos;
    uint16_t magazine_ready;
    uint8_t magazine_mask;
    uint16_t crc16;
};
#pragma pack(pop)

// ==================== 静态变量 ====================
#ifdef INTERBOARD_UART
static uart_drv_t *s_uart = &INTERBOARD_UART;
#endif

// 使用 DMA 堆分配缓冲区，确保 DMA 可访问
__attribute__((section(".dma_heap"))) static uint8_t
    s_rx_buf[sizeof(lower_to_upper_frame_t)];
__attribute__((section(".dma_heap"))) static uint8_t
    s_tx_buf[sizeof(upper_to_lower_frame_t)];

static volatile bool s_rx_new_frame = false;

// ==================== 接收回调（ISR） ====================
bool rx_callback(uint8_t *data, uint16_t len,
                 BaseType_t &xHigherPriorityTaskWoken)
{
    (void)xHigherPriorityTaskWoken;

    if (len == sizeof(lower_to_upper_frame_t))
    {
        UBaseType_t uxSaved = taskENTER_CRITICAL_FROM_ISR();
        memcpy(static_cast<void *>(s_rx_buf), data, len);
        s_rx_new_frame = true;
        taskEXIT_CRITICAL_FROM_ISR(uxSaved);
        return true;
    }
    return false;
}

// ==================== 遥控器数据采集 ====================
/**
 * @brief 从 RC 驱动读取遥控器数据并映射为底盘控制量
 *
 * 若遥控器离线，则输出默认值（速度 0、使能 0）。
 */
void read_rc_command(upper_to_lower_frame_t *frame)
{
    // 默认值
    float vx        = 0.0f;
    float vy        = 0.0f;
    float wz        = 0.0f;
    uint16_t enable = 0;
    uint16_t lift   = 0;

    bool rc_online = false;

#ifdef VT03_UART
    if (vt03_drv_t::instance().check_online())
    {
        read_scope_lock lock(vt03_drv_t::instance().get_lock());
        const auto &vrc = rc_drv_t::read();

        vx = vrc.axes.ly * AXIS_SCALE;
        vy = vrc.axes.lx * AXIS_SCALE;
        wz = vrc.axes.rx * AXIS_SCALE;

        enable    = (vrc.switches.gear.current_pos != sw_pos_t::DOWN) ? 1 : 0;
        rc_online = true;
    }
#endif

#ifdef DR16_UART
    if (!rc_online && dr16_drv_t::instance().check_online())
    {
        read_scope_lock lock(dr16_drv_t::instance().get_lock());
        const auto &vrc = rc_drv_t::read();

        vx = vrc.axes.ly * AXIS_SCALE;
        vy = vrc.axes.lx * AXIS_SCALE;
        wz = vrc.axes.rx * AXIS_SCALE;

        // 左拨杆向下 → 失能，其余档位使能
        enable = (vrc.switches.left.current_pos != sw_pos_t::DOWN) ? 1 : 0;

        // 左拨杆向上 → 收起(2)，否则放下(1)
        if (vrc.switches.left.current_pos == sw_pos_t::UP)
            lift = 2;
        else
            lift = 1;

        rc_online = true;
    }
#endif

    frame->enable    = enable;
    frame->vx        = vx;
    frame->vy        = vy;
    frame->wz        = wz;
    frame->lift_auto = lift;
}

// ==================== 发送函数 ====================
void send_frame()
{
    auto *frame = reinterpret_cast<upper_to_lower_frame_t *>(s_tx_buf);

    // 清零后填充，避免未使用字段残留脏数据
    memset(frame, 0, sizeof(upper_to_lower_frame_t));

    read_rc_command(frame);

    frame->frame_header = FRAME_HEADER;
    append_crc16_check_sum(reinterpret_cast<uint8_t *>(frame),
                           sizeof(upper_to_lower_frame_t));

#ifdef INTERBOARD_UART
    if (s_uart)
    {
        // 阻塞轮询发送，确保发送完成后再修改缓冲区
        s_uart->write(reinterpret_cast<uint8_t *>(frame),
                      sizeof(upper_to_lower_frame_t), 100);
    }
#endif
}

// ==================== 接收处理 ====================
void process_received_frame()
{
    if (!s_rx_new_frame)
        return;

    taskENTER_CRITICAL();
    s_rx_new_frame = false;
    taskEXIT_CRITICAL();

    auto *frame = reinterpret_cast<lower_to_upper_frame_t *>(s_rx_buf);

    if (frame->frame_header != FRAME_HEADER)
        return;
    if (!verify_crc16_check_sum(s_rx_buf, sizeof(lower_to_upper_frame_t)))
        return;
}

} // namespace

// ==================== 板间通信任务 ====================
namespace pyro
{

arm_interboard_task_t::arm_interboard_task_t()
    : task_base_t("arm_interboard", 512, 512, priority_t::BELOW_NORMAL)
{
}

status_t arm_interboard_task_t::init()
{
#ifdef INTERBOARD_UART
    // 注册接收回调
    s_uart->add_rx_event_callback(rx_callback, CALLBACK_OWNER);
    // 启动 DMA 接收
    s_uart->enable_rx_dma();
#endif
    return PYRO_OK;
}

void arm_interboard_task_t::run_loop()
{
    for (;;)
    {
        send_frame();
        process_received_frame();
        vTaskDelay(TASK_PERIOD);
    }
}

arm_interboard_task_t &arm_interboard_task()
{
    static arm_interboard_task_t instance;
    return instance;
}

} // namespace pyro
