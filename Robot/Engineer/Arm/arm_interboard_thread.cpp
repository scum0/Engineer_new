/**
 * @file arm_interboard_thread.cpp
 * @brief 机械臂板与底盘板的板间通信任务
 *
 * 接收遥控器数据（使用 PYRo/Component/RC 下的 RC 驱动），组包后经板间串口
 * (INTERBOARD_UART) 发送给底盘；同时接收底盘回传的状态帧。
 */

#include "arm_interboard_thread.h"
#include "../Application/arm_rc_receive.h"

#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"
#include "pyro_crc.h"
#include "pyro_task.h"

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"

#include <cstring>

using namespace pyro;

__attribute__((section(".dma_heap"))) uint8_t
    arm_interboard_task_t::_tx_dma_buf[sizeof(upper_to_lower_frame_t)];

// ==================== 接收回调（ISR） ====================
bool arm_interboard_task_t::rx_callback(
    uint8_t *data, uint16_t len, BaseType_t &xHigherPriorityTaskWoken)
{
    if (len == sizeof(lower_to_upper_frame_t))
    {
        if (_rx_msg_buf != nullptr &&
            xMessageBufferSendFromISR(_rx_msg_buf, data, len, &xHigherPriorityTaskWoken) == len)
        {
            if (_rx_task != nullptr)
            {
                vTaskNotifyGiveFromISR(_rx_task, &xHigherPriorityTaskWoken);
            }
            return true;
        }
    }
    return false;
}

// ==================== 发送函数 ====================
void arm_interboard_task_t::send_frame()
{
    auto *frame = reinterpret_cast<upper_to_lower_frame_t *>(_tx_buf);

    // 清零后填充，避免未使用字段残留脏数据
    memset(frame, 0, sizeof(upper_to_lower_frame_t));

    const arm_rc_t command = get_arm_rc();
    frame->enable       = command.enable;
    frame->vx           = command.vx;
    frame->vy           = command.vy;
    frame->wz           = command.wz;
    frame->lift_auto    = command.lift_auto;
    frame->frame_header = FRAME_HEADER;
    append_crc16_check_sum(reinterpret_cast<uint8_t *>(frame),
                           sizeof(upper_to_lower_frame_t));

#ifdef INTERBOARD_UART
    if (_uart)
    {
        _uart->write(reinterpret_cast<uint8_t *>(frame),sizeof(upper_to_lower_frame_t));
    }
#endif
}

// ==================== 接收处理 ====================
void arm_interboard_task_t::process_received_frames()
{
    lower_to_upper_frame_t frame{};
    while (xMessageBufferReceive(_rx_msg_buf, &frame, sizeof(frame), 0) ==
           sizeof(frame))
    {
        if (frame.frame_header != FRAME_HEADER)
            continue;
        if (!verify_crc16_check_sum(reinterpret_cast<uint8_t *>(&frame),
                                    sizeof(frame)))
            continue;

        scoped_mutex_t lock(_msg_mutex);
        _msg.chassis_online = frame.chassis_online != 0;
        _msg.magazine_pos   = frame.magazine_pos;
        _msg.magazine_ready = frame.magazine_ready != 0;
        _msg.magazine_mask  = frame.magazine_mask & 0x0F;
        _last_rx_tick       = xTaskGetTickCount();
    }
}

void arm_interboard_task_t::check_rx_timeout()
{
    if (_msg.chassis_online &&
        xTaskGetTickCount() - _last_rx_tick > RX_TIMEOUT)
    {
        _msg = {};
    }
}

chassis_msg_t arm_interboard_task_t::get_msg()
{
    scoped_mutex_t lock(_msg_mutex);
    check_rx_timeout();
    return _msg;
}

// ==================== 板间通信任务 ====================
namespace pyro
{

chassis_msg_t get_interboard_msg()
{
    return arm_interboard_task().get_msg();
}

arm_interboard_task_t::arm_interboard_task_t()
    : task_base_t("arm_interboard", 512, 512, priority_t::BELOW_NORMAL)
{
#ifdef INTERBOARD_UART
    _uart = &INTERBOARD_UART;
#endif
}

status_t arm_interboard_task_t::init()
{
#ifdef INTERBOARD_UART
    _tx_buf = _tx_dma_buf;

    constexpr size_t RX_MSG_BUF_SIZE = sizeof(lower_to_upper_frame_t) * 4U + 4U;
    _rx_msg_buf = xMessageBufferCreate(RX_MSG_BUF_SIZE);
    if (_rx_msg_buf == nullptr)
    {
        return PYRO_NO_MEMORY;
    }

    _tx_done_sem = xSemaphoreCreateBinary();
    if (_tx_done_sem == nullptr)
    {
        return PYRO_NO_MEMORY;
    }
    _uart->set_tx_cplt_callback(
        [this](BaseType_t &xHigherPriorityTaskWoken)
        {
            xSemaphoreGiveFromISR(_tx_done_sem, &xHigherPriorityTaskWoken);
        });

    // 注册接收回调
    _uart->add_rx_event_callback(
        [this](uint8_t *data, uint16_t len,
               BaseType_t &xHigherPriorityTaskWoken) -> bool
        {
            return rx_callback(data, len, xHigherPriorityTaskWoken);
        },
        CALLBACK_OWNER);
    // 启动 DMA 接收
    _uart->enable_rx_dma();
#endif
    return PYRO_OK;
}

void arm_interboard_task_t::run_loop()
{
    _rx_task = xTaskGetCurrentTaskHandle();

    for (;;)
    {
        // 上一次 DMA 发送完成后才覆盖发送缓冲区。
        if (xSemaphoreTake(_tx_done_sem, 0) == pdTRUE ||
            _uart->state.tx_busy == 0)
        {
            send_frame();
        }

        if (ulTaskNotifyTake(pdTRUE, 0) > 0)
        {
            process_received_frames();
        }
        {
            scoped_mutex_t lock(_msg_mutex);
            check_rx_timeout();
        }
        vTaskDelay(TASK_PERIOD);
    }
}

arm_interboard_task_t &arm_interboard_task()
{
    static arm_interboard_task_t instance;
    return instance;
}

} // namespace pyro
