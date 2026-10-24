#ifndef __PYRO_ARM_INTERBOARD_THREAD_H__
#define __PYRO_ARM_INTERBOARD_THREAD_H__

#include "pyro_task.h"
#include "pyro_mutex.h"
#include "pyro_uart_drv.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "semphr.h"
#include <cstdint>

namespace pyro
{

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

struct chassis_msg_t
{
    bool chassis_online{false};
    uint16_t magazine_pos{0};
    bool magazine_ready{false};
    uint8_t magazine_mask{0};
};

/**
 * @brief 获取最近一次有效底盘状态，通信超时后自动返回离线状态
 */
chassis_msg_t get_interboard_msg();

/**
 * @brief 板间通信任务（机械臂板 <-> 底盘板）
 */
class arm_interboard_task_t final : public task_base_t
{
  public:
    arm_interboard_task_t();
    chassis_msg_t get_msg();

  protected:
    status_t init() override;
    void run_loop() override;

  private:
    bool rx_callback(uint8_t *data, uint16_t len,
                     BaseType_t &xHigherPriorityTaskWoken);
    void send_frame();
    void process_received_frames();
    void check_rx_timeout();

    static constexpr uint16_t FRAME_HEADER   = 0xFFA5;
    static constexpr uint32_t CALLBACK_OWNER = 0xAA;
    static constexpr TickType_t TASK_PERIOD  = pdMS_TO_TICKS(20);
    static constexpr TickType_t RX_TIMEOUT   = pdMS_TO_TICKS(500);

    __attribute__((section(".dma_heap"))) static uint8_t
        _tx_dma_buf[sizeof(upper_to_lower_frame_t)];

    uart_drv_t *_uart{nullptr};
    uint8_t *_tx_buf{nullptr};
    TaskHandle_t _rx_task{nullptr};
    MessageBufferHandle_t _rx_msg_buf{nullptr};
    SemaphoreHandle_t _tx_done_sem{nullptr};
    TickType_t _last_rx_tick{0};
    chassis_msg_t _msg{};
    mutex_t _msg_mutex;
};

/**
 * @brief 获取板间通信任务单例
 */
arm_interboard_task_t &arm_interboard_task();

} // namespace pyro

#endif // __PYRO_ARM_INTERBOARD_THREAD_H__
