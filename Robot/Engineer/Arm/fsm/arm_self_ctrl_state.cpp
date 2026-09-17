#include "pyro_crc.h"
#include "pyro_uart_drv.h"
#include "cmsis_os.h"
#include <cstring>
#include "arm_fsm.h"
#include "protocol.h"
#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"

using namespace pyro;

typedef struct  __attribute__((packed))
{
    frame_header_t header;
    uint16_t cmd_id;
    float data[7];
    uint8_t uesd_data[2];   // 数据大小固定为30
    uint16_t CRC16;
}
referee_datalink_frame_t;

#ifdef SELF_CTRL_UART
static pyro::uart_drv_t* self_control_uart = &SELF_CTRL_UART;
#endif

referee_datalink_frame_t self_control_frame;
QueueSetHandle_t self_control_queue;
static uint8_t self_control_buf[128];

static constexpr float alpha = 0.02;
static constexpr uint8_t AXIS_NUM = 6;

bool self_control_callback(uint8_t *buf, uint16_t len,BaseType_t xHigherPriorityTaskWoken)
{
    if( len == sizeof(referee_datalink_frame_t) )
    {
        xQueueSendFromISR(self_control_queue, buf,  NULL);
        return true;
    }
    return false;
}

// 该模块的目的为从图传链路获取自控数据
class self_control_command_t
{
public:
    self_control_command_t();
    ~self_control_command_t();
    void update();
    float* get_self_control_command();
private:
    std::array<float,AXIS_NUM> _origin_command{};
    std::array<float,AXIS_NUM> _filter_command{};

};

self_control_command_t::self_control_command_t(){};
self_control_command_t::~self_control_command_t(){};

void self_control_command_t::update()
{
    memcpy(&self_control_frame, self_control_buf, sizeof(referee_datalink_frame_t));
    memcpy(_origin_command.data(),&self_control_frame.data[0],sizeof(float)*6);

    for (int i = 0; i < 6; i++)
    {
        _filter_command[i] = _filter_command[i]*(1-alpha) + _origin_command[i]*alpha;
    }
}

self_control_command_t self_control_command;

float* self_control_command_t::get_self_control_command()
{
    return _filter_command.data();
}

void arm_top_fsm_t::self_ctrl_state_t::enter(owner *owner)
{
    owner->is_self_ctrl = true;
}

void arm_top_fsm_t::self_ctrl_state_t::execute(owner *owner)
{
    owner->set_command(self_control_command.get_self_control_command());
}

void arm_top_fsm_t::self_ctrl_state_t::exit(owner *owner)
{
    owner->is_self_ctrl = false;
}

extern "C" void self_control_thread(void* argument)
{
    self_control_queue = xQueueCreate(10, sizeof(referee_datalink_frame_t));
    // 仅当CMake定义SELF_CTRL_UART时执行串口初始化
#ifdef SELF_CTRL_UART
    // 注册回调
    self_control_uart->add_rx_event_callback(self_control_callback, 0xA501);
#endif
    for (;;)
    {
        if(xQueueReceive(self_control_queue, self_control_buf, 0)==pdPASS)
        {
            auto* frame = reinterpret_cast<referee_datalink_frame_t*>(self_control_buf);

            if(verify_crc8_check_sum(self_control_buf,sizeof(frame_header_t))&&
            verify_crc16_check_sum(self_control_buf, sizeof(referee_datalink_frame_t))&&
            frame->header.sof == 0xA5&&
            frame->cmd_id == static_cast<uint16_t>(pyro::cmd_id::CUSTOM_CONTROLLER)&&
            frame->header.data_length == 30)
            {
                self_control_command.update();
            }
        }
        vTaskDelay(1);
    }
}