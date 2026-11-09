/**
 * @file sys_task_monitor.c
 * @brief 系统巡检任务，周期检测各pyro任务栈高水印，仅在剩余不足时告警
 * @note FreeRTOS配置需要打开：#define INCLUDE_uxTaskGetStackHighWaterMark 1
 */
#ifdef UES_LOG
#include "pyro_task.h"
#include "pyro_log.h"
#include "FreeRTOS.h"
#include "task.h"

using namespace pyro;

/* 栈告警阈值，剩余小于该字节输出警告，根据项目调整 */
#define STACK_WARN_THRESHOLD_WORD     32U

/**
 * @brief 系统监控主循环，1秒周期输出任务栈高水印
 * @param arg 未使用参数
 */
static void sys_monitor_task(void* arg)
{
    constexpr size_t TMP_BUF_CNT = 16U;
    pyro::task_base_t::task_monitor_item_t tmp_list[TMP_BUF_CNT];
    size_t item_cnt;

    for(;;)
    {
        //获取监控表快照（短暂持有互斥锁）
        pyro::task_base_t::get_monitor_list(tmp_list, TMP_BUF_CNT, &item_cnt);

        for(size_t i=0;i<item_cnt;i++)
        {
            auto &item = tmp_list[i];
            if(item.handle == nullptr)
            {
                continue;
            }
            /* uxTaskGetStackHighWaterMark 返回值单位为Word，Cortex‑M一个Word=4字节 */
            UBaseType_t remain_words = uxTaskGetStackHighWaterMark(item.handle);

            /* 栈剩余不足阈值，输出警告日志；正常情况也输出最小剩余栈 */
            if(remain_words < STACK_WARN_THRESHOLD_WORD)
            {
                PYRO_LOGW("SYS_MON", "[STACK WARN] %s stack remain only %lu word !", item.name, (unsigned long)remain_words);
            }
            else
            {
                PYRO_LOGI("SYS_MON", "%s stack remain %lu word", item.name, (unsigned long)remain_words);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000U));
    }
}

/**
 * @brief 启动系统监控任务
 * @note 调用时机：pyro::task_base_t::monitor_init()之后，创建业务任务之前
 */
extern"C" BaseType_t sys_monitor_start(void)
{
    pyro::task_base_t::monitor_init();
    return xTaskCreate(sys_monitor_task, "sys_mon", 1024, nullptr, 2, nullptr);
}
#endif
