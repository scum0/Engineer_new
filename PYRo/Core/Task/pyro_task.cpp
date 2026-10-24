/**
 * @file pyro_task.cpp
 * @brief Implementation of the task management base.
 * 任务管理基类的实现。
 */

#include "pyro_task.h"
#include "pyro_log.h"

namespace pyro
{

//==================== 静态变量定义 ====================
SemaphoreHandle_t task_base_t::_monitor_mutex = nullptr;
constexpr size_t task_base_t::MONITOR_MAX_TASK;
task_base_t::task_monitor_item_t task_base_t::_monitor_table[task_base_t::MONITOR_MAX_TASK]{};
size_t task_base_t::_monitor_count = 0U;

/**
 * @brief 初始化监控模块，创建监控互斥锁
 */
void task_base_t::monitor_init()
{
    if(_monitor_mutex == nullptr)
    {
        _monitor_mutex = xSemaphoreCreateMutex();
    }
}

/**
 * @brief Constructor for task_base_t.
 * task_base_t 构造函数。
 */
task_base_t::task_base_t(const char *name, const uint16_t init_stack,
                         const uint16_t loop_stack, const priority_t priority)
    : _loop_task_handle(nullptr), _task_name(name),
      _loop_stack_depth(loop_stack), _priority(priority)
{
    (void)init_stack;
}

/**
 * @brief Destructor ensuring task stop.
 * 确保任务停止的析构函数。
 */
task_base_t::~task_base_t()
{
    stop();
}

/**
 * @brief Spawns the initial setup task.
 * 生成初始设置任务。
 */
status_t task_base_t::start()
{
    if (_loop_task_handle != nullptr)
    {
        PYRO_LOGW("TASK", "task[%s] already start", _task_name);
        return PYRO_ERROR;
    }

    const status_t ret = init_entry_point(this);

    return ret;
}

/**
 * @brief Deletes the loop task if it exists.
 * 如果循环任务存在则将其删除。
 */
void task_base_t::stop()
{
    if (_loop_task_handle != nullptr)
    {
        PYRO_LOGI("TASK", "stop() called, delete task: %s", _task_name);
        vTaskDelete(_loop_task_handle);
        unregister_monitor();
        _loop_task_handle = nullptr;
    }
}

/**
 * @brief Suspend this task，封装FreeRTOS vTaskSuspend，增加日志事件输出
 */
void task_base_t::suspend()
{
    if(_loop_task_handle)
    {
        PYRO_LOGI("TASK", "SUSPEND task: %s", _task_name);
        vTaskSuspend(_loop_task_handle);
    }
}

/**
 * @brief Resume this task，封装vTaskResume，增加日志事件输出
 */
void task_base_t::resume()
{
    if(_loop_task_handle)
    {
        vTaskResume(_loop_task_handle);
        PYRO_LOGI("TASK", "RESUME task: %s", _task_name);
    }
}

/**
 * @brief Internal entry for the initialization phase.
 * 初始化阶段的内部入口。
 * @note 1.先调用子类虚init()；
 * @note 2.如果init成功，调用xTaskCreate创建循环任务；
 * @note 3.捕获xTaskCreate创建失败（堆内存不足），手动清空句柄，避免stop()传入非法TCB造成HardFault；
 * @note 4.创建成功：注册进入监控表，输出创建日志；创建失败输出错误日志
 */
status_t task_base_t::init_entry_point(void *arg)
{
    auto *self = static_cast<task_base_t *>(arg);
    if (!self)
        return PYRO_ERROR;

    const status_t init_ret = self->init();
    if (init_ret != PYRO_OK)
    {
        PYRO_LOGE("TASK", "task[%s] init() fail, ret=%d", self->_task_name, (int)init_ret);
        return init_ret;
    }

    if (self->_loop_stack_depth > 0)
    {
        const BaseType_t ret = xTaskCreate(
            loop_entry_point, self->_task_name, self->_loop_stack_depth,
            self, convert_priority(self->_priority),
            &self->_loop_task_handle);

        if (ret == pdPASS)
        {
            //任务创建成功，注册监控，打印创建事件日志
            self->register_monitor();
            xSemaphoreTake(_monitor_mutex, portMAX_DELAY);
            PYRO_LOGI("TASK", "CREATE task[%s] stack=%u words, prio=%u",
                      self->_task_name, self->_loop_stack_depth,
                      (unsigned)convert_priority(self->_priority));
            xSemaphoreGive(_monitor_mutex);
            return PYRO_OK;
        }
        else
        {
            /**
             * 【重要】xTaskCreate返回pdFALSE代表堆内存不足，任务未创建；
             * FreeRTOS不会自动把输出句柄置NULL，会保留旧的垃圾值；
             * 如果不清空，后续stop()会拿非法句柄调用vTaskDelete，直接HardFault。
             */
            self->_loop_task_handle = nullptr;
            PYRO_LOGE("TASK", "CREATE FAIL task[%s], xTaskCreate return fail(heap insufficient)", self->_task_name);
        }
    }
    return PYRO_OK;
}

/**
 * @brief Internal entry for the continuous loop phase.
 * 持续循环阶段的内部入口。
 * @note 子类run_loop正常return退出时，代表任务主动结束；打印退出事件，注销监控，自我删除。
 */
void task_base_t::loop_entry_point(void *arg)
{
    auto *self = static_cast<task_base_t *>(arg);
    if (self)
    {
        self->run_loop();
        //run_loop执行完毕主动退出，任务生命周期结束
        PYRO_LOGW("TASK", "task[%s] run_loop exit, will delete", self->_task_name);
        self->unregister_monitor();
        self->_loop_task_handle = nullptr;
    }
    vTaskDelete(nullptr);
}

/**
 * @brief Converts abstract priority to FreeRTOS scale.
 * 将抽象优先级转换为 FreeRTOS 刻度。
 *
 * @details
 * Formula: (p * (max - 1)) / 6.
 * 公式：(p * (max - 1)) / 6。
 */
UBaseType_t task_base_t::convert_priority(priority_t p)
{
    return static_cast<UBaseType_t>(p) * (configMAX_PRIORITIES - 1) / 6;
}

/**
 * @brief 将本任务实例注册进入全局监控表
 * @note 持有互斥锁保护；监控表满输出错误告警
 */
void task_base_t::register_monitor()
{
    if(_monitor_mutex == nullptr) return;
    xSemaphoreTake(_monitor_mutex, portMAX_DELAY);
    if(_monitor_count < MONITOR_MAX_TASK)
    {
        _monitor_table[_monitor_count].handle = _loop_task_handle;
        _monitor_table[_monitor_count].name   = _task_name;
        _monitor_table[_monitor_count].obj    = this;
        _monitor_count ++;
    }
    else
    {
        PYRO_LOGE("TASK", "monitor table full! increase MONITOR_MAX_TASK");
    }
    xSemaphoreGive(_monitor_mutex);
}

/**
 * @brief 将本任务实例从监控表注销
 * @note 持有互斥锁；采用"末尾元素覆盖当前位置"的方式删除，避免数组元素挪动
 */
void task_base_t::unregister_monitor()
{
    if(_monitor_mutex == nullptr) return;
    xSemaphoreTake(_monitor_mutex, portMAX_DELAY);
    for(size_t i=0;i<_monitor_count;i++)
    {
        if(_monitor_table[i].obj == this)
        {
            //用最后一个有效元素覆盖待删除项
            if(i != _monitor_count-1)
            {
                _monitor_table[i] = _monitor_table[_monitor_count-1];
            }
            _monitor_count --;
            break;
        }
    }
    xSemaphoreGive(_monitor_mutex);
}

/**
 * @brief 获取监控列表快照，拷贝副本给调用者
 * @note 只做内存拷贝，不在锁内调用uxTaskGetStackHighWaterMark等耗时API；
 *       外部拿到副本之后再遍历读取栈高水印。
 */
void task_base_t::get_monitor_list(task_monitor_item_t* out_buf, size_t buf_size, size_t* out_cnt)
{
    *out_cnt = 0;
    if(_monitor_mutex == nullptr || out_buf == nullptr) return;

    xSemaphoreTake(_monitor_mutex, portMAX_DELAY);
    size_t copy_cnt = (_monitor_count < buf_size) ? _monitor_count : buf_size;
    for(size_t i=0;i<copy_cnt;i++)
    {
        out_buf[i] = _monitor_table[i];
    }
    *out_cnt = copy_cnt;
    xSemaphoreGive(_monitor_mutex);
}

} // namespace pyro
