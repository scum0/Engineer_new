/**
 * @file pyro_task.h
 * @brief Base class for FreeRTOS task management.
 * FreeRTOS 任务管理基类。
 */

#ifndef __PYRO_TASK_H__
#define __PYRO_TASK_H__

#include "FreeRTOS.h"
#include "pyro_core_def.h"
#include "task.h"
#include "semphr.h"
#include <cstdint>

namespace pyro
{

/**
 * @brief Abstract base class for managing FreeRTOS tasks.
 * 管理 FreeRTOS 任务的抽象基类。
 */
class task_base_t
{
  public:
    /**
     * @brief Task priority levels.
     * 任务优先级等级。
     */
    enum class priority_t : uint8_t
    {
        IDLE,         /**< Idle (0). 空闲级。 */
        LOW,          /**< Low (1). 低优先级。 */
        BELOW_NORMAL, /**< Below Normal (2). 次正常级。 */
        NORMAL,       /**< Normal (3). 正常级。 */
        ABOVE_NORMAL, /**< Above Normal (4). 高正常级。 */
        HIGH,         /**< High (5). 高优先级。 */
        REALTIME,     /**< Real-time (6). 实时级。 */
    };

    /**
     * @brief Constructor for task_base_t.
     * task_base_t 构造函数。
     *
     * @param name Task name. 任务名称。
     * @param init_stack Init stack size. 初始化栈大小。
     * @param loop_stack Loop stack size. 循环栈大小。
     * @param priority Priority level. 优先级。
     */
    task_base_t(const char *name, uint16_t init_stack, uint16_t loop_stack,
                priority_t priority);

    /** @brief Virtual destructor. 虚析构函数。 */
    virtual ~task_base_t();

    /** @brief Starts the task. 启动任务。 */
    status_t start();

    /** @brief Stops the task. 停止任务。 */
    void stop();

    //==================== 新增：任务监控与控制接口 ====================
    /**
     * @brief Suspend this task(封装vTaskSuspend，增加日志打点)
     * @note 禁止直接调用FreeRTOS原生vTaskSuspend/vTaskResume，否则丢失监控事件
     */
    void suspend();

    /**
     * @brief Resume this task(封装vTaskResume，增加日志打点)
     */
    void resume();

    /**
     * @brief 监控表子项，保存任务运行时监控信息
     */
    struct task_monitor_item_t
    {
        TaskHandle_t handle{nullptr};      ///< FreeRTOS任务句柄
        const char* name{nullptr};         ///< 任务名称
        task_base_t* obj{nullptr};         ///< 指向pyro任务基类对象指针
    };

    /**
     * @brief 获取任务监控列表快照，供外部巡检任务调用
     * @param out_buf 输出缓冲区，接收监控项副本
     * @param buf_size 输出缓冲区最大容量
     * @param out_cnt 实际复制得到的有效条目数量
     * @note 内部持有互斥锁做拷贝，锁持有时间很短；不在锁内部做耗时操作
     */
    static void get_monitor_list(task_monitor_item_t* out_buf, size_t buf_size, size_t* out_cnt);

    /**
     * @brief 初始化监控模块，创建监控互斥锁
     * @note 在创建任何pyro任务之前调用一次
     */
    static void monitor_init();

  protected:
    /** @brief Initialization callback. 初始化回调。 */
    virtual status_t init()     = 0;

    /** @brief Main loop callback. 主循环回调。 */
    virtual void run_loop() = 0;

    /** @brief Task handle. 任务句柄。 */
    TaskHandle_t _loop_task_handle;

  private:
    const char *_task_name;
    uint16_t _loop_stack_depth;
    priority_t _priority;

    /** @brief Entry point for init. 初始化入口。 */
    static status_t init_entry_point(void *arg);

    /** @brief Entry point for loop. 循环入口。 */
    static void loop_entry_point(void *arg);

    /** @brief Priority converter. 优先级转换器。 */
    static UBaseType_t convert_priority(priority_t p);

    //==================== 静态全局监控相关变量 ====================
    static SemaphoreHandle_t _monitor_mutex;          ///< 监控表访问互斥锁，保护多任务并发读写监控表
    static constexpr size_t MONITOR_MAX_TASK = 16U;    ///< 支持最大监控任务数量，根据项目实际任务数调整
    static task_monitor_item_t _monitor_table[MONITOR_MAX_TASK]; ///< 任务监控总表
    static size_t _monitor_count;                      ///< 当前监控表有效任务数目

    /**
     * @brief 将当前任务实例注册进入全局监控表
     * @note xTaskCreate创建任务成功后调用；内部会拿互斥锁
     */
    void register_monitor();

    /**
     * @brief 将当前任务实例从全局监控表注销
     * @note 任务销毁/停止时调用；内部会拿互斥锁
     */
    void unregister_monitor();
};

// 打印任务运行信息函数声明
extern"C" void sys_monitor_start(void);

} // namespace pyro
#endif
