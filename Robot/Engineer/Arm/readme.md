## 1
template <typename Derived, typename ModuleParams>
其中 Derived 是继承的子类，ModuleParams 是模板内部所有业务相关类型

## 2
using CmdType    = typename ModuleParams::CmdType;      //目标指令
using ModuleDeps = typename ModuleParams::ModuleDeps;   //依赖
using ModuleCtx  = typename ModuleParams::ModuleCtx;    //上下文
Deps（依赖）：硬件驱动指针、全局底层硬件配置（电机句柄、PID 指针、硬件极限范围）；
特征：上电初始化后几乎不会修改；
使用场景：在业务 _init() 里通过 _module_deps 访问电机、PID；
Ctx（上下文）：实时运行状态，每 1ms 刷新；
模板拆分 Deps / Ctx，使得资源与运行状态解耦

看一下模板对应的实现
# CmdType
CmdType _current_cmd;                   //定义指令变量
CmdType _cmd_buffer[CMD_BUF_SIZE];

bool set_command(const CmdType &cmd);   //下发命令接口

_cmd_buffer[_head] = cmd;               //环形缓冲区、拿锁拷贝
_current_cmd = _cmd_buffer[_tail];
由于模板底层会用到 cmd_base_t 的成员（mode、timestamp），所以继承原有类 cmd_base_t

struct arm_cmd_t : public cmd_base_t
{
    float axis_target_pos[7];
};

# ModuleDeps
ModuleDeps _module_deps;

void configure(const ModuleDeps &deps)     //对外注入接口
{
    _module_deps = deps;
}

# ModuleCtx
ModuleCtx _ctx;

[[nodiscard]] const ModuleCtx &get_ctx() const      //对外线程读取接口
{
    return _ctx;
}

ctx 中的内容取代之前的全局变量，由模板自带互斥锁保护，不再占用全局栈
_ctx 的类型由 arm_module_params_t::ModuleCtx 决定

## 3.如何使用子类
1. 上层调用：
    arm_cmd_t cmd;
    arm_module_t::instance()->set_command(cmd);
    模板内部执行逻辑：锁互斥量_mutex，使用 CmdType 类型的环形缓冲区 _cmd_buffer，
    拷贝 arm_cmd_t 存入缓冲，释放锁，仅需单次拷贝结构体
2. 循环更新指令：
    模板循环自动调用
    void _update_command()
    {
        scoped_mutex_t lock(_mutex);
        if (_head != _tail)
        {
            // CmdType = arm_cmd_t
            _current_cmd = _cmd_buffer[_tail];
            _tail++;
        }
    }
    业务 _fsm_execute() 直接读取 _current_cmd（arm_cmd_t）获取目标值
3. 上层读取实时状态：
    const auto& ctx = arm_module_t::instance()->get_ctx();
    float pos = ctx.axis_current_pos[0];
    get_ctx() 返回 ModuleCtx&，模板内部加锁保证多线程安全