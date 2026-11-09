/**
 * @file pyro_flash_fs.h
 * @brief 外部 Flash（W25Qxx + OSPI）文件系统应用层
 *
 * 职责：
 *   1. 初始化 OSPI 传输层与 W25Qxx 设备层；
 *   2. 读取并校验 JEDEC ID（验证 OSPI 通路）；
 *   3. 挂载 LittleFS（失败则格式化后重试）；
 *   4. 对外提供简单的文件读写 / 删除接口。
 *
 * 使用方式：
 *   pyro::flash_fs_t::instance().init();
 *   ... 之后可用 read_file / write_file / remove_file ...
 */

#ifndef __PYRO_FLASH_FS_H__
#define __PYRO_FLASH_FS_H__

#include "pyro_core_config.h"
#include "w25qxx.h"
#include "w25qxx-ospi-transport.h"
#include "lfs.h"

#include <cstdint>
#include <cstddef>

namespace pyro
{

class flash_fs_t
{
  public:
    static flash_fs_t &instance();

    /**
     * @brief 初始化 Flash 与文件系统
     * @return true 成功，false 失败
     * @note 内部会先读 JEDEC ID 验证 OSPI 通路，再挂载 LittleFS
     */
    bool init();

    /** @brief 文件系统是否已就绪 */
    [[nodiscard]] bool is_ready() const { return _mounted; }

    /**
     * @brief 写入（覆盖）一个文件
     * @param path 文件路径（如 "/log.txt"）
     * @param data 数据
     * @param size 长度
     * @return true 成功
     */
    bool write_file(const char *path, const void *data, uint32_t size);

    /**
     * @brief 读取一个文件
     * @param path   文件路径
     * @param buffer 接收缓冲区
     * @param size   缓冲区大小
     * @return 实际读取的字节数，失败返回 -1
     */
    int32_t read_file(const char *path, void *buffer, uint32_t size);

    /**
     * @brief 删除一个文件
     * @param path 文件路径
     * @return true 成功
     */
    bool remove_file(const char *path);

    /** @brief 获取底层 Flash 设备（供日志后端等使用） */
    [[nodiscard]] W25Qxx &flash() { return _flash; }

    /**
     * @brief 读写自检：写测试文件 -> 读回 -> 逐字节比对 -> 删除
     * @return true 全部通过
     * @note 用于验证 应用层 -> LittleFS -> 适配层 -> W25Qxx -> OSPI/MDMA 整条链路
     */
    bool self_test();

    /**
     * @brief 检查并落盘上次崩溃记录（.noinit -> /crash.log）
     * @return true 有记录且已落盘
     * @note 需在文件系统挂载成功后调用
     */
    bool flush_crash_log();

  private:
    flash_fs_t();
    ~flash_fs_t() = default;
    flash_fs_t(const flash_fs_t &)            = delete;
    flash_fs_t &operator=(const flash_fs_t &) = delete;

    /** @brief 读取并打印 JEDEC ID，验证 OSPI 通路 */
    bool verify_jedec_id();

    /** @brief 配置并挂载 LittleFS（失败则格式化后重试） */
    bool mount_fs();

    W25QxxOspiTransport _transport;
    W25Qxx _flash;
    lfs_t _lfs{};
    lfs_config _cfg{};
    bool _mounted = false;
};

} // namespace pyro

/*---------------------------------------------------------------------------*/
/* C 风格统一入口：应用层只需在初始化流程中调用 pyro_flash_fs_init() */
/*---------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 初始化外部 Flash 文件系统（含崩溃日志落盘）
 * @return 1 成功，0 失败
 * @note 内部完成：读 JEDEC ID 验证 -> 挂载 LittleFS -> 落盘上次崩溃记录。
 *       需在 FreeRTOS 调度器启动后调用（内部会创建信号量）。
 *       仅当 elog_cfg.h 中选择 ELOG_OUTPUT_BACKEND_FLASH 时有效，否则直接返回 0。
 */
int pyro_flash_fs_init(void);

/**
 * @brief 文件系统是否已就绪
 * @return 1 就绪，0 未就绪
 */
int pyro_flash_fs_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* __PYRO_FLASH_FS_H__ */
