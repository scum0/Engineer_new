/**
 * @file pyro_flash_fs.cpp
 * @brief 外部 Flash（W25Qxx + OSPI）文件系统应用层实现
 */

#include "pyro_flash_fs.h"

#include "adapter-lfs.h"
#include "elog_cfg.h"
#include "elog_flash.h"
#include "pyro_crashlog.h"
#include "octospi.h"
#include "pyro_log.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace pyro
{

/*
 * LittleFS 缓存缓冲区。
 * read/prog 缓冲区会作为 MDMA 的源/目标（经 LfsAdapter -> W25Qxx -> OSPI），
 * 必须放在 DMA/MDMA 可访问的 RAM 区，不能放 DTCMRAM。
 * .dma_heap 段在链接脚本中映射到 RAM_D2。
 */
__attribute__((section(".dma_heap"))) static uint8_t s_lfs_read_buf[LFS_CACHE_SIZE];
__attribute__((section(".dma_heap"))) static uint8_t s_lfs_prog_buf[LFS_CACHE_SIZE];
__attribute__((section(".dma_heap"))) static uint8_t s_lfs_lookahead_buf[LFS_LOOKAHEAD_SIZE];

/* 自检用的 DMA 可访问缓冲区 */
static constexpr uint32_t SELF_TEST_SIZE = 256;
__attribute__((section(".dma_heap"))) static uint8_t s_test_tx[SELF_TEST_SIZE];
__attribute__((section(".dma_heap"))) static uint8_t s_test_rx[SELF_TEST_SIZE];

/* 崩溃日志落盘用的文本缓冲（写入 Flash 时作为 MDMA 源，必须 DMA 可访问） */
static constexpr uint32_t CRASH_TEXT_SIZE = 4096;
__attribute__((section(".dma_heap"))) static char s_crash_text[CRASH_TEXT_SIZE];

flash_fs_t &flash_fs_t::instance()
{
    /*
     * 单例整体放在 .dma_heap（RAM_D2）：
     * W25Qxx 内部的 _pRxBuff 会作为 MDMA 的目标缓冲区（读 ID / 状态寄存器），
     * 必须位于 MDMA 可访问的 RAM，不能落在 DTCMRAM。
     */
    __attribute__((section(".dma_heap"))) static flash_fs_t s_instance;
    return s_instance;
}

flash_fs_t::flash_fs_t()
    : _transport(&hospi1), _flash(_transport)
{
}

bool flash_fs_t::verify_jedec_id()
{
    /* 读取 JEDEC ID：制造商 ID + 设备 ID */
    const FlashErrCode ret = _flash.enquireJedecIdAsync();
    _flash.asyncRxCallback();

    const uint8_t  mid  = _flash.getMID();
    const uint16_t did  = _flash.getDevID16();

    if (ret != FlashErrCode::SUCCESS)
    {
        PYRO_LOGE("FLASH",
                  "JEDEC read transfer failed, ret=%d, MID=0x%02X, DID=0x%04X, OSPI err=0x%08lX",
                  static_cast<int>(ret), mid, did,
                  (unsigned long)HAL_OSPI_GetError(&hospi1));
        return false;
    }

    PYRO_LOGI("FLASH", "JEDEC ID: MID=0x%02X, DID=0x%04X", mid, did);

    /* W25Q64：制造商 0xEF（Winbond），设备 ID 0x4017 */
    if (mid == 0xEFU && did == 0x4017U)
    {
        PYRO_LOGI("FLASH", "W25Q64 detected, capacity=%lu bytes",
                  (unsigned long)_flash.capacity());
        return true;
    }

    PYRO_LOGE("FLASH", "unexpected JEDEC ID, check OSPI wiring / clock");
    return false;
}

bool flash_fs_t::mount_fs()
{
    /* 配置 LittleFS 块设备接口 */
    _cfg.read             = LfsAdapter::read;
    _cfg.prog             = LfsAdapter::program;
    _cfg.erase            = LfsAdapter::erase;
    _cfg.sync             = LfsAdapter::sync;
    _cfg.read_size        = LFS_READ_SIZE;
    _cfg.prog_size        = _flash.pageSize();
    _cfg.block_size       = _flash.sectorSize();
    /*
     * 末尾 ELOG_FLASH_SECTOR_NUM 个扇区预留给日志 Flash 后端使用，
     * LittleFS 只使用前面的扇区，避免两者互相覆盖。
     */
    const uint32_t total_sectors = _flash.sectorCount();
    _cfg.block_count      = (total_sectors > ELOG_FLASH_SECTOR_NUM)
                                ? (total_sectors - ELOG_FLASH_SECTOR_NUM)
                                : total_sectors;
    _cfg.block_cycles     = LFS_BLOCK_CYCLES;
    _cfg.cache_size       = LFS_CACHE_SIZE;
    _cfg.lookahead_size   = LFS_LOOKAHEAD_SIZE;
    _cfg.read_buffer      = s_lfs_read_buf;
    _cfg.prog_buffer      = s_lfs_prog_buf;
    _cfg.lookahead_buffer = s_lfs_lookahead_buf;
    _cfg.compact_thresh   = -1;

    /* 挂载，失败则格式化后重试 */
    int lfs_ret = lfs_mount(&_lfs, &_cfg);
    if (lfs_ret == 0)
    {
        PYRO_LOGI("FLASH", "LittleFS mount success");
        return true;
    }

    PYRO_LOGW("FLASH", "LittleFS mount failed (%d), formatting...", lfs_ret);
    lfs_ret = lfs_format(&_lfs, &_cfg);
    if (lfs_ret != 0)
    {
        PYRO_LOGE("FLASH", "LittleFS format failed (%d)", lfs_ret);
        return false;
    }

    lfs_ret = lfs_mount(&_lfs, &_cfg);
    if (lfs_ret != 0)
    {
        PYRO_LOGE("FLASH", "LittleFS mount after format failed (%d)", lfs_ret);
        return false;
    }

    PYRO_LOGI("FLASH", "LittleFS formatted and mounted");
    return true;
}

bool flash_fs_t::init()
{
    /* 1. 验证 OSPI 通路：读 JEDEC ID */
    if (!verify_jedec_id())
    {
        return false;
    }

    /* 2. 把设备注入日志 Flash 后端（若日志后端选择 FLASH 时生效） */
    /*    注：是否真正用于日志取决于 elog_cfg.h 的后端选择（ELOG_OUTPUT_BACKEND_FLASH） */
    elog_port_flash_set_device(&_flash);

    /* 3. 挂载文件系统 */
    LfsAdapter::config(&_flash);
    if (!mount_fs())
    {
        return false;
    }

    _mounted = true;

    /* 4. 若上次发生崩溃，把 .noinit 中的崩溃记录落盘到 /crash.log */
    (void)flush_crash_log();

    return true;
}

bool flash_fs_t::write_file(const char *path, const void *data, uint32_t size)
{
    if (!_mounted || path == nullptr || data == nullptr)
    {
        return false;
    }

    lfs_file_t file{};
    if (lfs_file_open(&_lfs, &file, path, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) != 0)
    {
        PYRO_LOGE("FLASH", "open for write failed: %s", path);
        return false;
    }

    const lfs_ssize_t written = lfs_file_write(&_lfs, &file, data, size);
    const int close_ret       = lfs_file_close(&_lfs, &file);

    if (written < 0 || static_cast<uint32_t>(written) != size || close_ret != 0)
    {
        PYRO_LOGE("FLASH", "write failed: %s, written=%ld", path, (long)written);
        return false;
    }

    return true;
}

int32_t flash_fs_t::read_file(const char *path, void *buffer, uint32_t size)
{
    if (!_mounted || path == nullptr || buffer == nullptr)
    {
        return -1;
    }

    lfs_file_t file{};
    if (lfs_file_open(&_lfs, &file, path, LFS_O_RDONLY) != 0)
    {
        PYRO_LOGE("FLASH", "open for read failed: %s", path);
        return -1;
    }

    const lfs_ssize_t read_len = lfs_file_read(&_lfs, &file, buffer, size);
    lfs_file_close(&_lfs, &file);

    if (read_len < 0)
    {
        PYRO_LOGE("FLASH", "read failed: %s", path);
        return -1;
    }

    return static_cast<int32_t>(read_len);
}

bool flash_fs_t::remove_file(const char *path)
{
    if (!_mounted || path == nullptr)
    {
        return false;
    }

    if (lfs_remove(&_lfs, path) != 0)
    {
        PYRO_LOGE("FLASH", "remove failed: %s", path);
        return false;
    }

    return true;
}

bool flash_fs_t::self_test()
{
    if (!_mounted)
    {
        PYRO_LOGE("FLASH", "self test skipped: filesystem not mounted");
        return false;
    }

    constexpr const char *TEST_PATH = "/selftest.bin";

    PYRO_LOGI("FLASH", "self test start, size=%lu", (unsigned long)SELF_TEST_SIZE);

    /* 1. 填充测试模式（含 0x00 / 0xFF 边界值，便于发现位翻转） */
    for (uint32_t i = 0; i < SELF_TEST_SIZE; ++i)
    {
        s_test_tx[i] = static_cast<uint8_t>(i ^ 0xA5U);
    }
    s_test_tx[0]                  = 0x00U;
    s_test_tx[SELF_TEST_SIZE - 1] = 0xFFU;

    /* 2. 写入文件 */
    if (!write_file(TEST_PATH, s_test_tx, SELF_TEST_SIZE))
    {
        PYRO_LOGE("FLASH", "self test FAIL: write");
        return false;
    }
    PYRO_LOGI("FLASH", "self test: write ok");

    /* 3. 清空接收缓冲，避免残留数据干扰比对 */
    memset(s_test_rx, 0, SELF_TEST_SIZE);

    /* 4. 读回文件 */
    const int32_t read_len = read_file(TEST_PATH, s_test_rx, SELF_TEST_SIZE);
    if (read_len != static_cast<int32_t>(SELF_TEST_SIZE))
    {
        PYRO_LOGE("FLASH", "self test FAIL: read len=%ld, expect=%lu",
                  (long)read_len, (unsigned long)SELF_TEST_SIZE);
        return false;
    }
    PYRO_LOGI("FLASH", "self test: read ok");

    /* 5. 逐字节比对，打印首个不一致的位置 */
    for (uint32_t i = 0; i < SELF_TEST_SIZE; ++i)
    {
        if (s_test_tx[i] != s_test_rx[i])
        {
            PYRO_LOGE("FLASH", "self test FAIL: mismatch at %lu, tx=0x%02X rx=0x%02X",
                      (unsigned long)i, s_test_tx[i], s_test_rx[i]);
            return false;
        }
    }
    PYRO_LOGI("FLASH", "self test: compare ok");

    /* 6. 删除测试文件 */
    if (!remove_file(TEST_PATH))
    {
        PYRO_LOGW("FLASH", "self test: cleanup remove failed");
    }

    PYRO_LOGI("FLASH", "self test PASS");
    return true;
}

bool flash_fs_t::flush_crash_log()
{
    if (!_mounted)
    {
        return false;
    }

    const pyro_crashlog_record_t *rec = pyro_crashlog_get_pending();
    if (rec == nullptr)
    {
        return false;
    }

    /*
     * 崩溃循环保护：同一 PC + CFSR 反复崩溃时，/crash.log 里已经保存了
     * 这个现场，不再重复擦写 Flash，只清除待落盘标志。
     * 若文件已被格式化删除，则仍然重新写入。
     */
    if (!pyro_crashloop_should_log())
    {
        struct lfs_info info{};
        if (lfs_stat(&_lfs, "/crash.log", &info) == LFS_ERR_OK)
        {
            PYRO_LOGW("FLASH",
                      "repeat crash #%lu at PC=0x%08lX, keep existing /crash.log",
                      (unsigned long)pyro_crashloop_get_count(),
                      (unsigned long)pyro_crashloop_get_last_pc());
            pyro_crashlog_clear_pending();
            return true;
        }
    }

    /* 把崩溃记录格式化成可读文本，写入 s_crash_text */
    uint32_t off = 0;

    auto append = [&](const char *fmt, ...) {
        if (off >= CRASH_TEXT_SIZE)
        {
            return;
        }
        va_list args;
        va_start(args, fmt);
        const int n = vsnprintf(&s_crash_text[off], CRASH_TEXT_SIZE - off, fmt, args);
        va_end(args);
        if (n > 0)
        {
            off += static_cast<uint32_t>(n);
            if (off >= CRASH_TEXT_SIZE)
            {
                off = CRASH_TEXT_SIZE - 1;
            }
        }
    };

    append("===== CRASH LOG =====\r\n");
    append("Fault: R0=%08lX R1=%08lX R2=%08lX R3=%08lX\r\n",
           (unsigned long)rec->fault.r0, (unsigned long)rec->fault.r1,
           (unsigned long)rec->fault.r2, (unsigned long)rec->fault.r3);
    append("       R12=%08lX LR=%08lX PC=%08lX PSR=%08lX\r\n",
           (unsigned long)rec->fault.r12, (unsigned long)rec->fault.lr,
           (unsigned long)rec->fault.pc, (unsigned long)rec->fault.psr);
    append("       CFSR=%08lX HFSR=%08lX BFAR=%08lX MMFAR=%08lX\r\n",
           (unsigned long)rec->fault.cfsr, (unsigned long)rec->fault.hfsr,
           (unsigned long)rec->fault.bfar, (unsigned long)rec->fault.mmar);
    append("       CrashCount=%lu FirstPC=%08lX FirstCFSR=%08lX\r\n",
           (unsigned long)pyro_crashloop_get_count(),
           (unsigned long)pyro_crashloop_get_first_pc(),
           (unsigned long)pyro_crashloop_get_first_cfsr());
    append("----- recent logs (%lu) -----\r\n", (unsigned long)rec->line_count);

    for (uint32_t i = 0; i < rec->line_count; ++i)
    {
        if (rec->lines[i][0] != '\0')
        {
            append("%s\r\n", rec->lines[i]);
        }
    }
    append("===== END =====\r\n");

    /* 写入 /crash.log */
    if (!write_file("/crash.log", s_crash_text, off))
    {
        PYRO_LOGE("FLASH", "flush crash log failed");
        return false;
    }

    /* 落盘成功：记录该现场已写入，后续相同现场不再重复擦写 Flash */
    pyro_crashloop_mark_logged();

    /* 落盘成功，清除待落盘标志 */
    pyro_crashlog_clear_pending();

    PYRO_LOGW("FLASH", "crash log saved to /crash.log (%lu bytes)", (unsigned long)off);
    return true;
}

} // namespace pyro

/*---------------------------------------------------------------------------*/
/* C 风格统一入口 */
/*---------------------------------------------------------------------------*/

extern "C" int pyro_flash_fs_init(void)
{
#if defined(ELOG_OUTPUT_BACKEND_FLASH)
    if (pyro::flash_fs_t::instance().init())
    {
        PYRO_LOGI("FLASH", "flash filesystem ready");
        return 1;
    }
    PYRO_LOGE("FLASH", "flash filesystem init failed");
    return 0;
#else
    return 0;
#endif
}

extern "C" int pyro_flash_fs_is_ready(void)
{
#if defined(ELOG_OUTPUT_BACKEND_FLASH)
    return pyro::flash_fs_t::instance().is_ready() ? 1 : 0;
#else
    return 0;
#endif
}
