/**
 * @file pyro_drv_log.h
 * @brief PYRo 驱动日志宏（由 UES_LOG 控制）
 *
 * 未定义 UES_LOG 时，PYRO_DRV_LOGx 全部展开为 ((void)0)，
 * 不生成任何日志代码，驱动行为与未加日志时完全一致。
 *
 * 定义 UES_LOG（例如编译选项 -DUES_LOG，或在 pyro_core_config.h 中
 * 取消对应注释）后，驱动日志经 PYRO_LOGx 输出到应用层注入的日志处理器。
 */

#ifndef __PYRO_DRV_LOG_H__
#define __PYRO_DRV_LOG_H__

#include "pyro_core_config.h"
#include "pyro_log.h"

#ifdef UES_LOG
#define PYRO_DRV_LOG_ENABLED 1

#define PYRO_DRV_LOGA(tag, ...) PYRO_LOG(PYRO_LOG_LVL_ASSERT, tag, __VA_ARGS__)
#define PYRO_DRV_LOGE(tag, ...) PYRO_LOG(PYRO_LOG_LVL_ERROR, tag, __VA_ARGS__)
#define PYRO_DRV_LOGW(tag, ...) PYRO_LOG(PYRO_LOG_LVL_WARN, tag, __VA_ARGS__)
#define PYRO_DRV_LOGI(tag, ...) PYRO_LOG(PYRO_LOG_LVL_INFO, tag, __VA_ARGS__)
#define PYRO_DRV_LOGD(tag, ...) PYRO_LOG(PYRO_LOG_LVL_DEBUG, tag, __VA_ARGS__)
#define PYRO_DRV_LOGV(tag, ...) PYRO_LOG(PYRO_LOG_LVL_VERBOSE, tag, __VA_ARGS__)
#else
#define PYRO_DRV_LOG_ENABLED 0

#define PYRO_DRV_LOGA(tag, ...) ((void)0)
#define PYRO_DRV_LOGE(tag, ...) ((void)0)
#define PYRO_DRV_LOGW(tag, ...) ((void)0)
#define PYRO_DRV_LOGI(tag, ...) ((void)0)
#define PYRO_DRV_LOGD(tag, ...) ((void)0)
#define PYRO_DRV_LOGV(tag, ...) ((void)0)
#endif

#endif /* __PYRO_DRV_LOG_H__ */
