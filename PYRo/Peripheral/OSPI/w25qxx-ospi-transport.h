/**
 *******************************************************************************
 * @file    w25qxx-ospi-transport.h
 * @brief   基于 STM32 OSPI 外设的 W25Qxx 传输层实现
 *******************************************************************************
 * @note
 *
 *  传输层自持传输完成信号量与互斥量，并把 HAL 的 OSPI 中断回调收敛到本模块，
 *  因此 transmit / receive / autoPolling 均为阻塞语义：
 *  函数返回时总线操作已经完成（或已超时），上层无需再关心信号量。
 *
 *******************************************************************************
 */

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/

#pragma once



/*-------- 1. includes and imports -----------------------------------------------------------------------------------*/

#include "w25qxx-transport.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_ospi.h"



/*-------- 2. interface ---------------------------------------------------------------------------------------------*/

/**
 * @brief W25Qxx 的 OSPI 传输实现
 */
class W25QxxOspiTransport : public IFlashTransport {
  public:
    explicit W25QxxOspiTransport(OSPI_HandleTypeDef* hospi);
    ~W25QxxOspiTransport() override;

    W25QxxOspiTransport(const W25QxxOspiTransport&)            = delete;
    W25QxxOspiTransport& operator=(const W25QxxOspiTransport&) = delete;

    [[nodiscard]] FlashErrCode transmit(const std::vector<W25QxxMsgElement>& msgs) const override;
    [[nodiscard]] FlashErrCode receive(const std::vector<W25QxxMsgElement>& msgs) const override;
    [[nodiscard]] FlashErrCode autoPolling(std::vector<W25QxxMsgElement>& msgs, uint32_t target, uint32_t mask,
                                           uint8_t interval) const override;

    [[nodiscard]] OSPI_HandleTypeDef* handle() const { return _hospi; }

    /* 以下由 HAL 中断回调调用（ISR 上下文） */
    void notifyTxComplete() const;
    void notifyCmdComplete() const;
    void notifyRxComplete() const;
    void notifyStatusMatch() const;

  private:
    OSPI_HandleTypeDef* _hospi = nullptr;

    SemaphoreHandle_t _txDone   = nullptr;
    SemaphoreHandle_t _cmdDone  = nullptr;
    SemaphoreHandle_t _rxDone   = nullptr;
    SemaphoreHandle_t _flagDone = nullptr;
    SemaphoreHandle_t _busLock  = nullptr;
};
