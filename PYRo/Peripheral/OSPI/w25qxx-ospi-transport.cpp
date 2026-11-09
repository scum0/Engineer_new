/**
 *******************************************************************************
 * @file    w25qxx-ospi-transport.cpp
 * @brief   基于 STM32 OSPI 外设的 W25Qxx 传输层实现
 *******************************************************************************
 * @note
 *
 * none
 *
 *******************************************************************************
 */


/* ------- define --------------------------------------------------------------------------------------------------*/



/* ------- include ---------------------------------------------------------------------------------------------------*/

#include "w25qxx-ospi-transport.h"

#include "pyro_log.h"
#include "stm32h7xx_hal.h"


/* ------- class prototypes-----------------------------------------------------------------------------------------*/

static FlashErrCode cmdConfig(const std::vector<W25QxxMsgElement>& msg, OSPI_RegularCmdTypeDef* cmd, uint8_t** pData);


/* ------- macro -----------------------------------------------------------------------------------------------------*/

#define FLASH_XFER_TIMEOUT_TICKS  1000
#define FLASH_ERASE_TIMEOUT_TICKS 10000


/* ------- variables -------------------------------------------------------------------------------------------------*/

/* 系统中只有一路 OSPI，中断回调通过该指针访问传输对象 */
static W25QxxOspiTransport* g_ospiTransport = nullptr;


/* ------- function implement ----------------------------------------------------------------------------------------*/

static void notifyFromIsr(const SemaphoreHandle_t sem) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

W25QxxOspiTransport::W25QxxOspiTransport(OSPI_HandleTypeDef* hospi) : _hospi(hospi) {

    _txDone   = xSemaphoreCreateBinary();
    _cmdDone  = xSemaphoreCreateBinary();
    _rxDone   = xSemaphoreCreateBinary();
    _flagDone = xSemaphoreCreateBinary();
    _busLock  = xSemaphoreCreateMutex();

    configASSERT(_txDone);
    configASSERT(_cmdDone);
    configASSERT(_rxDone);
    configASSERT(_flagDone);
    configASSERT(_busLock);

    g_ospiTransport = this;
}

W25QxxOspiTransport::~W25QxxOspiTransport() {
    if (g_ospiTransport == this) {
        g_ospiTransport = nullptr;
    }

    vSemaphoreDelete(_txDone);
    vSemaphoreDelete(_cmdDone);
    vSemaphoreDelete(_rxDone);
    vSemaphoreDelete(_flagDone);
    vSemaphoreDelete(_busLock);
}

void W25QxxOspiTransport::notifyTxComplete() const { notifyFromIsr(_txDone); }

void W25QxxOspiTransport::notifyCmdComplete() const { notifyFromIsr(_cmdDone); }

void W25QxxOspiTransport::notifyRxComplete() const { notifyFromIsr(_rxDone); }

void W25QxxOspiTransport::notifyStatusMatch() const { notifyFromIsr(_flagDone); }


FlashErrCode W25QxxOspiTransport::transmit(const std::vector<W25QxxMsgElement>& msgs) const {

    uint8_t* pData                  = nullptr;
    OSPI_RegularCmdTypeDef sCommand = {};

    if (cmdConfig(msgs, &sCommand, &pData) != FlashErrCode::SUCCESS) {
        PYRO_LOGE("OSPI", "transmit cmd config failed");
        return FlashErrCode::INVALID;
    }

    if (xSemaphoreTake(_busLock, portMAX_DELAY) != pdTRUE) {
        return FlashErrCode::BUSY;
    }

    FlashErrCode ret = FlashErrCode::SUCCESS;

    if (sCommand.DataMode == 0) {
        /* 纯命令：IT 方式下发，等待命令完成中断 */
        xSemaphoreTake(_cmdDone, 0);
        if (HAL_OSPI_Command_IT(_hospi, &sCommand) != HAL_OK) {
            ret = FlashErrCode::INVALID;
        }
        if (ret == FlashErrCode::SUCCESS && xSemaphoreTake(_cmdDone, FLASH_ERASE_TIMEOUT_TICKS) != pdTRUE) {
            ret = FlashErrCode::TIMEOUT;
        }
    } else {
        /* 带数据：下发命令后 DMA 发送，等待发送完成中断 */
        xSemaphoreTake(_txDone, 0);
        if (HAL_OSPI_Command(_hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            ret = FlashErrCode::INVALID;
        } else {
            if (HAL_OSPI_Transmit_DMA(_hospi, pData) != HAL_OK) {
                ret = FlashErrCode::INVALID;
            } else if (xSemaphoreTake(_txDone, FLASH_XFER_TIMEOUT_TICKS) != pdTRUE) {
                ret = FlashErrCode::TIMEOUT;
            }
        }
    }

    if (ret != FlashErrCode::SUCCESS) {
        PYRO_LOGE("OSPI", "transmit failed, ret=%d, state=0x%08lX, err=0x%08lX",
                  static_cast<int>(ret), (unsigned long)HAL_OSPI_GetState(_hospi),
                  (unsigned long)HAL_OSPI_GetError(_hospi));
    }

    xSemaphoreGive(_busLock);
    return ret;
}

FlashErrCode W25QxxOspiTransport::receive(const std::vector<W25QxxMsgElement>& msgs) const {

    uint8_t* pData                  = nullptr;
    OSPI_RegularCmdTypeDef sCommand = {};

    if (cmdConfig(msgs, &sCommand, &pData) != FlashErrCode::SUCCESS) {
        PYRO_LOGE("OSPI", "receive cmd config failed");
        return FlashErrCode::INVALID;
    }

    if (xSemaphoreTake(_busLock, portMAX_DELAY) != pdTRUE) {
        return FlashErrCode::BUSY;
    }

    FlashErrCode ret = FlashErrCode::SUCCESS;

    xSemaphoreTake(_rxDone, 0);
    if (HAL_OSPI_Command(_hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        ret = FlashErrCode::INVALID;
    } else if (HAL_OSPI_Receive_DMA(_hospi, pData) != HAL_OK) {
        /* DMA 启动失败时不会再产生完成中断，直接返回，避免白等一次超时 */
        ret = FlashErrCode::INVALID;
    } else {
        if (xSemaphoreTake(_rxDone, FLASH_XFER_TIMEOUT_TICKS) != pdTRUE) {
            ret = FlashErrCode::TIMEOUT;
        }
    }

    if (ret != FlashErrCode::SUCCESS) {
        PYRO_LOGE("OSPI", "receive failed, ret=%d, state=0x%08lX, err=0x%08lX",
                  static_cast<int>(ret), (unsigned long)HAL_OSPI_GetState(_hospi),
                  (unsigned long)HAL_OSPI_GetError(_hospi));
    }

    xSemaphoreGive(_busLock);
    return ret;
}

FlashErrCode W25QxxOspiTransport::autoPolling(std::vector<W25QxxMsgElement>& msgs, uint32_t target, uint32_t mask,
                                              uint8_t interval) const {

    uint8_t* pData                  = nullptr;
    OSPI_RegularCmdTypeDef sCommand = {};

    if (cmdConfig(msgs, &sCommand, &pData) != FlashErrCode::SUCCESS) {
        PYRO_LOGE("OSPI", "autoPolling cmd config failed");
        return FlashErrCode::INVALID;
    }

    if (xSemaphoreTake(_busLock, portMAX_DELAY) != pdTRUE) {
        return FlashErrCode::BUSY;
    }

    FlashErrCode ret = FlashErrCode::SUCCESS;

    const uint32_t state = HAL_OSPI_GetState(_hospi);
    if (state == HAL_OSPI_STATE_BUSY_TX || state == HAL_OSPI_STATE_BUSY_CMD) {
        ret = FlashErrCode::BUSY;
    } else if (state != HAL_OSPI_STATE_READY) {
        ret = FlashErrCode::INVALID;
    } else if (HAL_OSPI_Command(_hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        ret = FlashErrCode::INVALID;
    } else {
        OSPI_AutoPollingTypeDef aConfig = {};
        aConfig.Match                   = target;
        aConfig.Mask                    = mask;
        aConfig.Interval                = interval;
        aConfig.AutomaticStop           = HAL_OSPI_AUTOMATIC_STOP_ENABLE;
        aConfig.MatchMode               = HAL_OSPI_MATCH_MODE_AND;

        xSemaphoreTake(_flagDone, 0);
        if (HAL_OSPI_AutoPolling_IT(_hospi, &aConfig) != HAL_OK) {
            ret = FlashErrCode::INVALID;
        } else if (xSemaphoreTake(_flagDone, FLASH_XFER_TIMEOUT_TICKS) != pdTRUE) {
            ret = FlashErrCode::TIMEOUT;
        }
    }

    if (ret != FlashErrCode::SUCCESS) {
        PYRO_LOGE("OSPI", "autoPolling failed, ret=%d", static_cast<int>(ret));
    }

    xSemaphoreGive(_busLock);
    return ret;
}


static FlashErrCode cmdConfig(const std::vector<W25QxxMsgElement>& msg, OSPI_RegularCmdTypeDef* cmd, uint8_t** pData) {

    cmd->OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd->FlashId       = HAL_OSPI_FLASH_ID_1;
    cmd->DQSMode       = HAL_OSPI_DQS_DISABLE;
    cmd->SIOOMode      = HAL_OSPI_SIOO_INST_EVERY_CMD;

    for (auto& eachMsg : msg) {
        switch (eachMsg.getType()) {

            case MsgTypeEnum::INSTRUCTION: {
                cmd->Instruction        = eachMsg.getByte();
                cmd->InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
                cmd->InstructionSize    = (eachMsg.getNum() - 1) << 4;
                cmd->InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

            } break;

            case MsgTypeEnum::ADDRESS: {
                cmd->Address        = eachMsg.getByte();
                cmd->AddressMode    = eachMsg.getLines() << 8;
                cmd->AddressSize    = (eachMsg.getNum() - 1) << 12;
                cmd->AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
            } break;

            case MsgTypeEnum::ALTERNATE_BYTE: {
                cmd->AlternateBytes        = eachMsg.getByte();
                cmd->AlternateBytesMode    = eachMsg.getLines() << 16;
                cmd->AlternateBytesSize    = (eachMsg.getNum() - 1) << 20;
                cmd->AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
            } break;

            case MsgTypeEnum::DUMMY_CYCLE: {
                cmd->DummyCycles = eachMsg.getNum();
            } break;

            case MsgTypeEnum::DATA: {
                cmd->DataMode    = eachMsg.getLines() << 24;
                cmd->NbData      = eachMsg.getNum();
                cmd->DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
                *pData           = eachMsg.getPData();
            } break;
            default:
                return FlashErrCode::INVALID;
        }
    }

    return FlashErrCode::SUCCESS;
}



/*---- interrupt -----------------------------------------------------------------------------------------------------*/

void HAL_OSPI_RxCpltCallback(OSPI_HandleTypeDef* hospi) {
    if (g_ospiTransport != nullptr && hospi == g_ospiTransport->handle()) {
        g_ospiTransport->notifyRxComplete();
    }
}

void HAL_OSPI_CmdCpltCallback(OSPI_HandleTypeDef* hospi) {
    if (g_ospiTransport != nullptr && hospi == g_ospiTransport->handle()) {
        g_ospiTransport->notifyCmdComplete();
    }
}

void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef* hospi) {
    if (g_ospiTransport != nullptr && hospi == g_ospiTransport->handle()) {
        g_ospiTransport->notifyTxComplete();
    }
}

void HAL_OSPI_StatusMatchCallback(OSPI_HandleTypeDef* hospi) {
    if (g_ospiTransport != nullptr && hospi == g_ospiTransport->handle()) {
        g_ospiTransport->notifyStatusMatch();
    }
}
