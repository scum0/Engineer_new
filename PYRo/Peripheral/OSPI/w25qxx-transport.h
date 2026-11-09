/**
 *******************************************************************************
 * @file    w25qxx-transport.h
 * @brief   W25Qxx 命令模型与传输层接口（与具体总线实现无关）
 *******************************************************************************
 * @note
 *
 *  W25Qxx 芯片驱动只依赖本文件的 IFlashTransport 抽象：
 *  - 设备层（w25qxx.h）负责“要发什么命令”；
 *  - 传输层（如 w25qxx-ospi-transport.*）负责“命令怎么经总线发出去”。
 *
 *  这样更换总线（OSPI / SPI / QSPI）时无需改动设备层与上层。
 *
 *******************************************************************************
 */

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/

#pragma once



/*-------- 1. includes and imports -----------------------------------------------------------------------------------*/

#include "flash-intf.h"
#include "w25qxx-inst.h"

#include <cstdint>
#include <vector>



/*-------- 2. enum ---------------------------------------------------------------------------------------------------*/

/**
 * @brief 通信模式（数据线数量）
 */
enum class W25QxxCommModeEnum {
    NONE,
    STANDARD_SPI,
    DSPI,
    QSPI,
    OSPI,
};

/**
 * @brief 命令阶段类型
 */
enum class MsgTypeEnum {
    NONE,
    INSTRUCTION,
    ADDRESS,
    ALTERNATE_BYTE,
    DUMMY_CYCLE,
    DATA,
};



/*-------- 3. interface ---------------------------------------------------------------------------------------------*/

/**
 * @brief 一条命令的一个阶段（指令 / 地址 / 交替字节 / 空周期 / 数据）
 */
class W25QxxMsgElement {

  public:
    /**
     * @brief Initialize of address, alternate bytes, dummy-cycle
     * @param type Data type
     * @param byte 32bit
     * @param num number of bytes
     * @param mode lines
     */
    W25QxxMsgElement(const MsgTypeEnum type, const uint32_t byte, const uint16_t num, const W25QxxCommModeEnum mode)
        : _type(type), _mode(mode), _num(num), _byte(byte) {
        if (type == MsgTypeEnum::DATA || type == MsgTypeEnum::INSTRUCTION) {
            _type = MsgTypeEnum::NONE;
        }
    }

    /**
     * @brief initialize of data
     * @param type DATA
     * @param pData pointer of data
     * @param num length
     * @param mode lines
     */
    W25QxxMsgElement(const MsgTypeEnum type, uint8_t* pData, const uint16_t num, const W25QxxCommModeEnum mode)
        : _type(type), _mode(mode), _num(num), _pData(pData) {
        if (type != MsgTypeEnum::DATA) {
            _type = MsgTypeEnum::NONE;
        }
    }

    /**
     * @brief initialize of instruction
     * @param type INSTRUCTION
     * @param inst content
     */
    W25QxxMsgElement(const MsgTypeEnum type, W25QxxInst inst)
        : _type(type), _mode(W25QxxCommModeEnum::STANDARD_SPI), _num(1), _byte(static_cast<uint8_t>(inst)) {
        if (type != MsgTypeEnum::INSTRUCTION) {
            _type = MsgTypeEnum::NONE;
        }
    }

    W25QxxMsgElement(const MsgTypeEnum type, const uint16_t num) : _type(type), _num(num) {
        if (type != MsgTypeEnum::DUMMY_CYCLE || num > 31) {
            _type = MsgTypeEnum::NONE;
        }
    }

    /*********** setter & getter *************/
    [[nodiscard]] MsgTypeEnum getType() const { return _type; }
    [[nodiscard]] uint16_t getNum() const { return _num; }
    [[nodiscard]] uint32_t getByte() const { return _byte; }
    [[nodiscard]] uint8_t* getPData() const { return _pData; }
    [[nodiscard]] W25QxxCommModeEnum getMode() const { return _mode; }
    [[nodiscard]] uint8_t getLines() const { return static_cast<uint8_t>(_mode); }

  private:
    MsgTypeEnum _type = MsgTypeEnum::NONE;
    W25QxxCommModeEnum _mode;
    uint16_t _num;
    uint32_t _byte;
    uint8_t* _pData = nullptr;
};


/**
 * @brief W25Qxx 传输层接口
 *
 * @note 实现负责把命令模型翻译成具体总线的时序；上层不感知 HAL / OSPI。
 */
class IFlashTransport {
  public:
    virtual ~IFlashTransport() = default;

    /** @brief 发送一组命令（含数据阶段时表示写数据） */
    [[nodiscard]] virtual FlashErrCode transmit(const std::vector<W25QxxMsgElement>& msgs) const = 0;

    /** @brief 发送读取命令并接收数据 */
    [[nodiscard]] virtual FlashErrCode receive(const std::vector<W25QxxMsgElement>& msgs) const = 0;

    /** @brief 自动轮询状态寄存器直到匹配 target/mask */
    [[nodiscard]] virtual FlashErrCode autoPolling(std::vector<W25QxxMsgElement>& msgs, uint32_t target, uint32_t mask,
                                                   uint8_t interval) const = 0;
};
