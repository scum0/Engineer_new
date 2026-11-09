/**
 *******************************************************************************
 * @file    w25qxx.cpp
 * @brief   简要描述
 *******************************************************************************
 * @attention
 *
 * none
 *
 *******************************************************************************
 * @note
 *
 * none
 *
 *******************************************************************************
 * @author  MekLi
 * @date    2025/8/24
 * @version 1.0
 *******************************************************************************
 */


/* ------- define --------------------------------------------------------------------------------------------------*/





/* ------- include ---------------------------------------------------------------------------------------------------*/

#include "w25qxx.h"
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"


/* ------- class prototypes-----------------------------------------------------------------------------------------*/




static void memcpy_reverse_order(void* dest, const void* src, size_t len);

/* ------- macro -----------------------------------------------------------------------------------------------------*/


/* ------- variables -------------------------------------------------------------------------------------------------*/



/* ------- function implement ----------------------------------------------------------------------------------------*/

FlashErrCode W25Qxx::enquireJedecIdAsync() {

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::READ_JEDEC_ID);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, _pRxBuff, 3, W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {inst, data};

    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    /* 3. submit the assignment request */

    const W25QxxRxOpt mID(&_manufacturerID, 1);
    const W25QxxRxOpt devID(&_deviceID16, 2);

    _rxOpts.push_back(mID);
    _rxOpts.push_back(devID);


    return ret;
}


FlashErrCode W25Qxx::enquireDeviceIdAsync() {

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::DEVICE_ID);
    const W25QxxMsgElement dum(MsgTypeEnum::DUMMY_CYCLE, 3 * 8);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, _pRxBuff, 1, W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {inst, dum, data};

    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    /* 3. submit the assignment request */

    const W25QxxRxOpt devID(&_deviceID8, 1);

    _rxOpts.push_back(devID);

    return ret;
}


FlashErrCode W25Qxx::enquireUniqueIdAsync() {
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::READ_UNIQUE_ID);
    const W25QxxMsgElement dum(MsgTypeEnum::DUMMY_CYCLE, 3 * 8);
    const W25QxxMsgElement alte(MsgTypeEnum::ALTERNATE_BYTE, static_cast<uint32_t>(0x00), 1,
                                W25QxxCommModeEnum::STANDARD_SPI);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, _pRxBuff, 4, W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {inst, dum, data, alte};

    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    /* 3. submit the assignment request */

    const W25QxxRxOpt devID(&_uniqueID, 4);

    _rxOpts.push_back(devID);

    return ret;
}

FlashErrCode W25Qxx::enquireStatusRegisterAsync(const W25QxxRegisterEnum sr) {

    /* 1. collect messages */

    auto ret   = FlashErrCode::NONE;
    auto inst  = W25QxxInst::NONE;
    void* data = nullptr;

    switch (sr) {
        case W25QxxRegisterEnum::STATUS_REGISTER_1: {
            inst = W25QxxInst::READ_SR1;
            data = &_statusRegister1;
        } break;

        case W25QxxRegisterEnum::STATUS_REGISTER_2: {
            inst = W25QxxInst::READ_SR2;
            data = &_statusRegister2;
        } break;

        case W25QxxRegisterEnum::STATUS_REGISTER_3: {
            inst = W25QxxInst::READ_SR3;
            data = &_statusRegister3;
        } break;

        default: {
            return FlashErrCode::INVALID;
        }
    }

    const W25QxxMsgElement readSrInst(MsgTypeEnum::INSTRUCTION, inst);
    const W25QxxMsgElement readSr(MsgTypeEnum::DATA, _pRxBuff, 1, W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {readSrInst, readSr};

    const W25QxxRxOpt srRequest(data, 1);

    /* 2. transmit messages */

    ret = _transport.receive(msgs);

    /* 3. submit the assignment request */

    _rxOpts.push_back(srRequest);

    return ret;
}

FlashErrCode W25Qxx::writeEnable() const {

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::WRITE_ENABLE);

    const std::vector msgs = {inst};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::writeRegister(const W25QxxRegisterEnum sr, uint8_t byte) const {

    /* 1. collect messages */

    auto ret  = FlashErrCode::NONE;
    auto inst = W25QxxInst::NONE;

    switch (sr) {
        case W25QxxRegisterEnum::STATUS_REGISTER_1: {
            inst = W25QxxInst::WRITE_SR1;
        } break;

        case W25QxxRegisterEnum::STATUS_REGISTER_2: {
            inst = W25QxxInst::WRITE_SR2;
        } break;

        case W25QxxRegisterEnum::STATUS_REGISTER_3: {
            inst = W25QxxInst::WRITE_SR3;
        } break;

        default: {
            return FlashErrCode::INVALID;
        }
    }

    const W25QxxMsgElement readSrInst(MsgTypeEnum::INSTRUCTION, inst);
    const W25QxxMsgElement writeSr(MsgTypeEnum::DATA, &byte, 1, W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {readSrInst, writeSr};

    /* 2. transmit messages */

    ret                    = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::writeEnableForVolatileStatusRegister() const {

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::WRITE_ENABLE_FOR_SR);

    const std::vector msgs = {inst};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::writeDisable() const {

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::WRITE_DISABLE);

    const std::vector msgs = {inst};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::readData(const uint32_t address, void* pData, const uint16_t len) const {

    if (pData == nullptr) {
        return FlashErrCode::INVALID;
    }

    if (len == 0) {
        return FlashErrCode::INVALID;
    }

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::READ_DATA);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, address, 3, W25QxxCommModeEnum::STANDARD_SPI);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, static_cast<uint8_t*>(pData), len, W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {
        inst,
        addr,
        data,
    };

    /* 2. transmit messages */

    const auto ret = _transport.receive(msgs);

    return ret;
}

FlashErrCode W25Qxx::fastReadData(const uint32_t address, void* pData, const uint16_t len) const {

    if (pData == nullptr) {
        return FlashErrCode::INVALID;
    }

    if (len == 0) {
        return FlashErrCode::INVALID;
    }

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::FAST_READ);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, address, 3, W25QxxCommModeEnum::STANDARD_SPI);
    const W25QxxMsgElement dummy(MsgTypeEnum::DUMMY_CYCLE, 8);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, static_cast<uint8_t*>(pData), len, W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {inst, addr, data, dummy};

    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    return ret;
}

FlashErrCode W25Qxx::fastReadDualOutput(const uint32_t address, void* pData, const uint16_t len) const {

    if (pData == nullptr) {
        return FlashErrCode::INVALID;
    }

    if (len == 0) {
        return FlashErrCode::INVALID;
    }

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::FAST_READ_DUAL_OUTPUT);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, address, 3, W25QxxCommModeEnum::STANDARD_SPI);
    const W25QxxMsgElement dummy(MsgTypeEnum::DUMMY_CYCLE, 8);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, static_cast<uint8_t*>(pData), len, W25QxxCommModeEnum::DSPI);

    const std::vector msgs = {inst, addr, data, dummy};

    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    return ret;
}

FlashErrCode W25Qxx::fastReadQuadOutput(uint32_t address, void* pData, uint16_t len) const {

    if (pData == nullptr) {
        return FlashErrCode::INVALID;
    }

    if (len == 0) {
        return FlashErrCode::INVALID;
    }

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::FAST_READ_QUAD_OUTPUT);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, address, 3, W25QxxCommModeEnum::STANDARD_SPI);
    const W25QxxMsgElement dummy(MsgTypeEnum::DUMMY_CYCLE, 8);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, static_cast<uint8_t*>(pData), len, W25QxxCommModeEnum::QSPI);

    const std::vector msgs = {inst, addr, data, dummy};


    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    return ret;
}

FlashErrCode W25Qxx::fastReadDualIO(const uint32_t address, void* pData, const uint16_t len) const {

    if (pData == nullptr) {
        return FlashErrCode::INVALID;
    }

    if (len == 0) {
        return FlashErrCode::INVALID;
    }

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::FAST_READ_DUAL_IO);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, address, 3, W25QxxCommModeEnum::DSPI);
    const W25QxxMsgElement dummy(MsgTypeEnum::DUMMY_CYCLE, 4);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, static_cast<uint8_t*>(pData), len, W25QxxCommModeEnum::DSPI);

    const std::vector msgs = {inst, addr, data, dummy};

    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    return ret;
}

FlashErrCode W25Qxx::fastReadQuadIO(const uint32_t address, void* pData, const uint16_t len) const {

    if (pData == nullptr) {
        return FlashErrCode::INVALID;
    }

    if (len == 0) {
        return FlashErrCode::INVALID;
    }

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::FAST_READ_QUAD_IO);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, address, 3, W25QxxCommModeEnum::QSPI);
    const W25QxxMsgElement dummy(MsgTypeEnum::DUMMY_CYCLE, 4);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, static_cast<uint8_t*>(pData), len, W25QxxCommModeEnum::QSPI);

    const std::vector msgs = {inst, addr, data, dummy};

    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    return ret;
}

FlashErrCode W25Qxx::setBurstWithWrap(const W25QxxWrapLenEnum len) const {

    uint8_t bit5 = 0, bit6 = 0, bit4 = 0;
    switch (len) {
        case W25QxxWrapLenEnum::NONE:
            bit4 = 1;
            break;
        case W25QxxWrapLenEnum::WRAP_8_BYTE:
            bit5 = bit6 = 0;
            break;
        case W25QxxWrapLenEnum::WRAP_16_BYTE:
            bit5 = 0;
            bit6 = 1;
            break;
        case W25QxxWrapLenEnum::WRAP_32_BYTE:
            bit5 = 1;
            bit6 = 0;
            break;
        case W25QxxWrapLenEnum::WRAP_64_BYTE:
            bit5 = bit6 = 1;
            break;
        default:
            return FlashErrCode::INVALID;
    }

    const uint8_t byte = bit4 << 4 | bit5 << 5 | bit6 << 6;

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::SET_BURST_WITH_WRAP);
    const W25QxxMsgElement dummy(MsgTypeEnum::DUMMY_CYCLE, 6);
    const W25QxxMsgElement alt(MsgTypeEnum::ALTERNATE_BYTE, byte, 1, W25QxxCommModeEnum::QSPI);

    const std::vector msgs = {inst, alt, dummy};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::pageProgram(const uint32_t address, const void* pData, const uint16_t len) const {

    if (pData == nullptr) {
        return FlashErrCode::INVALID;
    }

    if (len == 0) {
        return FlashErrCode::INVALID;
    }

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::PAGE_PROGRAM);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, address, 3, W25QxxCommModeEnum::STANDARD_SPI);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, (uint8_t*)(pData), len, W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {inst, addr, data};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::quadInputPageProgram(const uint32_t address, const void* pData, const uint16_t len) const {

    if (pData == nullptr) {
        return FlashErrCode::INVALID;
    }

    if (len == 0) {
        return FlashErrCode::INVALID;
    }

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::QUAD_INPUT_PAGE_PROGRAM);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, address, 3, W25QxxCommModeEnum::STANDARD_SPI);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, (uint8_t*)(pData), len, W25QxxCommModeEnum::QSPI);

    const std::vector msgs = {inst, addr, data};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::sectorErase(const uint32_t address) const {

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::SECTOR_ERASE);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, address, 3, W25QxxCommModeEnum::STANDARD_SPI);
    const std::vector msgs = {inst, addr};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::blockErase32KB(uint32_t address) const {

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::BLOCK_ERASE_32KB);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, address, 3, W25QxxCommModeEnum::STANDARD_SPI);
    const std::vector msgs = {inst, addr};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::blockErase64KB(uint32_t address) const {

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::BLOCK_ERASE_64KB);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, address, 3, W25QxxCommModeEnum::STANDARD_SPI);
    const std::vector msgs = {inst, addr};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::chipErase() const {
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::CHIP_ERASE);
    const std::vector msgs = {inst};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::eraseOrProgramSuspend() const {

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::ERASE_PROGRAM_SUSPEND);
    const std::vector msgs = {inst};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::eraseOrProgramResume() const {

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::ERASE_PROGRAM_RESUME);
    const std::vector msgs = {inst};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::powerDown() const {

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::POWER_DOWN);
    const std::vector msgs = {inst};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::powerOn() const {

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::RELEASE_POWER_DOWN);
    const std::vector msgs = {inst};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::enquireManDevIDAsync() {

    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::READ_MANUFACTURER_DEVICE_ID);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, static_cast<uint32_t>(0x00), 3, W25QxxCommModeEnum::STANDARD_SPI);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, _pRxBuff, 2, W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {inst, data, addr};

    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    /* 3. submit the assignment request */

    const W25QxxRxOpt mID(&_manufacturerID, 1);
    const W25QxxRxOpt devID(&_deviceID8, 1);

    _rxOpts.push_back(mID);
    _rxOpts.push_back(devID);


    return ret;
}

FlashErrCode W25Qxx::enquireManDevIDDualIOAsync() {
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::READ_MANUFACTURER_DEVICE_ID_DUAL_IO);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, static_cast<uint32_t>(0x00), 3, W25QxxCommModeEnum::DSPI);
    const W25QxxMsgElement dummy(MsgTypeEnum::DUMMY_CYCLE, 4);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, _pRxBuff, 4, W25QxxCommModeEnum::DSPI);

    const std::vector msgs = {inst, data, addr, dummy};

    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    /* 3. submit the assignment request */

    const W25QxxRxOpt mID(&_manufacturerID, 1);
    const W25QxxRxOpt devID(&_deviceID8, 1);

    _rxOpts.push_back(mID);
    _rxOpts.push_back(devID);


    return ret;
}

FlashErrCode W25Qxx::enquireManDevIDQuadIOAsync() {
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::READ_MANUFACTURER_DEVICE_ID_QUAD_IO);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, static_cast<uint32_t>(0x00), 3, W25QxxCommModeEnum::QSPI);
    const W25QxxMsgElement dummy(MsgTypeEnum::DUMMY_CYCLE, 6);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, _pRxBuff, 6, W25QxxCommModeEnum::QSPI);

    const std::vector msgs = {inst, data, addr, dummy};

    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    /* 3. submit the assignment request */

    const W25QxxRxOpt mID(&_manufacturerID, 1);
    const W25QxxRxOpt devID(&_deviceID8, 1);

    _rxOpts.push_back(mID);
    _rxOpts.push_back(devID);


    return ret;
}

FlashErrCode W25Qxx::enquireSfdpRegisterAsync() {
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::READ_SFDP_REGISTER);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, static_cast<uint32_t>(0x00), 3, W25QxxCommModeEnum::STANDARD_SPI);
    const W25QxxMsgElement dummy(MsgTypeEnum::DUMMY_CYCLE, 8);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, _pRxBuff, 8, W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {inst, data, addr, dummy};

    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    /* 3. submit the assignment request */

    const W25QxxRxOpt sfdp(&_sfdpRegister, 8);

    _rxOpts.push_back(sfdp);


    return ret;
}

FlashErrCode W25Qxx::eraseSecurityRegister(const W25QxxRegisterEnum sr) const {
    uint32_t addrByte = 0;
    switch (sr) {
        case W25QxxRegisterEnum::SECURITY_REGISTER_1:
            addrByte |= 1 << 12;
            break;
        case W25QxxRegisterEnum::SECURITY_REGISTER_2:
            addrByte |= 1 << 13;
            break;
        case W25QxxRegisterEnum::SECURITY_REGISTER_3:
            addrByte |= 1 << 12 | 1 << 13;
            break;
        default:
            return FlashErrCode::INVALID;
    }
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::ERASE_SECURITY_REGISTER);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, addrByte, 3, W25QxxCommModeEnum::STANDARD_SPI);


    const std::vector msgs = {inst, addr};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::programSecurityRegister(const W25QxxRegisterEnum sr, void* pData, const uint8_t len) const {
    if (len != 0xFF) {
        return FlashErrCode::INVALID;
    }
    if (pData == nullptr) {
        return FlashErrCode::INVALID;
    }

    uint32_t addrByte = 0;
    switch (sr) {
        case W25QxxRegisterEnum::SECURITY_REGISTER_1:
            addrByte |= 1 << 12;
            break;
        case W25QxxRegisterEnum::SECURITY_REGISTER_2:
            addrByte |= 1 << 13;
            break;
        case W25QxxRegisterEnum::SECURITY_REGISTER_3:
            addrByte |= 1 << 12 | 1 << 13;
            break;
        default:
            return FlashErrCode::INVALID;
    }
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::PROGRAM_SECURITY_REGISTER);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, addrByte, 3, W25QxxCommModeEnum::STANDARD_SPI);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, static_cast<uint8_t*>(pData), 0xFF,
                                W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {inst, addr, data};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::readSecurityRegister(const W25QxxRegisterEnum sr, const uint8_t address, void* pData,
                                       const uint8_t len) const {

    if (pData == nullptr) {
        return FlashErrCode::INVALID;
    }

    uint32_t addrByte = 0;
    switch (sr) {
        case W25QxxRegisterEnum::SECURITY_REGISTER_1:
            addrByte |= 1 << 12;
            break;
        case W25QxxRegisterEnum::SECURITY_REGISTER_2:
            addrByte |= 1 << 13;
            break;
        case W25QxxRegisterEnum::SECURITY_REGISTER_3:
            addrByte |= 1 << 12 | 1 << 13;
            break;
        default:
            return FlashErrCode::INVALID;
    }
    /* 1. collect messages */

    addrByte |= (address & 0xFF);

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::READ_SECURITY_REGISTER);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, addrByte, 3, W25QxxCommModeEnum::STANDARD_SPI);
    const W25QxxMsgElement data(MsgTypeEnum::DATA, static_cast<uint8_t*>(pData), len, W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {inst, addr, data};

    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    return ret;
}

FlashErrCode W25Qxx::lockBlockOrSector(const uint32_t address) const {
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::INDIVIDUAL_BLOCK_SECTOR_LOCK);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, static_cast<uint32_t>(address), 3,
                                W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {inst, addr};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::unlockBlockOrSector(const uint32_t address) const {
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::INDIVIDUAL_BLOCK_SECTOR_UNLOCK);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, static_cast<uint32_t>(address), 3,
                                W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {inst, addr};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::enquireBlockOrSectorLock(const uint32_t address, uint8_t* isLocked) const {
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::READ_BLOCK_SECTOR_LOCK);
    const W25QxxMsgElement addr(MsgTypeEnum::ADDRESS, static_cast<uint32_t>(address), 3,
                                W25QxxCommModeEnum::STANDARD_SPI);

    const std::vector msgs = {inst, addr};

    /* 2. transmit messages */

    const auto ret         = _transport.receive(msgs);

    return ret;
}

FlashErrCode W25Qxx::globalLock() const {
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::GLOBAL_BLOCK_SECTOR_LOCK);
    const std::vector msgs = {inst};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::globalUnlock() const {
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::GLOBAL_BLOCK_SECTOR_UNLOCK);

    const std::vector msgs = {inst};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::resetEnable() const {
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::ENABLE_RESET);

    const std::vector msgs = {inst};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}

FlashErrCode W25Qxx::reset() const {
    /* 1. collect messages */

    const W25QxxMsgElement inst(MsgTypeEnum::INSTRUCTION, W25QxxInst::RESET_DEVICE);

    const std::vector msgs = {inst};

    /* 2. transmit messages */

    const auto ret         = _transport.transmit(msgs);

    return ret;
}


FlashErrCode W25Qxx::asyncRxCallback() {
    uint8_t index = 0;
    for (auto& eachOpt : _rxOpts) {
        // memcpy(eachOpt.getPData(), _pRxBuff + index, eachOpt.getLen());
        memcpy_reverse_order(eachOpt.getPData(), index + _pRxBuff, eachOpt.getLen());
        index += eachOpt.getLen();
    }
    _rxOpts.clear();

    return FlashErrCode::SUCCESS;
}

FlashErrCode W25Qxx::asyncWaitForFlag(const W25QxxStateEnum s) {
    uint8_t target;
    uint8_t mask;
    uint8_t interval = 2;
    W25QxxInst inst;

    switch (s) {
        case W25QxxStateEnum::FREE:
            target = 0x00;
            mask   = 0x01;
            inst   = W25QxxInst::READ_SR1;
            break;
        default:
            return FlashErrCode::INVALID;
    }

    const W25QxxMsgElement readSrInst(MsgTypeEnum::INSTRUCTION, inst);
    const W25QxxMsgElement writeSr(MsgTypeEnum::DATA, _pRxBuff, 1, W25QxxCommModeEnum::STANDARD_SPI);

    std::vector<W25QxxMsgElement> msgs = {readSrInst, writeSr};



    auto ret                           = _transport.autoPolling(msgs, target, mask, interval);

    return ret;
}

static void memcpy_reverse_order(void* dest, const void* src, size_t len) {
    auto d           = static_cast<uint8_t*>(dest);
    const uint8_t* s = static_cast<const uint8_t*>(src) + len - 1;

    while (len--) {
        *d++ = *s--;
    }
}



/* ------- IFlashDevice implement ------------------------------------------------------------------------------------*/

FlashErrCode W25Qxx::read(const uint32_t address, void* buffer, const uint32_t length) {
    return fastReadQuadOutput(address, buffer, static_cast<uint16_t>(length));
}

FlashErrCode W25Qxx::program(const uint32_t address, const void* buffer, const uint32_t length) {

    /* page program 不能跨页，按页拆分并逐页收发完成 */
    uint32_t addr   = address;
    auto p          = static_cast<const uint8_t*>(buffer);
    uint32_t remain = length;

    while (remain > 0) {
        const uint32_t pageOff = addr & (PAGE_SIZE - 1);
        const uint32_t chunk   = (remain > PAGE_SIZE - pageOff) ? (PAGE_SIZE - pageOff) : remain;

        auto ret = writeEnable();
        if (ret != FlashErrCode::SUCCESS) {
            return ret;
        }

        ret = pageProgram(addr, p, static_cast<uint16_t>(chunk));
        if (ret != FlashErrCode::SUCCESS) {
            return ret;
        }

        ret = asyncWaitForFlag(W25QxxStateEnum::FREE);
        if (ret != FlashErrCode::SUCCESS) {
            return ret;
        }

        addr += chunk;
        p += chunk;
        remain -= chunk;
    }

    return FlashErrCode::SUCCESS;
}

FlashErrCode W25Qxx::eraseSector(const uint32_t address) {

    auto ret = writeEnable();
    if (ret != FlashErrCode::SUCCESS) {
        return ret;
    }

    ret = sectorErase(address);
    if (ret != FlashErrCode::SUCCESS) {
        return ret;
    }

    /* 擦除在片内异步进行，稍作等待后再轮询 WIP 位，避免命令刚下发时误判为空闲 */
    vTaskDelay(1);

    return asyncWaitForFlag(W25QxxStateEnum::FREE);
}

FlashErrCode W25Qxx::waitUntilIdle() { return asyncWaitForFlag(W25QxxStateEnum::FREE); }

uint32_t W25Qxx::pageSize() const { return PAGE_SIZE; }

uint32_t W25Qxx::sectorSize() const { return SECTOR_SIZE; }

uint32_t W25Qxx::sectorCount() const { return SECTOR_COUNT; }
