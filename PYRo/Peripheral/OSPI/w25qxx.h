/**
 *******************************************************************************
 * @file    w25qxx.h
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


/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/

#pragma once
#include "w25qxx-transport.h"

#include <cstdint>
#include <vector>


/*-------- includes --------------------------------------------------------------------------------------------------*/





/*-------- class prototypes-------------------------------------------------------------------------------------------*/

enum class W25QxxRegisterEnum {
    STATUS_REGISTER_1,
    STATUS_REGISTER_2,
    STATUS_REGISTER_3,
    SECURITY_REGISTER_1,
    SECURITY_REGISTER_2,
    SECURITY_REGISTER_3,
};

enum class W25QxxWrapLenEnum {
    NONE,
    WRAP_8_BYTE,
    WRAP_16_BYTE,
    WRAP_32_BYTE,
    WRAP_64_BYTE,
};

enum class W25QxxStateEnum {
    NONE,
    FREE,
    BUSY,
    SUSPEND,
};



class W25QxxRxOpt {

  public:
    W25QxxRxOpt(void* pData, const uint16_t len) : _pData(pData), _len(len) {};

    /*********** setter & getter ***********/
    [[nodiscard]] void* getPData() const { return _pData; }
    [[nodiscard]] uint16_t getLen() const { return _len; }

  private:
    void* _pData  = nullptr;
    uint16_t _len = 0;
};




/**
 * @brief The interface of W25Qxx
 */
class W25Qxx : public IFlashDevice {
public:
    explicit W25Qxx(IFlashTransport& transport) : _transport(transport) {};

    /**
     * @brief Send instruction to read the ID of flash.
     * @return FlashErrCode
     */
    FlashErrCode enquireJedecIdAsync();


    /**
     * @brief Send instruction to read the 8 bit ID of flash device.
     * @return FlashErrCode
     */
    FlashErrCode enquireDeviceIdAsync();


    /**
     * @brief Send instruction to read the 64 bit unique ID of flash device.
     * @return FlashErrCode
     */
    FlashErrCode enquireUniqueIdAsync();


    /**
     * @brief ReadRegister
     * @param sr Select the status register want to read.
     * @return Error Code
     */
    FlashErrCode enquireStatusRegisterAsync(W25QxxRegisterEnum sr);


    /**
     * @brief Write Enable
     * @return W25Qxx Error Code
     */
    FlashErrCode writeEnable() const;


    /**
     * @brief Write byte into Registers
     * @param sr Status Register Enum
     * @param byte content
     * @return W25Qxx Error Code
     */
    FlashErrCode writeRegister(const W25QxxRegisterEnum sr, uint8_t byte) const;


    /**
     * @brief Write enable for volatile status register.
     * @return Error Code
     */
    FlashErrCode writeEnableForVolatileStatusRegister() const;


    /**
     * @brief Write disable.
     * @return Error Code
     */
    FlashErrCode writeDisable() const;


    /**
     * @brief read data from flash.
     * @param address 24 bits address
     * @param pData pointer of data
     * @param len length to read
     * @return error code
     */
    FlashErrCode readData(uint32_t address, void* pData, uint16_t len) const;

    FlashErrCode fastReadData(uint32_t address, void* pData, uint16_t len) const;

    FlashErrCode fastReadDualOutput(uint32_t address, void* pData, uint16_t len) const;

    FlashErrCode fastReadQuadOutput(uint32_t address, void* pData, uint16_t len) const;

    FlashErrCode fastReadDualIO(uint32_t address, void* pData, uint16_t len) const;

    FlashErrCode fastReadQuadIO(uint32_t address, void* pData, uint16_t len) const;


    /**
     * @brief set burst with wrap
     * @param len wrap length
     * @return error code
     */
    FlashErrCode setBurstWithWrap(W25QxxWrapLenEnum len) const;



    FlashErrCode pageProgram(uint32_t address, const void *pData, uint16_t len) const;

    FlashErrCode quadInputPageProgram(uint32_t address, const void* pData, uint16_t len) const;



    FlashErrCode sectorErase(uint32_t address) const;

    FlashErrCode blockErase32KB(uint32_t address) const;

    FlashErrCode blockErase64KB(uint32_t address) const;

    FlashErrCode chipErase()const;



    FlashErrCode eraseOrProgramSuspend() const;

    FlashErrCode eraseOrProgramResume() const;




    FlashErrCode powerDown() const;

    FlashErrCode powerOn() const;



    FlashErrCode enquireManDevIDAsync();

    FlashErrCode enquireManDevIDDualIOAsync();

    FlashErrCode enquireManDevIDQuadIOAsync();

    FlashErrCode enquireSfdpRegisterAsync();



    FlashErrCode eraseSecurityRegister(W25QxxRegisterEnum sr) const ;

    FlashErrCode programSecurityRegister(W25QxxRegisterEnum sr, void *pData, uint8_t len) const ;

    FlashErrCode readSecurityRegister(W25QxxRegisterEnum sr, uint8_t address, void *pData, uint8_t len) const;



    FlashErrCode lockBlockOrSector(uint32_t address)const;

    FlashErrCode unlockBlockOrSector(uint32_t address)const;

    FlashErrCode enquireBlockOrSectorLock(uint32_t address, uint8_t *isLocked) const;

    FlashErrCode globalLock()const;

    FlashErrCode globalUnlock()const;


    FlashErrCode resetEnable()const;
    FlashErrCode reset()const;


    FlashErrCode asyncRxCallback();

    FlashErrCode asyncWaitForFlag(W25QxxStateEnum s);


    /************* setter & getter *************/

    [[nodiscard]] W25QxxCommModeEnum getCommMode() const { return _commMode; }
    void setCommMode(const W25QxxCommModeEnum mode) { _commMode = mode; }

    [[nodiscard]] uint8_t getBusyBit() const {return _statusRegister1 & 1;}

    [[nodiscard]] uint8_t getWelBit() const {return _statusRegister1 & (1 << 1);}

    [[nodiscard]] uint8_t getQeBit() const{return _statusRegister2 & (1 << 1);}

    [[nodiscard]] uint8_t getWpsBit() const {return _statusRegister3 & (1 << 2);}

    [[nodiscard]] uint8_t getSusBit() const {return _statusRegister2 & (1 << 7);}

    [[nodiscard]] uint64_t getSFDP() const {return _sfdpRegister;}

    [[nodiscard]] uint8_t getSR1() const {return _statusRegister1;}

    [[nodiscard]] uint8_t getSR2() const {return _statusRegister2;}

    [[nodiscard]] uint8_t getSR3() const {return _statusRegister3;}

    [[nodiscard]] uint8_t getMID() const {return _manufacturerID;}

    [[nodiscard]] uint16_t getDevID16() const{return _deviceID16;}

    [[nodiscard]] uint16_t getDevID8() const{return _deviceID8;}

    [[nodiscard]] uint64_t getUniqueID() const{return _uniqueID;}


    /************* setter & getter *************/


    /************* IFlashDevice *************/

    [[nodiscard]] FlashErrCode read(uint32_t address, void* buffer, uint32_t length) override;

    [[nodiscard]] FlashErrCode program(uint32_t address, const void* buffer, uint32_t length) override;

    [[nodiscard]] FlashErrCode eraseSector(uint32_t address) override;

    [[nodiscard]] FlashErrCode waitUntilIdle() override;

    [[nodiscard]] uint32_t pageSize() const override;

    [[nodiscard]] uint32_t sectorSize() const override;

    [[nodiscard]] uint32_t sectorCount() const override;


    /************* storage geometry（W25Q64：8 MB） *************/

    static constexpr uint32_t PAGE_SIZE    = 256;  // 页编程粒度
    static constexpr uint32_t SECTOR_SIZE  = 4096; // 扇区擦除粒度
    static constexpr uint32_t SECTOR_COUNT = 2048; // 扇区总数


  private:
    uint8_t _manufacturerID      = 0x00;
    uint16_t _deviceID16         = 0x00;
    uint8_t _deviceID8           = 0x00;
    uint64_t _uniqueID           = 0x00;
    uint8_t _pRxBuff[128]        = {};
    uint8_t _statusRegister1     = 0x00;
    uint8_t _statusRegister2     = 0x00;
    uint8_t _statusRegister3     = 0x00;
    uint64_t _sfdpRegister = 0;
    W25QxxCommModeEnum _commMode = W25QxxCommModeEnum::NONE;
    IFlashTransport& _transport;
    std::vector<W25QxxRxOpt> _rxOpts;
};



/*-------- define ----------------------------------------------------------------------------------------------------*/


/*-------- macro -----------------------------------------------------------------------------------------------------*/


/*-------- variables -------------------------------------------------------------------------------------------------*/


/*-------- function prototypes ---------------------------------------------------------------------------------------*/
