/**
 *******************************************************************************
 * @file    flash-intf.h
 * @brief   通用 NOR Flash 设备驱动接口（与具体芯片、总线方式无关）
 *******************************************************************************
 * @note
 *
 *  上层只依赖本接口，不引入 HAL / 具体芯片（W25Qxx）头文件，从而实现上下层解耦。
 *
 *  read / program / eraseSector 为“下发命令”语义：返回 SUCCESS 只代表命令已成功下发，
 *  命令的实际完成由实现内部的中断 / 信号量等待机制保证。
 *
 *******************************************************************************
 */

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/

#pragma once



/*-------- 1. includes and imports -----------------------------------------------------------------------------------*/

#include <cstdint>



/*-------- 2. enum ---------------------------------------------------------------------------------------------------*/

/**
 * @brief Flash 驱动的通用错误码（与具体芯片实现无关）
 */
enum class FlashErrCode {
    NONE = 0,
    BUSY,
    TIMEOUT,
    INVALID,
    SUCCESS,
};



/*-------- 3. interface ---------------------------------------------------------------------------------------------*/

/**
 * @brief 通用 NOR Flash 设备接口
 */
class IFlashDevice {
  public:
    virtual ~IFlashDevice() = default;

    /****************** 数据访问 *******************/

    /**
     * @brief 从指定地址读取数据
     * @param address 起始地址（字节）
     * @param buffer  接收缓冲区
     * @param length  读取长度（字节）
     * @return 错误码
     */
    [[nodiscard]] virtual FlashErrCode read(uint32_t address, void* buffer, uint32_t length) = 0;

    /**
     * @brief 向指定地址编程写入数据（调用前需保证目标区域已擦除；写使能由实现内部完成）
     * @param address 起始地址（字节）
     * @param buffer  待写入数据
     * @param length  写入长度（字节）
     * @return 错误码
     */
    [[nodiscard]] virtual FlashErrCode program(uint32_t address, const void* buffer, uint32_t length) = 0;

    /**
     * @brief 擦除包含指定地址的扇区
     * @param address 扇区内任意地址
     * @return 错误码
     */
    [[nodiscard]] virtual FlashErrCode eraseSector(uint32_t address) = 0;

    /**
     * @brief 等待片上写 / 擦操作完成
     * @return 错误码
     */
    [[nodiscard]] virtual FlashErrCode waitUntilIdle() = 0;

    /****************** 几何信息 *******************/

    /** @brief 最小可编程单元（页）大小，字节 */
    [[nodiscard]] virtual uint32_t pageSize() const = 0;

    /** @brief 最小可擦除单元（扇区）大小，字节 */
    [[nodiscard]] virtual uint32_t sectorSize() const = 0;

    /** @brief 扇区总数 */
    [[nodiscard]] virtual uint32_t sectorCount() const = 0;

    /** @brief 设备总容量，字节 */
    [[nodiscard]] uint32_t capacity() const { return sectorSize() * sectorCount(); }
};
