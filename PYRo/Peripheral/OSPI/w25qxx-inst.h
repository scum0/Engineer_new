/**
 *******************************************************************************
 * @file    w25qxx-inst.h
 * @brief   Whe instruction of W25Qxx NOR flash.
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



/*-------- 1. includes and imports -----------------------------------------------------------------------------------*/




/*-------- 2. enum ---------------------------------------------------------------------------------------------------*/

enum class W25QxxInst {
    NONE                                = 0x00,
    WRITE_ENABLE                        = 0x06,

    WRITE_ENABLE_FOR_SR                 = 0x50,

    WRITE_DISABLE                       = 0x04,

    READ_SR1                            = 0x05,
    READ_SR2                            = 0x35,
    READ_SR3                            = 0x15,
    /**
     * @brief The writeable bits include SEC/TB/BP[2:0] in SR1, CMP/LB[3:1]/QE/SRL in SR2, DRV1/DRV0/WPS in SR3
     * @note To write non-volatile SR bits, send WRITE_ENABLE first. To write volatile SR bits, send WRITE_ENABLE_FOR_SR
     * first and make sure the WEL bit is 0.
     * @attention Under no circumstances can SRL and LB[3:1] be changed form 1 to 0.
     * @attention If you write volatile SR bit, it will take effect after NCS is pulled high.
     * @attention You can transmit 2 bytes data after instruction WRTIE_SR1 instead of just one, allowing you to write
     * to both SR1 and SR2.
     */
    WRITE_SR1                           = 0x01,
    WRITE_SR2                           = 0x31,
    WRITE_SR3                           = 0x11,

    /**
     * @brief Read data from flash.
     * @note Transmit a 24-bit address, and as long as your clock signal continues, you will continuoursly receive
     * consecutive data from the flash.
     * @attention Will be ignored while the BUSY bit is 1.
     * @attention Only can be used in Standard SPI mode.
     */
    READ_DATA                           = 0x03,


    /**
     * @brief Read data at the highest possible frequency.
     * @note Transmit a 24-bit address and 8 dummy-cycle. The data when dummy-cycle is "don't care".
     */
    FAST_READ                           = 0x0B,

    /**
     * @brief Read data from two lines
     * @note The data will transmit by IO0 and IO1. Transmit 24-bit address and 8 dummy-cycle.
     * @attention Transmit instruction and address in Standard SPI mode.
     */
    FAST_READ_DUAL_OUTPUT               = 0x3B,

    /**
     * @brief Read data from four lines
     * @note The data will transmit by IO0 to IO3.
     * @attention Enable the QE bit first.
     * @attention Transmit instruction and address in Standard SPI mode.
     */
    FAST_READ_QUAD_OUTPUT               = 0x6B,

    /**
     * @brief Two lines in and two lines out.
     * @note It is similar to the Fast Read Dual Output, but its address should transmit by two lines.
     * @attention No dummy-cycle need.
     */
    FAST_READ_DUAL_IO                   = 0xBB,


    /**
     * @brief Four lines in and four lines out.
     * @attention 4 dummy-cycles need.
     * @attention enable QE bit first.
     */
    FAST_READ_QUAD_IO                   = 0xEB,


    /**
     * @brief Set wrap mode.
     * @note Use with "Fast Read Quad I/O" instructions to perform a circular access within a specific length of memory.
     * @attention Instructions with one line and 6 dummy-cycle and 1 alternate byte with four lines.
     * @note The bit 4 of alternate byte must be 0, bit 5 and bit 6 control the 8/16/32/64 Byte of wrap.
     *
     * Bit6  |Bit5  |Length
     *  0    | 0    | 8  Bytes
     *  0    | 1    | 16 Bytes
     *  1    | 0    | 32 Bytes
     *  1    | 1    | 64 Bytes
     *
     */
    SET_BURST_WITH_WRAP                 = 0x77,


    /**
     * @brief Program a page(256KB)
     * @attention WEL = 1 must
     * @attention If the number of clock more than the end of page, then it will return back to the head.
     * @attention Erase first.
     */
    PAGE_PROGRAM                        = 0x02,

    /**
     * @brief Program with 4 lines
     * @attention If your clock frequency exceeds 5MHz, then the speed bottleneck will be the internal write speed of
     * the flash, not the data transfer rate.
     * @attention Transmit instruction and address with one line, data with four lines.
     */
    QUAD_INPUT_PAGE_PROGRAM             = 0x32,


    /**
     * @brief Erase the sector(4K)
     * @attention WEL = 1 first
     * @attention If the last bit of last byte has been transmitted, the CS must be driven high or it will not be
     * executed.
     */
    SECTOR_ERASE                        = 0x20,


    /**
     * @brief Erase 32KB block
     */
    BLOCK_ERASE_32KB                    = 0x52,

    /**
     * @brief Erase 64KB block
     */
    BLOCK_ERASE_64KB                    = 0xD8,


    /**
     * @brief Erase the whole chip
     */
    CHIP_ERASE                          = 0xC7,
    CHIP_ERASE_BKP                      = 0x60,

    /**
     * @brief Suspend the erase or program option.
     * @attention It is invalid for CHIP ERASE.
     * @attention Can't program when program instruction is suspended. Can't erase when erase instruction is suspended.
     */
    ERASE_PROGRAM_SUSPEND               = 0x75,

    /**
     * @brief Resume the erase or program option.
     * @attention If SUS bit is 0 or BUSY bit is 1, this instruction will be ignored.
     */
    ERASE_PROGRAM_RESUME                = 0x7A,

    /**
     * @brief Power down.
     * @attention Only recognize Release Power-down Instruction after power down.
     */
    POWER_DOWN                          = 0xB9,

    /**
     * @brief Release power down
     */
    RELEASE_POWER_DOWN                  = 0xAB,

    /**
     * @brief Get device ID
     */
    DEVICE_ID                           = 0xAB,

    /**
     * @brief Read ID
     * @attention 24 dummy-cycle.
     */
    READ_MANUFACTURER_DEVICE_ID         = 0x90,


    /**
     * @brief Read ID with two lines
     * @attention 24 bit null address.
     * @attention Transmit instruction with one line.
     */
    READ_MANUFACTURER_DEVICE_ID_DUAL_IO = 0x92,


    /**
     * @brief Read ID with four lines
     * @attention 4 dummy-cycle / 2 null alternate bytes.
     */
    READ_MANUFACTURER_DEVICE_ID_QUAD_IO = 0x94,


    /**
     * @brief Read unique ID
     * @attention 4 null alternate bytes.
     */
    READ_UNIQUE_ID                      = 0x4B,

    /**
     * @brief Read JEDEC IDs
     */
    READ_JEDEC_ID                       = 0x9F,

    /**
     * @brief Read SFDP register
     * @attention 24-bit address(A23 - A8 = 0, A7 - A0 used to define the starting byte address for 256-Byte SFDP
     * register.
     * @attention 8 dummy-cycle
     */
    READ_SFDP_REGISTER                  = 0x5A,

    /**
     * @brief Erase three 256-Byte Security Registers
     * @attention
     * A15-A12
     * 0 0 0 1   Security Register #1
     * 0 0 1 0   Security Register #2
     * 0 0 1 1   Security Register #3
     */
    ERASE_SECURITY_REGISTER             = 0x44,

    /**
     * @brief Program 256 Bytes to Security Register.
     */
    PROGRAM_SECURITY_REGISTER           = 0x42,

    /**
     * @brief Read security register.
     * @attention 8 dummy-cycle
     */
    READ_SECURITY_REGISTER              = 0x48,

    /**
     * @brief Protect the memory array from adverse Erase/Program.
     * @attention WPS bit must is 1.
     * @attention WEL bit must is 1.
     */
    INDIVIDUAL_BLOCK_SECTOR_LOCK        = 0x36,

    /**
     * @brief Unlock memory array
     * @attention WPS = 1
     * @attention WEL = 1
     */
    INDIVIDUAL_BLOCK_SECTOR_UNLOCK      = 0x39,


    /**
     * @brief Read memory array lock.
     */
    READ_BLOCK_SECTOR_LOCK              = 0x3D,

    /**
     * @brief Lock all
     */
    GLOBAL_BLOCK_SECTOR_LOCK            = 0x7E,

    /**
     * @brief Unlock all
     */
    GLOBAL_BLOCK_SECTOR_UNLOCK          = 0x98,

    /**
     * @brief Reset the device
     * @attention  Any other commands other than “Reset (99h)” after the “Enable Reset (66h)” command will disable the
     * “Reset Enable” state.
     */
    ENABLE_RESET                        = 0x66,
    RESET_DEVICE                        = 0x99,
};
/*-------- 3. interface ---------------------------------------------------------------------------------------------*/


/*-------- 4. decorator ----------------------------------------------------------------------------------------------*/


/*-------- 5. factories ----------------------------------------------------------------------------------------------*/