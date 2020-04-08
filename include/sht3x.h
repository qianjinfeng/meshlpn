/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef SHT3X_H__
#define SHT3X_H__


#include "nrf_twi_mngr.h"

#ifdef __cplusplus
extern "C" {
#endif


///* all measurement commands return T (CRC) RH (CRC) */
//#if USE_SENSIRION_CLOCK_STRETCHING
//static const u8 CMD_MEASURE_HPM[]     = { 0x2C, 0x06 };
//static const u8 CMD_MEASURE_LPM[]     = { 0x2C, 0x10 };
//#else
//static const u8 CMD_MEASURE_HPM[]     = { 0x24, 0x00 };
//static const u8 CMD_MEASURE_LPM[]     = { 0x24, 0x16 };
//#endif /* USE_SENSIRION_CLOCK_STRETCHING */
//static const u8 CMD_READ_STATUS_REG[] = { 0xF3, 0x2D };
//static const u8 COMMAND_SIZE = sizeof(CMD_MEASURE_HPM);
//#ifdef SHT_ADDRESS
//static const u8 SHT3X_ADDRESS = SHT_ADDRESS;
//#else
//static const u8 SHT3X_ADDRESS = 0x44;
//#endif


// 0x90 is the SHT3X's address in the mbed Application Shield, it contains
// R/W bit and "nrf_drv_twi" (and consequently "nrf_twi_mngr") requires slave
// address without this bit, hence shifting.
#define SHT3X_ADDR          0x44 //(0x90U >> 1)

#define SHT3X_REG_TEMP      0x00
#define SHT3X_REG_CONF      0x01

// [use "/ 32" instead of ">> 5", as the result of right-shifting of a signed
//  type value is implementation-defined]
#define SHT3X_GET_TEMPERATURE_VALUE(temp_hi, temp_lo) \
    ((((int16_t)temp_hi << 8) | temp_lo) / 32)


extern uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND sht3x_conf_reg_addr;
extern uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND sht3x_temp_reg_addr;


#define SHT3X_READ(p_reg_addr, p_buffer, byte_cnt) \
    NRF_TWI_MNGR_WRITE(SHT3X_ADDR, p_reg_addr, 1,        NRF_TWI_MNGR_NO_STOP), \
    NRF_TWI_MNGR_READ (SHT3X_ADDR, p_buffer,   byte_cnt, 0)

#define SHT3X_READ_TEMP(p_buffer) \
    SHT3X_READ(&sht3x_temp_reg_addr, p_buffer, 2)

#define SHT3X_INIT_TRANSFER_COUNT 1

extern nrf_twi_mngr_transfer_t const sht3x_init_transfers[SHT3X_INIT_TRANSFER_COUNT];

#ifdef __cplusplus
}
#endif

#endif // SHT3X_H__
