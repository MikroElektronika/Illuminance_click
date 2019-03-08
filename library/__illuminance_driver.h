/*
    __illuminance_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __illuminance_driver.h
@brief    Illuminance Driver
@mainpage Illuminance Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   ILLUMINANCE
@brief      Illuminance Click Driver
@{

| Global Library Prefix | **ILLUMINANCE** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Sep 2018.**      |
| Developer             | **Nenad Filipovic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _ILLUMINANCE_H_
#define _ILLUMINANCE_H_

/** 
 * @macro T_ILLUMINANCE_P
 * @brief Driver Abstract type 
 */
#define T_ILLUMINANCE_P    const uint8_t*

/** @defgroup ILLUMINANCE_COMPILE Compilation Config */              /** @{ */

//  #define   __ILLUMINANCE_DRV_SPI__                            /**<     @macro __ILLUMINANCE_DRV_SPI__  @brief SPI driver selector */
   #define   __ILLUMINANCE_DRV_I2C__                            /**<     @macro __ILLUMINANCE_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __ILLUMINANCE_DRV_UART__                           /**<     @macro __ILLUMINANCE_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup ILLUMINANCE_VAR Variables */                           /** @{ */

/* I2C address */
extern const uint8_t _ILLUMINANCE_TSL2561_I2C_ADDR_LOW;
extern const uint8_t _ILLUMINANCE_TSL2561_I2C_ADDR_FLOAT;
extern const uint8_t _ILLUMINANCE_TSL2561_I2C_ADDR_HIGH;

/* Register address */
extern const uint8_t _ILLUMINANCE_TSL2561_REGISTER_CONTROL;
extern const uint8_t _ILLUMINANCE_TSL2561_REGISTER_TIMING;
extern const uint8_t _ILLUMINANCE_TSL2561_REGISTER_THRESHHOLDL_LOW;
extern const uint8_t _ILLUMINANCE_TSL2561_REGISTER_THRESHHOLDL_HIGH;
extern const uint8_t _ILLUMINANCE_TSL2561_REGISTER_THRESHHOLDH_LOW;
extern const uint8_t _ILLUMINANCE_TSL2561_REGISTER_THRESHHOLDH_HIGH;
extern const uint8_t _ILLUMINANCE_TSL2561_REGISTER_INTERRUPT;
extern const uint8_t _ILLUMINANCE_TSL2561_REGISTER_CRC;
extern const uint8_t _ILLUMINANCE_TSL2561_REGISTER_ID;
extern const uint8_t _ILLUMINANCE_TSL2561_REGISTER_CHAN0_LOW;
extern const uint8_t _ILLUMINANCE_TSL2561_REGISTER_CHAN0_HIGH;
extern const uint8_t _ILLUMINANCE_TSL2561_REGISTER_CHAN1_LOW;
extern const uint8_t _ILLUMINANCE_TSL2561_REGISTER_CHAN1_HIGH;

/* Config bits */
extern const uint8_t _ILLUMINANCE_TSL2561_INTEGRATIONTIME_13MS;
extern const uint8_t _ILLUMINANCE_TSL2561_INTEGRATIONTIME_101MS;
extern const uint8_t _ILLUMINANCE_TSL2561_INTEGRATIONTIME_402MS;
extern const uint8_t _ILLUMINANCE_TSL2561_GAIN_0X;
extern const uint8_t _ILLUMINANCE_TSL2561_GAIN_16X;
extern const uint8_t _ILLUMINANCE_TSL2561_COMMAND_BIT;
extern const uint8_t _ILLUMINANCE_TSL2561_CLEAR_BIT;
extern const uint8_t _ILLUMINANCE_TSL2561_WORD_BIT;
extern const uint8_t _ILLUMINANCE_TSL2561_BLOCK_BIT;
extern const uint8_t _ILLUMINANCE_TSL2561_CONTROL_POWERON;
extern const uint8_t _ILLUMINANCE_TSL2561_CONTROL_POWEROFF;
extern const uint8_t _ILLUMINANCE_TSL2561_LUX_SCALE;
extern const uint8_t _ILLUMINANCE_TSL2561_RATIO_SCALE;
extern const uint8_t _ILLUMINANCE_TSL2561_CH_SCALE;
extern const uint16_t _ILLUMINANCE_TSL2561_CHSCALE_TINT0;
extern const uint16_t _ILLUMINANCE_TSL2561_CHSCALE_TINT1;

                                                                       /** @} */
/** @defgroup ILLUMINANCE_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup ILLUMINANCE_INIT Driver Initialization */              /** @{ */

#ifdef   __ILLUMINANCE_DRV_SPI__
void illuminance_spiDriverInit(T_ILLUMINANCE_P gpioObj, T_ILLUMINANCE_P spiObj);
#endif
#ifdef   __ILLUMINANCE_DRV_I2C__
void illuminance_i2cDriverInit(T_ILLUMINANCE_P gpioObj, T_ILLUMINANCE_P i2cObj, uint8_t slave);
#endif
#ifdef   __ILLUMINANCE_DRV_UART__
void illuminance_uartDriverInit(T_ILLUMINANCE_P gpioObj, T_ILLUMINANCE_P uartObj);
#endif


/** @defgroup ILLUMINANCE_FUNC Driver Functions */                   /** @{ */

/**
 * @brief Generic write data function
 *
 * @param[in] address         Register address
 *
 * @param[in] writeCommand    Command to write
 *
 * Function write byte of data to TSL2561.
 */
void illuminance_writeData( uint8_t address, uint8_t writeCommand );

/**
 * @brief Generic read data function
 *
 * @param[in] address         Register address
 *
 * @return    8-bit data from addressed register from TSL2561
 *
 * Function read byte of data from register address of TSL2561.
 */
uint8_t illuminance_readData( uint8_t address );

/**
 * @brief Function get data
 *
 * @param[out] valueCh0             pointer to read Visible & Infrared data
 *
 * @param[out] valueCh1             pointer to read Infrared data
 *
 * Function get data Visible & Infrared value.
 *
 */
void illuminance_getResult( uint16_t *valueCh0, uint16_t *valueCh1 );

/**
 * @brief Calculate lux value
 *
 * @param[in] valueGain      0x00 - No gain, 0x01 - 16x gain
 *
 * @param[in] valueInt       0x00 - 13.7ms, 0x01 - 101ms, 0x02 - 402ms
 *
 * @param[in] channel0       Visible & Infrared data
 *
 * @param[in] channel1       Infrared data
 *
 * @return                   16-bit resulting lux calculation
 *
 * Calculate lux value from the TSL2561 sensor.
 */
uint16_t illuminance_calculateLux( uint16_t valueGain, uint16_t valueInt, uint16_t channel0, uint16_t channel1 );

/**
 * @brief Get interrupt status
 *
 * @return          status reading from pins
 *
 * Functions for reads interrupt pin.
 */
uint8_t illuminance_getInterrupt();



                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Illuminance_STM.c
    @example Click_Illuminance_TIVA.c
    @example Click_Illuminance_CEC.c
    @example Click_Illuminance_KINETIS.c
    @example Click_Illuminance_MSP.c
    @example Click_Illuminance_PIC.c
    @example Click_Illuminance_PIC32.c
    @example Click_Illuminance_DSPIC.c
    @example Click_Illuminance_AVR.c
    @example Click_Illuminance_FT90x.c
    @example Click_Illuminance_STM.mbas
    @example Click_Illuminance_TIVA.mbas
    @example Click_Illuminance_CEC.mbas
    @example Click_Illuminance_KINETIS.mbas
    @example Click_Illuminance_MSP.mbas
    @example Click_Illuminance_PIC.mbas
    @example Click_Illuminance_PIC32.mbas
    @example Click_Illuminance_DSPIC.mbas
    @example Click_Illuminance_AVR.mbas
    @example Click_Illuminance_FT90x.mbas
    @example Click_Illuminance_STM.mpas
    @example Click_Illuminance_TIVA.mpas
    @example Click_Illuminance_CEC.mpas
    @example Click_Illuminance_KINETIS.mpas
    @example Click_Illuminance_MSP.mpas
    @example Click_Illuminance_PIC.mpas
    @example Click_Illuminance_PIC32.mpas
    @example Click_Illuminance_DSPIC.mpas
    @example Click_Illuminance_AVR.mpas
    @example Click_Illuminance_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __illuminance_driver.h

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */