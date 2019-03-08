/*
    __illuminance_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__illuminance_driver.h"
#include "__illuminance_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __ILLUMINANCE_DRV_I2C__
static uint8_t _slaveAddress;
#endif


/* I2C address */
const uint8_t _ILLUMINANCE_TSL2561_I2C_ADDR_LOW                          = 0x29;
const uint8_t _ILLUMINANCE_TSL2561_I2C_ADDR_FLOAT                        = 0x39;
const uint8_t _ILLUMINANCE_TSL2561_I2C_ADDR_HIGH                         = 0x49;

/* Register address */
const uint8_t _ILLUMINANCE_TSL2561_REGISTER_CONTROL                      = 0x00;
const uint8_t _ILLUMINANCE_TSL2561_REGISTER_TIMING                       = 0x01;
const uint8_t _ILLUMINANCE_TSL2561_REGISTER_THRESHHOLDL_LOW              = 0x02;
const uint8_t _ILLUMINANCE_TSL2561_REGISTER_THRESHHOLDL_HIGH             = 0x03;
const uint8_t _ILLUMINANCE_TSL2561_REGISTER_THRESHHOLDH_LOW              = 0x04;
const uint8_t _ILLUMINANCE_TSL2561_REGISTER_THRESHHOLDH_HIGH             = 0x05;
const uint8_t _ILLUMINANCE_TSL2561_REGISTER_INTERRUPT                    = 0x06;
const uint8_t _ILLUMINANCE_TSL2561_REGISTER_CRC                          = 0x08;
const uint8_t _ILLUMINANCE_TSL2561_REGISTER_ID                           = 0x0A;
const uint8_t _ILLUMINANCE_TSL2561_REGISTER_CHAN0_LOW                    = 0x0C;
const uint8_t _ILLUMINANCE_TSL2561_REGISTER_CHAN0_HIGH                   = 0x0D;
const uint8_t _ILLUMINANCE_TSL2561_REGISTER_CHAN1_LOW                    = 0x0E;
const uint8_t _ILLUMINANCE_TSL2561_REGISTER_CHAN1_HIGH                   = 0x0F;

/* Config bits */
// 13.7ms
const uint8_t _ILLUMINANCE_TSL2561_INTEGRATIONTIME_13MS                  = 0x00;
// 101ms
const uint8_t _ILLUMINANCE_TSL2561_INTEGRATIONTIME_101MS                 = 0x01;
// 402ms
const uint8_t _ILLUMINANCE_TSL2561_INTEGRATIONTIME_402MS                 = 0x02;
// No gain
const uint8_t _ILLUMINANCE_TSL2561_GAIN_0X                               = 0x00;
// 16x gain
const uint8_t _ILLUMINANCE_TSL2561_GAIN_16X                              = 0x10;
// Command bit must be 1
const uint8_t _ILLUMINANCE_TSL2561_COMMAND_BIT                           = 0x80;
// Clears any pending interrupt - write 1 to clear
const uint8_t _ILLUMINANCE_TSL2561_CLEAR_BIT                             = 0x40;
// 1 = read/write word - rather than byte
const uint8_t _ILLUMINANCE_TSL2561_WORD_BIT                              = 0x20;
// 1 = using block read/write
const uint8_t _ILLUMINANCE_TSL2561_BLOCK_BIT                             = 0x10;
// Power ON
const uint8_t _ILLUMINANCE_TSL2561_CONTROL_POWERON                       = 0x03;
// Power OFF
const uint8_t _ILLUMINANCE_TSL2561_CONTROL_POWEROFF                      = 0x00;
// Scale by 2^14
const uint8_t _ILLUMINANCE_TSL2561_LUX_SCALE                               = 14;
// Scale ratio by 2^9
const uint8_t _ILLUMINANCE_TSL2561_RATIO_SCALE                             = 9;
// Scale channel values by 2^10
const uint8_t _ILLUMINANCE_TSL2561_CH_SCALE                                = 10;
// 322/11 * 2^TSL2561_LUX_CHSCALE
const uint16_t _ILLUMINANCE_TSL2561_CHSCALE_TINT0                      = 0x7517;
// 322/81 * 2^TSL2561_LUX_CHSCALE
const uint16_t _ILLUMINANCE_TSL2561_CHSCALE_TINT1                      = 0x0FE7;

/* T, FN and CL package coefficients */
// 0.125 * 2^RATIO_SCALE
const uint16_t _ILLUMINANCE_TSL2561_K1T                                = 0x0040;
// 0.0304 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_B1T                                = 0x01f2;
// 0.0272 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_M1T                                = 0x01be;
// 0.250 * 2^RATIO_SCALE
const uint16_t _ILLUMINANCE_TSL2561_K2T                                = 0x0080;
// 0.0325 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_B2T                                = 0x0214;
// 0.0440 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_M2T                                = 0x02d1;
// 0.375 * 2^RATIO_SCALE
const uint16_t _ILLUMINANCE_TSL2561_K3T                                = 0x00c0;
// 0.0351 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_B3T                                = 0x023f;
// 0.0544 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_M3T                                = 0x037b;
// 0.50 * 2^RATIO_SCALE
const uint16_t _ILLUMINANCE_TSL2561_K4T                                = 0x0100;
// 0.0381 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_B4T                                = 0x0270;
// 0.0624 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_M4T                                = 0x03fe;
// 0.61 * 2^RATIO_SCALE
const uint16_t _ILLUMINANCE_TSL2561_K5T                                = 0x0138;
// 0.0224 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_B5T                                = 0x016f;
// 0.0310 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_M5T                                = 0x01fc;
// 0.80 * 2^RATIO_SCALE
const uint16_t _ILLUMINANCE_TSL2561_K6T                                = 0x019a;
// 0.0128 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_B6T                                = 0x00d2;
// 0.0153 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_M6T                                = 0x00fb;
// 1.3 * 2^RATIO_SCALE
const uint16_t _ILLUMINANCE_TSL2561_K7T                                = 0x029a;
// 0.00146 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_B7T                                = 0x0018;
// 0.00112 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_M7T                                = 0x0012;
// 1.3 * 2^RATIO_SCALE
const uint16_t _ILLUMINANCE_TSL2561_K8T                                = 0x029a;
// 0.000 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_B8T                                = 0x0000;
// 0.000 * 2^LUX_SCALE
const uint16_t _ILLUMINANCE_TSL2561_M8T                                = 0x0000;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */



/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */



/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __ILLUMINANCE_DRV_SPI__

void illuminance_spiDriverInit(T_ILLUMINANCE_P gpioObj, T_ILLUMINANCE_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __ILLUMINANCE_DRV_I2C__

void illuminance_i2cDriverInit(T_ILLUMINANCE_P gpioObj, T_ILLUMINANCE_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __ILLUMINANCE_DRV_UART__

void illuminance_uartDriverInit(T_ILLUMINANCE_P gpioObj, T_ILLUMINANCE_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif



/* ----------------------------------------------------------- IMPLEMENTATION */

/* Generic write data function to TSL2561 */
void illuminance_writeData( uint8_t address, uint8_t writeCommand )
{
    uint8_t buffer[2];
    buffer[0]= address;
    buffer[1]= writeCommand;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, buffer, 2, END_MODE_STOP );
}

/* Generic read data function from TSL2561 */
uint8_t illuminance_readData( uint8_t address )
{
    uint8_t writeReg[1];
    uint8_t readReg[1];

    writeReg[0] = address;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, writeReg, 1, END_MODE_RESTART );
    hal_i2cRead( _slaveAddress, readReg, 1, END_MODE_STOP );

    return readReg[0];
}

/* Function get data Visible & Infrared value */
void illuminance_getResult( uint16_t *valueCh0, uint16_t *valueCh1 )
{
    uint16_t result;
    uint8_t buffer0[2];
    uint8_t buffer1[2];

    buffer0[0] = illuminance_readData( _ILLUMINANCE_TSL2561_COMMAND_BIT | _ILLUMINANCE_TSL2561_REGISTER_CHAN0_LOW );
    buffer0[1] = illuminance_readData( _ILLUMINANCE_TSL2561_COMMAND_BIT | _ILLUMINANCE_TSL2561_REGISTER_CHAN0_HIGH );
    
    buffer1[0] = illuminance_readData( _ILLUMINANCE_TSL2561_COMMAND_BIT | _ILLUMINANCE_TSL2561_REGISTER_CHAN1_LOW );
    buffer1[1] = illuminance_readData( _ILLUMINANCE_TSL2561_COMMAND_BIT | _ILLUMINANCE_TSL2561_REGISTER_CHAN1_HIGH );

    result = 0x00;
    result = buffer0[1];
    result <<= 8;
    result |= buffer0[0];
    
    *valueCh0 = result;

    result = 0x00;
    result = buffer1[1];
    result <<= 8;
    result |= buffer1[0];
    
    *valueCh1 = result;
}

/* Calculate lux value from the ADC readings */
uint16_t illuminance_calculateLux( uint16_t valueGain, uint16_t valueInt, uint16_t ch0, uint16_t ch1 )
{
    uint32_t chScale;
    uint32_t channel0;
    uint32_t channel1;
    uint32_t temp;
    uint32_t lux;
    uint32_t ratio;
    uint32_t ratio1;
    uint16_t b;
    uint16_t m;
  
    ratio1 = 0;

    switch ( valueInt )
    {
        case 0: // 13.7 msec
            chScale = _ILLUMINANCE_TSL2561_CHSCALE_TINT0;
            break;
        case 1: // 101 msec
            chScale = _ILLUMINANCE_TSL2561_CHSCALE_TINT1;
            break;
        default: // assume no scaling
            chScale = ( 1 << _ILLUMINANCE_TSL2561_CH_SCALE );
            break;
    }

    // scale if gain is NOT 16X
    if ( !valueGain )
    {
        // scale 1X to 16X
        chScale = chScale << 4;
    }
    
    // scale the channel values
    channel0 = ( ch0 * chScale ) >> _ILLUMINANCE_TSL2561_CH_SCALE;
    channel1 = ( ch1 * chScale ) >> _ILLUMINANCE_TSL2561_CH_SCALE;

    // find the ratio of the channel values (Channel1/Channel0)
    // protect against divide by zero
    if ( channel0 != 0 )
    {
        ratio1 = (channel1 << (_ILLUMINANCE_TSL2561_RATIO_SCALE +1 )) / channel0;
    }
    
    // round the ratio value
    ratio = (ratio1 + 1) >> 1;

    if ( ( ratio >= 0 ) && ( ratio <= _ILLUMINANCE_TSL2561_K1T ) )
    {
         b = _ILLUMINANCE_TSL2561_B1T;
         m = _ILLUMINANCE_TSL2561_M1T;
    }
    else if ( ratio <= _ILLUMINANCE_TSL2561_K2T )
    {
        b = _ILLUMINANCE_TSL2561_B2T;
        m = _ILLUMINANCE_TSL2561_M2T;
    }
    else if ( ratio <= _ILLUMINANCE_TSL2561_K3T )
    {
        b = _ILLUMINANCE_TSL2561_B3T;
        m = _ILLUMINANCE_TSL2561_M3T;
    }
    else if ( ratio <= _ILLUMINANCE_TSL2561_K4T )
    {
        b = _ILLUMINANCE_TSL2561_B4T;
        m = _ILLUMINANCE_TSL2561_M4T;
    }
    else if ( ratio <= _ILLUMINANCE_TSL2561_K5T )
    {
        b = _ILLUMINANCE_TSL2561_B5T;
        m = _ILLUMINANCE_TSL2561_M5T;
    }
    else if ( ratio <= _ILLUMINANCE_TSL2561_K6T )
    {
        b = _ILLUMINANCE_TSL2561_B6T;
        m = _ILLUMINANCE_TSL2561_M6T;
    }
    else if ( ratio <= _ILLUMINANCE_TSL2561_K7T )
    {
        b = _ILLUMINANCE_TSL2561_B7T;
        m = _ILLUMINANCE_TSL2561_M7T;
    }
    else if ( ratio > _ILLUMINANCE_TSL2561_K8T )
    {
        b = _ILLUMINANCE_TSL2561_B8T;
        m = _ILLUMINANCE_TSL2561_M8T;
    }

    temp = ( ( channel0 * b ) - ( channel1 * m ) );

    // do not allow negative lux value
    if ( temp < 0 )
    {
        temp = 0;
    }
    
    // round lsb (2^(LUX_SCALE-1))
    temp = temp + ( 1 << ( _ILLUMINANCE_TSL2561_LUX_SCALE - 1 ) );

    // strip off fractional portion
    lux = temp >> _ILLUMINANCE_TSL2561_LUX_SCALE;
    
    return lux ;
}

/* Functions for reads interrupt pin */
uint8_t illuminance_getInterrupt()
{
    return hal_gpio_intGet();
}




/* -------------------------------------------------------------------------- */
/*
  __illuminance_driver.c

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