/*
Example for Illuminance Click

    Date          : Sep 2018.
    Author        : Nenad Filipovic

Test configuration MSP :
    
    MCU              : MSP432
    Dev. Board       : Clicker 2 for MSP432
    ARM Compiler ver : v6.0.0.0

---

Description :

The application is composed of three sections :

- System Initialization - Initializes I2C, GPIO and LOG structures, set INT pin as input.
- Application Initialization - Initialization driver enable's - I2C, power ON and initialize TSL2561 and start write log.
- Application Task - (code snippet) This is a example which demonstrates the use of Illuminance Click board.
     Measured and calculate illuminance ( abmient light ) from sensor,
     results are being sent to the Usart Terminal where you can track their changes.
     All data logs on usb uart for aproximetly every 1 sec when the brightness value changes.

Additional Functions :

- UART
- Conversions
- C_Stdlib

*/

#include "Click_Illuminance_types.h"
#include "Click_Illuminance_config.h"

uint16_t valueCh0;
uint16_t valueCh1;
uint16_t luxValue;
uint16_t luxValueOld = 0;
uint8_t sensitivity = 50;
char txt[15];

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_INT_PIN, _GPIO_INPUT );
    mikrobus_i2cInit( _MIKROBUS1, &_ILLUMINANCE_I2C_CFG[0] );
    mikrobus_logInit( _MIKROBUS2, 9600 );
    Delay_100ms();
}

void applicationInit()
{
    illuminance_i2cDriverInit( (T_ILLUMINANCE_P)&_MIKROBUS1_GPIO, (T_ILLUMINANCE_P)&_MIKROBUS1_I2C, _ILLUMINANCE_TSL2561_I2C_ADDR_HIGH );

    // Power On
    illuminance_writeData( _ILLUMINANCE_TSL2561_COMMAND_BIT | _ILLUMINANCE_TSL2561_REGISTER_CONTROL , _ILLUMINANCE_TSL2561_CONTROL_POWERON );
    Delay_10ms();
    // Set Timing 402 ms
    illuminance_writeData( _ILLUMINANCE_TSL2561_COMMAND_BIT | _ILLUMINANCE_TSL2561_REGISTER_TIMING , _ILLUMINANCE_TSL2561_INTEGRATIONTIME_402MS );
    Delay_10ms();

    mikrobus_logWrite( "--------------------------------", _LOG_LINE );
    mikrobus_logWrite( "          Illuminance", _LOG_LINE );
    mikrobus_logWrite( "--------------------------------", _LOG_LINE );
    Delay_10ms();
}

void applicationTask()
{
    illuminance_getResult( &valueCh0, &valueCh1 );

    luxValue = illuminance_calculateLux( _ILLUMINANCE_TSL2561_GAIN_0X, _ILLUMINANCE_TSL2561_INTEGRATIONTIME_402MS , valueCh0, valueCh1 );
    Delay_1sec();

    if ( ( ( luxValue - luxValueOld ) > sensitivity ) && ( ( luxValueOld - luxValue ) > sensitivity ) )
    {
        mikrobus_logWrite( " Full  Spectrum :", _LOG_TEXT );
        IntToStr( luxValue, txt );
        mikrobus_logWrite( txt, _LOG_TEXT );
        mikrobus_logWrite( " [ lux ]", _LOG_LINE );

        mikrobus_logWrite( " Visible  Value :", _LOG_TEXT );
        IntToStr( ( valueCh0 - valueCh1 ), txt );
        mikrobus_logWrite( txt, _LOG_TEXT );
        mikrobus_logWrite( " [ lux ]", _LOG_LINE );

        mikrobus_logWrite( " Infrared Value :", _LOG_TEXT );
        IntToStr( valueCh1 , txt );
        mikrobus_logWrite( txt, _LOG_TEXT );
        mikrobus_logWrite( " [ lux ]", _LOG_LINE );

        mikrobus_logWrite( "--------------------------------", _LOG_LINE );

        luxValueOld = luxValue;
    }

}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
            applicationTask();
    }
}