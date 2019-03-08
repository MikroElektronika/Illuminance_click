![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# Illuminance Click

---

- **CIC Prefix**  : ILLUMINANCE
- **Author**      : Nenad Filipovic
- **Verison**     : 1.0.0
- **Date**        : Sep 2018.

---

### Software Support

We provide a library for the Illuminance Click on our [LibStock](https://libstock.mikroe.com/projects/view/1135/illuminance-click-example) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

The library covers all the necessary functions to control Illuminance Click.
Library performs the communication with the device via I2C driver by writting to registers and by reading from registers.
Library reads measurements for illuminance ( abmient light ) and calculate lux value from TSL2561 sensor.

Key functions :

- ``` void illuminance_writeData( uint8_t address, uint8_t writeCommand ) ``` - Generic write data function
- ``` void illuminance_getResult( uint16_t *valueCh0, uint16_t *valueCh1 ) ``` - Function get data Visible & Infrared value
- ``` uint16_t illuminance_calculateLux( uint16_t valueGain, uint16_t valueInt, uint16_t ch0, uint16_t ch1 ) ``` - Calculate lux value

**Examples Description**

The application is composed of three sections :

- System Initialization - Initializes I2C, GPIO and LOG structures, set INT pin as input.
- Application Initialization - Initialization driver enable's - I2C, power ON and initialize TSL2561 and start write log.
- Application Task - (code snippet) This is a example which demonstrates the use of Illuminance Click board.
     Measured and calculate illuminance ( abmient light ) from sensor,
     results are being sent to the Usart Terminal where you can track their changes.
     All data logs on usb uart for aproximetly every 1 sec when the brightness value changes.


```.c

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

```



The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/1135/illuminance-click-example) page.

Other mikroE Libraries used in the example:

- I2C
- UART
- Conversions

**Additional notes and informations**

Depending on the development board you are using, you may need 
[USB UART click](http://shop.mikroe.com/usb-uart-click), 
[USB UART 2 Click](http://shop.mikroe.com/usb-uart-2-click) or 
[RS232 Click](http://shop.mikroe.com/rs232-click) to connect to your PC, for 
development systems with no UART to USB interface available on the board. The 
terminal available in all Mikroelektronika 
[compilers](http://shop.mikroe.com/compilers), or any other terminal application 
of your choice, can be used to read the message.

---
---
