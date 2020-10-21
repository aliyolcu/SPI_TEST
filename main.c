#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/watchdog.h"
#include "driverlib/adc.h"
#include "driverlib/can.h"
#include "driverlib/eeprom.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "inc/hw_hibernate.h"
#include "inc/hw_gpio.h"
#include "driverlib/hibernate.h"

#define NUM_SSI_DATA    3

//defýntýon of our mcp
#define    IODIRA    (0x00)      // MCP23x17 I/O Direction Register
#define    IODIRB    (0x01)      // 1 = Input (default), 0 = Output

#define    IPOLA     (0x02)      // MCP23x17 Input Polarity Register
#define    IPOLB     (0x03)      // 0 = Normal (default)(low reads as 0), 1 = Inverted (low reads as 1)

#define    GPINTENA  (0x04)      // MCP23x17 Interrupt on Change Pin Assignements
#define    GPINTENB  (0x05)      // 0 = No Interrupt on Change (default), 1 = Interrupt on Change

#define    DEFVALA   (0x06)      // MCP23x17 Default Compare Register for Interrupt on Change
#define    DEFVALB   (0x07)      // Opposite of what is here will trigger an interrupt (default = 0)

#define    INTCONA   (0x08)      // MCP23x17 Interrupt on Change Control Register
#define    INTCONB   (0x09)      // 1 = pin is compared to DEFVAL, 0 = pin is compared to previous state (default)

#define    IOCON     (0x0A)      // MCP23x17 Configuration Register
//                   (0x0B)      //     Also Configuration Register

#define    GPPUA     (0x0C)      // MCP23x17 Weak Pull-Up Resistor Register
#define    GPPUB     (0x0D)      // INPUT ONLY: 0 = No Internal 100k Pull-Up (default) 1 = Internal 100k Pull-Up

#define    INTFA     (0x0E)      // MCP23x17 Interrupt Flag Register
#define    INTFB     (0x0F)      // READ ONLY: 1 = This Pin Triggered the Interrupt

#define    INTCAPA   (0x10)      // MCP23x17 Interrupt Captured Value for Port Register
#define    INTCAPB   (0x11)      // READ ONLY: State of the Pin at the Time the Interrupt Occurred

#define    GPIOA     (0x12)      // MCP23x17 GPIO Port Register
#define    GPIOB     (0x13)      // Value on the Port - Writing Sets Bits in the Output Latch

#define    OLATA     (0x14)      // MCP23x17 Output Latch Register
#define    OLATB     (0x15)      // 1 = Latch High, 0 = Latch Low (default) Reading Returns Latch State, Not Port Value!


#define    OPCODEW       (0b01000000)  // Opcode for MCP23S17 with LSB (bit0) set to write (0), address OR'd in later, bits 1-3
#define    OPCODER       (0b01000001)  // Opcode for MCP23S17 with LSB (bit0) set to read (1), address OR'd in later, bits 1-3
#define    ADDR_ENABLE   (0b00001000)  // Configuration register for MCP23S17, the only thing we change is enabling hardware addressing

#define    address        0

#define    ADDR_OPCODEW   (OPCODEW | (address << 1)) // important note :here ý put ýn  _address place 0x00
#define    ADDR_reg       reg
#define    ADDR_word      word



uint32_t pui32DataTx[NUM_SSI_DATA] = {0x40,0x00,0x00};
uint32_t ui32Index;

//mcp varýable
uint32_t _modeCache   = 0xFFFF;                // Default I/O mode is all input, 0xFFFF
uint32_t _outputCache = 0x0000;                // Default output state is all off, 0x0000
uint32_t _pullupCache = 0x0000;                // Default pull-up state is all off, 0x0000
uint32_t _invertCache = 0x0000;                // Default input inversion state is not inverted, 0x0000

//mcp functýon
void wordWrite(uint8_t reg, unsigned int word) {  // Accept the start register and word

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, !GPIO_PIN_2);

    SSIDataPut(SSI2_BASE, ADDR_OPCODEW);
    SSIDataPut(SSI2_BASE, reg);
    SSIDataPut(SSI2_BASE,(uint8_t) (word));
    SSIDataPut(SSI2_BASE,(uint8_t) (word >> 8));

    while(SSIBusy(SSI2_BASE))
    {

    }

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

void byteWrite(uint8_t reg, uint8_t value) {

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, !GPIO_PIN_2);

    SSIDataPut(SSI2_BASE, ADDR_OPCODEW);
    SSIDataPut(SSI2_BASE, reg);
    SSIDataPut(SSI2_BASE,value);

    while(SSIBusy(SSI2_BASE))
    {

    }

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

// MODE SETTING FUNCTIONS - BY PIN AND BY WORD
void pinMode(unsigned int mode) {     // Accept the word…
  wordWrite(IODIRA, mode);                 // Call the the generic word writer with start register and the mode cache
  _modeCache = mode;
}

// WRITE FUNCTIONS - BY WORD AND BY PIN

void digitalWrite(uint8_t pin, uint8_t value) {
  if (pin < 1 | pin > 16) return;
  if (pin < 1 | pin > 16) return;
  if (value) {
    _outputCache |= 1 << (pin - 1);
  } else {
    _outputCache &= ~(1 << (pin - 1));
  }
  wordWrite(GPIOA, _outputCache);
}
/* mouaiad */
void delayMS ( int ms ) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
}

void WriteSPI( void ){

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, !GPIO_PIN_2);

    for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
    {
        SSIDataPut(SSI2_BASE, pui32DataTx[ui32Index]);
    }

    while(SSIBusy(SSI2_BASE))
    {

    }
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

bool isState = false;

int main(void)
{
    SysCtlClockSet(
        SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ
                  | SYSCTL_OSC_MAIN);

    /* SPI */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //CS
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_STRENGTH_12MA,
                     GPIO_PIN_TYPE_STD);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 , !GPIO_PIN_2);


    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    //GPIOPinConfigure(GPIO_PH5_SSI2FSS);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PH7 - SSI2Tx
    //      PH6 - SSI2Rx
    //      PH5 - SSI2Fss
    //      PH4 - SSI2CLK

    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_7 | GPIO_PIN_4);

    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_2,
                       SSI_MODE_MASTER, 1000000, 8);

    SSIEnable(SSI2_BASE);

    /* SPI */

//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, !GPIO_PIN_2);
//
//    for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
//    {
//        SSIDataPut(SSI2_BASE, pui32DataTx[ui32Index]);
//    }
//
//    while(SSIBusy(SSI2_BASE))
//    {
//
//    }
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

    byteWrite(IOCON, ADDR_ENABLE);
    pinMode(0x0000);

    while(1){

//        if( isState ){
//            pui32DataTx[0] = 0x40;
//            pui32DataTx[1] = 0x09;
//            pui32DataTx[2] = 0xFF;
//        }else{
//            pui32DataTx[0] = 0x40;
//            pui32DataTx[1] = 0x09;
//            pui32DataTx[2] = 0x00;
//        }
//
//        WriteSPI();
//        isState = !isState;

        digitalWrite(1,1);
        digitalWrite(2,1);
        delayMS(100);
        digitalWrite(1,0);
        digitalWrite(2,0);
        delayMS(100);
    }
}
