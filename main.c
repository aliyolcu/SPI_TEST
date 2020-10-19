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

uint32_t pui32DataTx[NUM_SSI_DATA] = {0x40,0x00,0x00};
uint32_t ui32Index;

void delayMS ( int ms ) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
}

void WriteSPI( void ){

    GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_5, GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_5, !GPIO_PIN_5);

    for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
    {
        SSIDataPut(SSI2_BASE, pui32DataTx[ui32Index]);
    }

    while(SSIBusy(SSI2_BASE))
    {

    }
    GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_5, GPIO_PIN_5);
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

	while(1){

	    if( isState ){
	        pui32DataTx[0] = 0x40;
	        pui32DataTx[1] = 0x09;
	        pui32DataTx[2] = 0xFF;
	    }else{
	        pui32DataTx[0] = 0x40;
	        pui32DataTx[1] = 0x09;
	        pui32DataTx[2] = 0x00;
	    }

	    WriteSPI();
	    isState = !isState;

	    delayMS(1000);
	}
}
