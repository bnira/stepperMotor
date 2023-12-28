//*****************************************************************************
// Modifications and control by Boniface Thuranira
// lab2_UART.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2011-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the DK-TM4C123G Firmware Package.
//
//*****************************************************************************
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "grlib/grlib.h"
#include "drivers/cfal96x64x16.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "driverlib/adc.h"
int32_t exit_program = 0;
tRectangle sRect;
tContext sContext;
int i;
int32_t local_char = ' ';
char buf[10];
int buttonPressed;
int blinkyToggle=1;
bool blinkyOn = false;
#define timeOn 20000
#define timeOff 3800000
#define ui32Flags
#define MSG_OBJ_RX_INT_ENABLE
#define stepsRev 200
#define INIT_RPM 60
#define RPM_MIN 1
const uint8_t full_step_array[4] = {0x0C, 0x06, 0x03, 0x09};
const uint8_t wave_drive_array[4] = {0x08, 0x04, 0x02, 0x01};
int wave = 0;
int signal = 0;
#define MOTOR_OFF (0x00)
int32_t RPM = 60;
 volatile uint32_t ui32Loop;
 bool ADCOn = false;
char buf[10];
unsigned long period;
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (lab2_UART)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void flashLed();

void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE,
                                   UARTCharGetNonBlocking(UART0_BASE));
    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
}
void printMenu() {
    UARTSend("\r\n", 2);
    UARTSend("Menu selection:\r\n", 17);
    UARTSend("M - print this menu\r\n", 21);
    UARTSend("Q - quit the program\r\n", 23);
    UARTSend("D - toggle flashing LED\r\n", 25);
  }
void processMenu(int32_t local_char){
  if (local_char == 'M') {
    printMenu(); 
  }
      else if (local_char == 'Q') {
      exit_program = 1;
      UARTSend("Terminal stopped successfully\r\n", 31);
      }
     else if (local_char == 'D') {
        blinkyOn = !blinkyOn;
      }
   else if (local_char == 'W') {
       RPM++;
     }
     else if (local_char == 'A') {
       RPM--;
       if (RPM == 0) {
         RPM = 1;
       }
     }
      else if (local_char == 'D') {
        blinkyOn = !blinkyOn;
       
      }
}
void 
enableUART() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
   
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    
    IntEnable(INT_UART0); 
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}
void 
countDown(void)
{
  int div = 0;
  div = ((RPM * stepsRev)/INIT_RPM);
  
}
void
Timer0IntHandler(void)
{
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    
    //
    // Toggle the flag for the first timer.
    //
    if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_0)) {
      GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0| GPIO_PIN_1, 0); 
    }
    else
    {
      GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_1, 4);
    }
    /*ADCProcessorTrigger(ADC0_BASE, 3);  
    while (!ADCIntStatus(ADC0_BASE, 3, false)) {
    }
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, ADCValueStore);
    if (countToggle == 0) {
      Req = ADCValueStore[0] / 10;
    }
    else if (countToggle == 1) {
      Req = ADCValueStore[0] * 100;
    }*/
   /* TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / Req);
    // Update the interrupt status on the display.
    sprintf(buf, "Req = %d     ", Req);
    GrStringDrawCentered(&sContext, buf, -1,
                         GrContextDpyWidthGet(&sContext) / 2, 35, 1);
    sprintf(buf, "Srv = %d   ", SRV);
    GrStringDrawCentered(&sContext, buf, -1,
                         GrContextDpyWidthGet(&sContext) / 2, 20, 1);
    SRV = 0;*/
    IntMasterEnable();
  
    if(signal==0)
      GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, wave_drive_array[wave]);
    else if (signal==1)
      GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, full_step_array[wave]);
    
    
    wave++;
    if(wave == 3) {
      wave=0;
    }

    sprintf(buf, "wave = %d     ", wave);
    GrStringDrawCentered(&sContext, buf, -1,
                         GrContextDpyWidthGet(&sContext) / 2, 35, 1);
      
}
void splashScreen() {
     GrContextFontSet(&sContext, g_psFontCm12);
     GrContextForegroundSet(&sContext, ClrWhite);
     GrStringDrawCentered(&sContext, "Welcome!", -1, 
GrContextDpyWidthGet(&sContext) / 2,((GrContextDpyHeightGet(&sContext) - 18) / 2) +
20, 0);
     SysCtlDelay(16000000);
     sRect.i16XMin = 0;
     sRect.i16YMin = 0;
     sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
     sRect.i16YMax = 60;
     GrContextForegroundSet(&sContext, ClrBlack);
     GrRectFill(&sContext, &sRect);
     }
void blinky() {
  if(blinkyOn == false){
             GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_PIN_2); 
             SysCtlDelay(timeOn);
           for(ui32Loop = 0; ui32Loop < timeOn; ui32Loop++);
            GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0);
            SysCtlDelay(timeOff);
           for(ui32Loop = 0; ui32Loop < timeOff; ui32Loop++);
      }
}
void
enableTimer() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/*period -1*/);
 
    period = (SysCtlClockGet() / 10) / 2;
    
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    
    
    TimerEnable(TIMER0_BASE, TIMER_A);
}


//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
//
// This displaysthe menu in puTTY
int
main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                     SYSCTL_XTAL_16MHZ);
    //SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
     //                  SYSCTL_OSC_MAIN);
    IntMasterDisable();
    //
    // Initialize the display driver.
    //
    CFAL96x64x16Init();

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sCFAL96x64x16);
    enableUART();

     /* SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
 
  /*  period = (SysCtlClockGet() / 10) / 2;
    
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    
    
    TimerEnable(TIMER0_BASE, TIMER_A);*/
    // enable ADC
    /*SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)) {
    }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {    
    }
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_1);
    ADCSequenceDisable(ADC0_BASE, 3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH17 | ADC_CTL_IE | 
ADC_CTL_END);
  //  ADCSequenceStepConfigure();
   // ADCSequenceStepConfigure();
    ADCSequenceEnable(ADC0_BASE, 3); */
    splashScreen();
    //
    // Fill the top part of the screen with blue to create the banner.
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 9;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);

    //
    // Change foreground for white text.
    //
    GrContextForegroundSet(&sContext, ClrWhite);

    //
    // Put the application name in the middle of the banner.
    //
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    GrStringDrawCentered(&sContext, "stepperMotor ctrl", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 4, 0);
    
   

    //
    // Initialize the display and write some instructions.
    //
   /* GrStringDrawCentered(&sContext, "Connect a", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 20, false);
    GrStringDrawCentered(&sContext, "terminal", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 30, false);
    GrStringDrawCentered(&sContext, "to UART0.", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 40, false);
    GrStringDrawCentered(&sContext, "115000,N,8,1", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 50, false);*/
     
    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
   /* IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
   */
    //
    // Prompt for text to be entered.
    //
    UARTSend((uint8_t *)"Enter text: ", 12);

    //
    // Loop forever echoing data through the UART.
    //
    //int32_t local_char = ' ';
   // int CharRecv;
    printMenu();
    
    int i = 1;
    IntMasterEnable();
    while(i == 1) {
    processMenu(local_char);}
    while(1) {
      blinkyOn =!blinkyOn;  }
    while(exit_program == 0) 
    {
    i++;
    while (UARTCharsAvail(UART0_BASE)) {
      local_char = UARTCharGetNonBlocking(UART0_BASE);
      if (local_char != -1) {
       //  CharRecv++;
      UARTSend("\r\n",2);
      UARTSend("Your selection was: ",20);
      UARTSend("\r\n",2);
      UARTSend((uint8_t *)&local_char, 1);
      processMenu(local_char);
      }
        }
    
     if(blinkyOn == false){
             GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_PIN_2); 
             SysCtlDelay(timeOn);
           for(ui32Loop = 0; ui32Loop < timeOn; ui32Loop++);
            GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0);
            SysCtlDelay(timeOff);
           for(ui32Loop = 0; ui32Loop < timeOff; ui32Loop++);
      }
    }
}


