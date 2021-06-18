//*****************************************************************************
//
// STC-1200 Digital Transport Controller Boot Loader for Ampex MM-1200
//
// Copyright (C) 2016-2018, RTZ Professional Audio, LLC
// All Rights Reserved
//
// RTZ is registered trademark of RTZ Professional Audio, LLC
//
//*****************************************************************************
//
// bl_main.c - The file holds the main control loop of the boot loader.
//
// Copyright (c) 2006-2017 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_flash.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
//#include "inc/hw_nvic.h"
#include "inc/hw_ssi.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "bl_config.h"
#include "bootloader/bl_flash.h"
#include "bootloader/bl_hooks.h"

//
// These define the X24015 LED's that are used to blink during the bootloader
// update process. These are the alarm (ALM) and activity (ACT) LED's.
//

#define LED_GPIO_SYSCTL_PERIPH  SYSCTL_PERIPH_GPIOP
#define LED_GPIO_PORT_BASE      GPIO_PORTP_BASE         // LED's port base
#define LED_STAT_PIN            GPIO_PIN_3              // LED STAT pin

#define PIN_LOW     ( 0)
#define PIN_HIGH    (~0)

/*** Static Function Prototypes */
static void BlinkLED(int n);

/*** Global Data ***/
static int count = 0;

//*****************************************************************************
//
// Performs application-specific low level hardware initialization on system
// reset.
//
// If hooked, this function will be called immediately after the boot loader
// code relocation completes.  An application may perform any required low
// hardware initialization during this function.  Note that the system clock
// has not been set when this function is called.  Initialization that assumes
// the system clock is set may be performed in the BL_INIT_FN_HOOK function
// instead.
//
// void MyHwInitFunc(void);
//
//*****************************************************************************

void MyHwInitFunc(void)
{
}

//*****************************************************************************
//
// Performs application-specific reinitialization on boot loader entry via SVC.
//
// If hooked, this function will be called immediately after the boot loader
// reinitializes the system clock when it is entered from an application
// via the SVC mechanism rather than as a result of a system reset.  An
// application may perform any additional reinitialization in this function.
//
// void MyReinitFunc(void);
//
//*****************************************************************************

//void MyReinitFunc()
//{
//}

//*****************************************************************************
//
// Performs application-specific initialization on system reset.
//
// This function will be called immediately after the boot loader sets the 
// system clock.
//
// void MyInitFunc(void);
//
//*****************************************************************************

void MyInitFunc(void)
{
    // Enable Port peripheral
    ROM_SysCtlPeripheralEnable(LED_GPIO_SYSCTL_PERIPH);
    // Enable LED pin GPIOOutput
    ROM_GPIOPinTypeGPIOOutput(LED_GPIO_PORT_BASE, LED_STAT_PIN);
    // LED off initially
    ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, LED_STAT_PIN, PIN_LOW);
}

//*****************************************************************************
//
// Performs application-specific cleanup before the bootloader exits.
//
// This function will be called just prior to jumping to the application.
//
// void MyExitFunc(void);
//
//*****************************************************************************

void MyExitFunc(void)
{
    // Reset and disable the GPIO peripherals used by the boot loader.
    ROM_SysCtlPeripheralDisable(LED_GPIO_SYSCTL_PERIPH);
    ROM_SysCtlPeripheralReset(LED_GPIO_SYSCTL_PERIPH);
}

//*****************************************************************************
//
// Informs an application that a firmware update is starting.
//
// If hooked, this function will be called when a new firmware download is
// about to start.  The application may use this signal to initialize any
// progress display.
//
// void MyStartFunc(void);
//
//*****************************************************************************

void MyStartFunc(void)
{
    // Turn on STAT_LED1
    ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, LED_STAT_PIN, PIN_LOW);

    UARTPuts("\nBootloader starting\n");

    BlinkLED(1);
}

//*****************************************************************************
//
// Indicates the firmware flash process has begun
//
// If hooked, this function will be called when the firmware image file
// is opened on the SD drive. If an error occurs, the error code is passed.
//
// void MyOpenFunc(void);
//
//*****************************************************************************

void MyMountFunc(uint32_t error)
{
    if (error)
    {
        UARTprintf("Error %d : mounting SD drive\n", error);

        // Error mounting the SD card
        BlinkLED((int)error);
    }
    else
    {
        UARTprintf("Mounting SD drive\n");

        // Error mounting the SD card
        BlinkLED(1);
    }
}

//*****************************************************************************
//
// Indicates the firmware flash process has begun
//
// If hooked, this function will be called when the firmware image file
// is opened on the SD drive. If an error occurs, the error code is passed.
//
// void MyOpenFunc(void);
//
//*****************************************************************************

void MyOpenFunc(uint32_t error)
{
    if (error)
    {
        UARTprintf("Error %d : opening image\n", error);

        BlinkLED((int)error);
    }
    else
    {
        BlinkLED(1);

        UARTprintf("Opening image: %s\n", BL_IMAGE_FILENAME);
    }
}

//*****************************************************************************
//
// Indicates the firmware flash process has begun
//
// If hooked, this function will be called when a firmware download ends.
// The application may use this signal to update its user interface.  Typically
// a system reset will occur shortly after this function returns as the boot
// loader attempts to boot the new image.
//
// void MyEndFunc(void);
//
//*****************************************************************************

void MyBeginFunc(void)
{
    // Turn on STAT_LED while flashing
    ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, LED_STAT_PIN, PIN_HIGH);

    UARTprintf("Flashing image\n");
}

//*****************************************************************************
//
// Indicates the firmware flash has successfully completed
//
// If hooked, this function will be called when a firmware download ends.
// The application may use this signal to update its user interface.  Typically
// a system reset will occur shortly after this function returns as the boot
// loader attempts to boot the new image.
//
// void MyEndFunc(void);
//
//*****************************************************************************

void MyEndFunc(void)
{
    // Status both LED's off
    ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, LED_STAT_PIN, PIN_LOW);

    UARTprintf("\nFlash complete\n");
}

//*****************************************************************************
//
// Informs an application of download progress.
//
// If hooked, this function will be called periodically during firmware
// download.  The application may use this to update its user interface.
// When using a protocol which does not inform the client of the final size of
// the download in advance (e.g. TFTP), the ulTotal parameter will be 0,
// otherwise it indicates the expected size of the complete download.
//
// void MyProgressFunc(uint32_t ulCompleted, uint32_t ulTotal);
//
// where:
//
// - ulCompleted indicates the number of bytes already downloaded.
// - ulTotal indicates the number of bytes expected or 0 if this is not known.
//
//*****************************************************************************

void MyProgressFunc(uint32_t ulCompleted, uint32_t ulTotal)
{
    if (++count >= 1)
    {
        count = 0;

        UARTPutch('.');

        // Toggle status LED on PP2
        uint32_t pin = ROM_GPIOPinRead(LED_GPIO_PORT_BASE, LED_STAT_PIN) ? PIN_LOW : PIN_HIGH;

        ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, LED_STAT_PIN, pin);
    }
}

//*****************************************************************************
// Helper Functions to blink status LED
//*****************************************************************************

void BlinkLED(int n)
{
    int i;

    for (i=0; i < n; i++)
    {
        ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, LED_STAT_PIN, LED_STAT_PIN);
        ROM_SysCtlDelay(3000000);
        ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, LED_STAT_PIN, !LED_STAT_PIN);
        ROM_SysCtlDelay(3000000);
    }
}

// END-OF-FILE
