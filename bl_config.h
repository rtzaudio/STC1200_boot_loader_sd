//*****************************************************************************
//
// bl_config.h - The configurable parameters of the boot loader.
//
// Copyright (c) 2010-2017 Texas Instruments Incorporated.  All rights reserved.
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

#ifndef __BL_CONFIG_H__
#define __BL_CONFIG_H__

#include "custom.h"

//*****************************************************************************
//
// The following defines are used to configure the operation of the boot
// loader.  For each define, its interactions with other defines are described.
// First is the dependencies (i.e. the defines that must also be defined if it
// is defined), next are the exclusives (i.e. the defines that can not be
// defined if it is defined), and finally are the requirements (i.e. the
// defines that must be defined if it is defined).
//
// The following defines must be defined in order for the boot loader to
// operate:
//
//     One of CAN_ENABLE_UPDATE, ENET_ENABLE_UPDATE, I2C_ENABLE_UPDATE,
//            SSI_ENABLE_UPDATE, UART_ENABLE_UPDATE, or USB_ENABLE_UPDATE
//     APP_START_ADDRESS
//     STACK_SIZE
//     BUFFER_SIZE
//
//*****************************************************************************

//*****************************************************************************
//
// The frequency of the crystal used to clock the microcontroller.
//
// This defines the crystal frequency used by the microcontroller running the
// boot loader.  If this is unknown at the time of production, then use the
// UART_AUTOBAUD feature to properly configure the UART.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
#define CRYSTAL_FREQ            25000000

//*****************************************************************************
//
// This enables the boosting of the LDO voltage to 2.75V.  For boot loader
// configurations that enable the PLL (for example, using the Ethernet port)
// on a part that has the PLL errata, this should be enabled.  This applies to
// revision A2 of Fury-class devices.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
//#define BOOST_LDO_VOLTAGE

//*****************************************************************************
//
// The starting address of the application.  This must be a multiple of 1024
// bytes (making it aligned to a page boundary).  A vector table is expected at
// this location, and the perceived validity of the vector table (stack located
// in SRAM, reset vector located in flash) is used as an indication of the
// validity of the application image.
//
// The flash image of the boot loader must not be larger than this value.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
#define APP_START_ADDRESS       0x00004000

//*****************************************************************************
//
// The address at which the application locates its exception vector table.
// This must be a multiple of 1KB (making it aligned to a page boundary).
// Typically, an application will start with its vector table and this value
// will default to APP_START_ADDRESS.  This option is provided to cater for
// applications which run from external memory which may not be accessible by
// the NVIC (the vector table offset register is only 30 bits long).
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
#define VTABLE_START_ADDRESS    0x00004000

//*****************************************************************************
//
// The size of a single, erasable page in the flash.  This must be a power
// of 2.  The default value of 1KB represents the page size for the internal
// flash on all Tiva MCUs and this value should only be overridden if
// configuring a boot loader to access external flash devices with a page size
// different from this.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
#define FLASH_PAGE_SIZE         0x00004000

//*****************************************************************************
//
// The amount of space at the end of flash to reserved.  This must be a
// multiple of 1024 bytes (making it aligned to a page boundary).  This
// reserved space is not erased when the application is updated, providing
// non-volatile storage that can be used for parameters.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
//#define FLASH_RSVD_SPACE        0x00000800

//*****************************************************************************
//
// The number of words of stack space to reserve for the boot loader.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
#define STACK_SIZE              1024

//*****************************************************************************
//
// The number of words in the data buffer used for receiving packets.  This
// value must be at least 3.  If using autobauding on the UART, this must be at
// least 20.  The maximum usable value is 65 (larger values will result in
// unused space in the buffer).
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
#define BUFFER_SIZE             20

//*****************************************************************************
//
// Enables the pin-based forced update check.  When enabled, the boot loader
// will go into update mode instead of calling the application if a pin is read
// at a particular polarity, forcing an update operation.  In either case, the
// application is still able to return control to the boot loader in order to
// start an update.
//
// Depends on: None
// Exclusive of: None
// Requires: FORCED_UPDATE_PERIPH, FORCED_UPDATE_PORT, FORCED_UPDATE_PIN,
//           FORCED_UPDATE_POLARITY
//
//*****************************************************************************
#define ENABLE_UPDATE_CHECK

//*****************************************************************************
//
// The GPIO module to enable in order to check for a forced update.  This will
// be one of the SYSCTL_RCGCGPIO_Rx values, where Rx represents the required
// GPIO port. This applies to Blizzard class and later devices for
// FORCED_UPDATE_PORT.
//
// Depends on: ENABLE_UPDATE_CHECK
// Exclusive of: None
// Requries: None
//
//*****************************************************************************
#define FORCED_UPDATE_PERIPH    SYSCTL_RCGCGPIO_R7		/* Port-H */

//*****************************************************************************
//
// The GPIO port to check for a forced update.  This will be one of the
// GPIO_PORTx_BASE values, where "x" is replaced with the port name (such as
// B).  The value of "x" should match the value of "x" for
// FORCED_UPDATE_PERIPH.
//
// Depends on: ENABLE_UPDATE_CHECK
// Exclusive of: None
// Requries: None
//
//*****************************************************************************
#define FORCED_UPDATE_PORT      GPIO_PORTH_BASE

//*****************************************************************************
//
// The pin to check for a forced update.  This is a value between 0 and 7.
//
// Depends on: ENABLE_UPDATE_CHECK
// Exclusive of: None
// Requries: None
//
//*****************************************************************************
#define FORCED_UPDATE_PIN       3     // PH3 connected to BOOT switch

//*****************************************************************************
//
// The polarity of the GPIO pin that results in a forced update.  This value
// should be 0 if the pin should be low and 1 if the pin should be high.
//
// Depends on: ENABLE_UPDATE_CHECK
// Exclusive of: None
// Requries: None
//
//*****************************************************************************
#define FORCED_UPDATE_POLARITY  0     // PF0 = 0 triggers boot loader update

//*****************************************************************************
//
// This enables a weak pull up for the GPIO pin used in a forced update.  This
// value should be 0 if the pin should be have an internal weak pull down and
// 1 if the pin should have an interal weak pull up.
// Only FORCED_UPDATE_WPU or FORCED_UPDATE_WPD or neither should be defined.
//
// Depends on: ENABLE_UPDATE_CHECK
// Exclusive of: None
// Requries: None
//
//*****************************************************************************
//#define FORCED_UPDATE_WPU
//#define FORCED_UPDATE_WPD

//*****************************************************************************
//
// This enables the use of the GPIO_LOCK mechanism for configuration of
// protected GPIO pins (for example JTAG pins).  If this value is not defined,
// the locking mechanism will not be used.  The only legal values for this
// feature are GPIO_LOCK_KEY for Fury devices and GPIO_LOCK_KEY_DD for all
// other devices except Sandstorm devices, which do not support this feature.
//
// Depends on: ENABLE_UPDATE_CHECK
// Exclusive of: None
// Requries: None
//
//*****************************************************************************
//#define FORCED_UPDATE_KEY       GPIO_LOCK_KEY
//#define FORCED_UPDATE_KEY       GPIO_LOCK_KEY_DD

//*****************************************************************************
//
// Boot loader hook functions.
//
// The following defines allow you to add application-specific function which
// are called at various points during boot loader execution.
//
//*****************************************************************************

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
//#define BL_HW_INIT_FN_HOOK      MyHwInitFunc

//*****************************************************************************
//
// Performs application-specific initialization on system reset.
//
// If hooked, this function will be called immediately after the boot loader
// sets the system clock.  An application may perform any additional
// initialization during this function.
//
// void MyInitFunc(void);
//
//*****************************************************************************
#define BL_INIT_FN_HOOK         MyInitFunc

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
//#define BL_REINIT_FN_HOOK       MyReinitFunc

//*****************************************************************************
//
// Informs an application that a download is starting.
//
// If hooked, this function will be called when a new firmware download is
// about to start.  The application may use this signal to initialize any
// progress display.
//
// void MyStartFunc(void);
//
//*****************************************************************************
#define BL_START_FN_HOOK        MyStartFunc

//*****************************************************************************
//
// Informs an application of the SD drive mount status
//
// If hooked, this function will be called when loader mounts the SD drive.
//
// void MyMountFunc(uint32_t error);
//
//*****************************************************************************
#define BL_MOUNT_FN_HOOK        MyMountFunc

//*****************************************************************************
//
// Informs an application of the SD drive image file open status.
//
// If hooked, this function will be called when loader opens the SD drive.
//
// void MyOpenFunc(uint32_t error);
//
//*****************************************************************************
#define BL_OPEN_FN_HOOK         MyOpenFunc

//*****************************************************************************
//
// Informs an application that the flash memory process has started.
//
// If hooked, this function will be called when loader begins the firmware
// flash process.
//
// void MyBeginFunc(uint32_t error);
//
//*****************************************************************************
#define BL_BEGIN_FN_HOOK        MyBeginFunc

//*****************************************************************************
//
// Informs an application that the flash memory process has completed.
//
// If hooked, this function will be called when loader completes the firmware
// flash process.
//
// void MyEndFunc(uint32_t error);
//
//*****************************************************************************
#define BL_END_FN_HOOK          MyEndFunc

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
// void MyProgressFunc(unsigned long ulCompleted, unsigned long ulTotal);
//
// where:
//
// - ulCompleted indicates the number of bytes already downloaded.
// - ulTotal indicates the number of bytes expected or 0 if this is not known.
//
//*****************************************************************************
#define BL_PROGRESS_FN_HOOK     MyProgressFunc

//*****************************************************************************
//
// Informs application that the flash memory process is exiting the bootloader.
//
// If hooked, this function will be called when loader completes the firmware
// flash process.
//
// void MyExitFunc(uint32_t error);
//
//*****************************************************************************
#define BL_EXIT_FN_HOOK         MyExitFunc

//*****************************************************************************
//
// Defines the application bin image filename to look for on the SD drive.
// The boot loader will attempt to mount and open this file ten times. If
// successful, it will flash the contents of the file into program
// memory space.
//
//*****************************************************************************
#define BL_IMAGE_FILENAME       "STC1200.bin"

//*****************************************************************************
//
// STC-1200 SD Card Socket SPI Pin Assignments
//
//  .baseAddr = SSI1_BASE,          /* SPI base address */
//  .portSCK  = GPIO_PORTB_BASE,    /* SPI SCK PORT */
//  .pinSCK   = GPIO_PIN_5,         /* SCK PIN (PB5) */
//  .portMISO = GPIO_PORTE_BASE,    /* SPI MISO PORT */
//  .pinMISO  = GPIO_PIN_5,         /* MISO PIN (PE5) */
//  .portMOSI = GPIO_PORTE_BASE,    /* SPI MOSI PORT */
//  .pinMOSI  = GPIO_PIN_4,         /* MOSI PIN (PE4) */
//  .portCS   = GPIO_PORTB_BASE,    /* GPIO CS PORT */
//  .pinCS    = GPIO_PIN_4          /* CS PIN (PB4) */
//
//*****************************************************************************

// Peripherals used by SD drive
#define SD_SYSCTL_PERIPH_SSI            SYSCTL_PERIPH_SSI1
#define SD_SYSCTL_PERIPH_GPIO_SCLK      SYSCTL_PERIPH_GPIOB
#define SD_SYSCTL_PERIPH_GPIO_MOSI      SYSCTL_PERIPH_GPIOE
#define SD_SYSCTL_PERIPH_GPIO_MISO      SYSCTL_PERIPH_GPIOE
#define SD_SYSCTL_PERIPH_GPIO_FSS       SYSCTL_PERIPH_GPIOB

/* Base port for SD SSI */
#define SD_SSI_BASE                     SSI1_BASE

/* SD Drive FSS port/pin (PB4) definitions */
#define SD_GPIO_FSS_BASE                GPIO_PORTB_BASE
#define SD_GPIO_FSS_PINCFG              GPIO_PB4_SSI1FSS
#define SD_GPIO_FSS_PIN                 GPIO_PIN_4

/* SD Drive SCLK port/pin (PB5) definitions */
#define SD_GPIO_SCLK_BASE               GPIO_PORTB_BASE
#define SD_GPIO_SCLK_PINCFG             GPIO_PB5_SSI1CLK
#define SD_GPIO_SCLK_PIN                GPIO_PIN_5

/* SD Drive MOSI port/pin (PE4) definitions */
#define SD_GPIO_MOSI_BASE               GPIO_PORTE_BASE
#define SD_GPIO_MOSI_PINCFG             GPIO_PE4_SSI1XDAT0
#define SD_GPIO_MOSI_PIN                GPIO_PIN_4

/* SD Drive MISO port/pin (PE5) definitions */
#define SD_GPIO_MISO_BASE               GPIO_PORTE_BASE
#define SD_GPIO_MISO_PINCFG             GPIO_PE5_SSI1XDAT1
#define SD_GPIO_MISO_PIN                GPIO_PIN_5

//*****************************************************************************
// THE UART IS AVAILABLE FOR DEBUG OUTPUT MESSAGE SUPPORT
//*****************************************************************************

//*****************************************************************************
//
// Selects the baud rate to be used for the UART.
//
// Depends on: UART_ENABLE_UPDATE, CRYSTAL_FREQ
// Exclusive of: UART_AUTOBAUD
// Requires: None
//
//*****************************************************************************
#define UART_FIXED_BAUDRATE     115200

//*****************************************************************************
//
// Selects the clock enable for the UART peripheral module
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UARTx_BASE
//
//*****************************************************************************
#define UART_CLOCK_ENABLE         SYSCTL_RCGCUART_R1

//*****************************************************************************
//
// Selects the base address of the UART peripheral module
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_CLOCK_ENABLE
//
//*****************************************************************************
#define UARTx_BASE                UART1_BASE

//*****************************************************************************
//
// Selects the clock enable for the GPIO corresponding to UART RX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_RXPIN_BASE, UART_RXPIN_PCTL and UART_RXPIN_POS
//
//*****************************************************************************
#define UART_RXPIN_CLOCK_ENABLE SYSCTL_RCGCGPIO_R1    // Port-B Clock

//*****************************************************************************
//
// Selects the base address for the GPIO corresponding to UART RX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_RXPIN_CLOCK_ENABLE, UART_RXPIN_PCTL and UART_RXPIN_POS
//
//*****************************************************************************
#define UART_RXPIN_BASE         GPIO_PORTB_BASE         // PB0

//*****************************************************************************
//
// Selects the port control value for the GPIO corresponding to UART RX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_RXPIN_CLOCK_ENABLE, UART_RXPIN_BASE and UART_RXPIN_POS
//
//*****************************************************************************
#define UART_RXPIN_PCTL         0x1

//*****************************************************************************
//
// Selects the pin number for the GPIO corresponding to UART RX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_RXPIN_CLOCK_ENABLE, UART_RXPIN_BASE and UART_RXPIN_PCTL
//
//*****************************************************************************
#define UART_RXPIN_POS          0                       // PB0

//*****************************************************************************
//
// Selects the clock enable for the GPIO corresponding to UART TX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_TXPIN_BASE, UART_TXPIN_PCTL and UART_TXPIN_POS
//
//*****************************************************************************
#define UART_TXPIN_CLOCK_ENABLE SYSCTL_RCGCGPIO_R1      // Port-B Clock

//*****************************************************************************
//
// Selects the base address for the GPIO corresponding to UART TX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_TXPIN_CLOCK_ENABLE, UART_TXPIN_PCTL and UART_TXPIN_POS
//
//*****************************************************************************
#define UART_TXPIN_BASE         GPIO_PORTB_BASE         // PB1

//*****************************************************************************
//
// Selects the port control value for the GPIO corresponding to UART TX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_TXPIN_CLOCK_ENABLE, UART_TXPIN_BASE and UART_TXPIN_POS
//
//*****************************************************************************
#define UART_TXPIN_PCTL         0x1

//*****************************************************************************
//
// Selects the pin number for the GPIO corresponding to UART TX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_TXPIN_CLOCK_ENABLE, UART_TXPIN_BASE and UART_TXPIN_PCTL
//
//*****************************************************************************
#define UART_TXPIN_POS          1                       // PB1

#endif // __BL_CONFIG_H__
