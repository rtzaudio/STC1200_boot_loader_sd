//*****************************************************************************
//
// bl_main.c - The file holds the main control loop of the boot loader.
//
// Copyright (c) 2006-2020 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 2.2.0.295 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "inc/hw_gpio.h"
#include "inc/hw_flash.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ssi.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "bl_config.h"
#include "bootloader/bl_flash.h"
#include "bootloader/bl_hooks.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "Petit/pff.h"
#include "Petit/pffconf.h"

//*****************************************************************************
// Specifies how many bytes read from file to write flash at one time
// if higher increase ram size decrease program load time
// else decrease ram size increase program load time
//*****************************************************************************

#define WRITE_DATA_PACKET_SIZE  128

static BYTE bWriteBuffer[WRITE_DATA_PACKET_SIZE];
static FATFS fatfs;

// Function Prototypes

void ConfigureSSIPort(uint32_t ui32Protocol, uint32_t ui32Mode,
                      uint32_t ui32BitRate, uint32_t ui32DataWidth);

//*****************************************************************************
//
// Make sure that the application start address falls on a flash page boundary
//
//*****************************************************************************
#if (APP_START_ADDRESS & (FLASH_PAGE_SIZE - 1))
#error ERROR: APP_START_ADDRESS must be a multiple of FLASH_PAGE_SIZE bytes!
#endif

//*****************************************************************************
//
// Make sure that the flash reserved space is a multiple of flash pages.
//
//*****************************************************************************
#if (FLASH_RSVD_SPACE & (FLASH_PAGE_SIZE - 1))
#error ERROR: FLASH_RSVD_SPACE must be a multiple of FLASH_PAGE_SIZE bytes!
#endif

//*****************************************************************************
//
//! \addtogroup bl_main_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// A prototype for the function (in the startup code) for calling the
// application.
//
//*****************************************************************************
extern void CallApplication(uint32_t ui32Base);

//*****************************************************************************
//
// A prototype for the function (in the startup code) for a predictable length
// delay.
//
//*****************************************************************************
extern void Delay(uint32_t ui32Count);

//*****************************************************************************
//
// Converts a word from big endian to little endian.  This macro uses compiler-
// specific constructs to perform an inline insertion of the "rev" instruction,
// which performs the byte swap directly.
//
//*****************************************************************************
#if defined(ewarm)
#include <intrinsics.h>
#define SwapWord(x)             __REV(x)
#endif
#if defined(codered) || defined(gcc) || defined(sourcerygxx)
#define SwapWord(x) __extension__                                             \
        ({                                                                    \
             register uint32_t __ret, __inp = x;                              \
             __asm__("rev %0, %1" : "=r" (__ret) : "r" (__inp));              \
             __ret;                                                           \
        })
#endif
#if defined(rvmdk) || defined(__ARMCC_VERSION)
#define SwapWord(x)             __rev(x)
#endif
#if defined(ccs)
uint32_t
SwapWord(uint32_t x)
{
    __asm("    rev     r0, r0\n"
          "    bx      lr\n"); // need this to make sure r0 is returned
    return(x + 1); // return makes compiler happy - ignored
}
#endif

//*****************************************************************************
//
//! Configures the microcontroller.
//!
//! This function configures the peripherals and GPIOs of the microcontroller,
//! preparing it for use by the boot loader.  The interface that has been
//! selected as the update port will be configured, and auto-baud will be
//! performed if required.
//!
//! \return None.
//
//*****************************************************************************

void ConfigureDevice(void)
{
#ifdef CRYSTAL_FREQ
    //
    // Since the crystal frequency was specified, enable the main oscillator
    // and clock the processor from it.
    //
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    //
    // Since the crystal frequency was specified, enable the main oscillator
    // and clock the processor from it. Check for whether the Oscillator range
    // has to be set and wait states need to be updated
    //
    if(CRYSTAL_FREQ >= 10000000)
    {
        HWREG(SYSCTL_MOSCCTL) |= (SYSCTL_MOSCCTL_OSCRNG);
        HWREG(SYSCTL_MOSCCTL) &= ~(SYSCTL_MOSCCTL_PWRDN |
                                   SYSCTL_MOSCCTL_NOXTAL);
    }
    else
    {
        HWREG(SYSCTL_MOSCCTL) &= ~(SYSCTL_MOSCCTL_PWRDN |
                                   SYSCTL_MOSCCTL_NOXTAL);
    }

    //
    // Wait for the Oscillator to Stabilize
    //
    Delay(524288);

    if(CRYSTAL_FREQ > 16000000)
    {
        HWREG(SYSCTL_MEMTIM0)  = (SYSCTL_MEMTIM0_FBCHT_1_5 |
                                  (1 << SYSCTL_MEMTIM0_FWS_S) |
                                  SYSCTL_MEMTIM0_EBCHT_1_5 |
                                  (1 << SYSCTL_MEMTIM0_EWS_S) |
                                  SYSCTL_MEMTIM0_MB1);
        HWREG(SYSCTL_RSCLKCFG) = (SYSCTL_RSCLKCFG_MEMTIMU |
                                  SYSCTL_RSCLKCFG_OSCSRC_MOSC);
    }
    else
    {
        HWREG(SYSCTL_RSCLKCFG) = (SYSCTL_RSCLKCFG_OSCSRC_MOSC);
    }
#else
    HWREG(SYSCTL_RCC) &= ~(SYSCTL_RCC_MOSCDIS);
    Delay(524288);
    HWREG(SYSCTL_RCC) = ((HWREG(SYSCTL_RCC) & ~(SYSCTL_RCC_OSCSRC_M)) |
                         SYSCTL_RCC_OSCSRC_MAIN);
#endif
#endif
}

//*****************************************************************************
//
//! Configure the SSI port the SPI for the SD interface in SPI mode.
//
//*****************************************************************************

void ConfigureSSI(void)
{
    /* Enable SD SSI peripherals */

    ROM_SysCtlPeripheralEnable(SD_SYSCTL_PERIPH_SSI);
    ROM_SysCtlPeripheralEnable(SD_SYSCTL_PERIPH_GPIO_SCLK);
    ROM_SysCtlPeripheralEnable(SD_SYSCTL_PERIPH_GPIO_MOSI);
    ROM_SysCtlPeripheralEnable(SD_SYSCTL_PERIPH_GPIO_MISO);
    ROM_SysCtlPeripheralEnable(SD_SYSCTL_PERIPH_GPIO_FSS);

    /* SSI-1 Configure Pins */

    // Enable pin for SSI SSI1CLK
    ROM_GPIOPinConfigure(SD_GPIO_SCLK_PINCFG);
    ROM_GPIOPinTypeSSI(SD_GPIO_SCLK_BASE, SD_GPIO_SCLK_PIN);

    // Enable pin for SSI SSI1XDAT0(MOSI)
    ROM_GPIOPinConfigure(SD_GPIO_MOSI_PINCFG);
    ROM_GPIOPinTypeSSI(SD_GPIO_MOSI_BASE, SD_GPIO_MOSI_PIN);

    // Enable pin for SSI SSI1XDAT1(MISO)
    ROM_GPIOPinConfigure(SD_GPIO_MISO_PINCFG);
    ROM_GPIOPinTypeSSI(SD_GPIO_MISO_BASE, SD_GPIO_MISO_PIN);

    // Enable pin PB4 for SSI1 SSI1FSS
    //GPIOPinConfigure(SD_GPIO_FSS_PINCFG);
    //GPIOPinTypeSSI(SD_GPIO_FSS_BASE, SD_GPIO_FSS_PIN);
    // Enable pin PK7 for GPIOOutput (SSI1FSS_SD)
    ROM_GPIOPinTypeGPIOOutput(SD_GPIO_FSS_BASE, SD_GPIO_FSS_PIN);

    /* Configure pad settings */

    /* SCLK (PB5) */
    MAP_GPIOPadConfigSet(SD_GPIO_SCLK_BASE,
                         SD_GPIO_SCLK_PIN,
                         GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    /* MOSI (PE4) */
    MAP_GPIOPadConfigSet(SD_GPIO_MOSI_BASE,
                         SD_GPIO_MOSI_PIN,
                         GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    /* MISO (PE5) */
    MAP_GPIOPadConfigSet(SD_GPIO_MISO_BASE,
                         SD_GPIO_MISO_PIN,
                         GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
    /* CS (PK7) */
    MAP_GPIOPadConfigSet(SD_GPIO_FSS_BASE,
                         SD_GPIO_FSS_PIN,
                         GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    //
    // Configure and enable the SSI port for SPI master mode.  Use SSI1,
    // system clock supply, idle clock level low and active low clock in
    // Freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.
    //

    uint32_t sysclock = 120000000;

    ROM_SSIConfigSetExpClk(SD_SSI_BASE, sysclock, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 400000, 8);

    //
    // Enable the SSI1 module.
    //
    ROM_SSIEnable(SD_SSI_BASE);
}

//*****************************************************************************
//
//! This function performs the update on the selected port.
//!
//! This function is called directly by the boot loader or it is called as a
//! result of an update request from the application.
//!
//! \return Never returns.
//
//*****************************************************************************

void Updater(void)
{
    uint32_t EraseSize=0;
    uint32_t AppAddress=0;
    uint32_t i,j;
    uint32_t WriteDataPacketCount;
    uint32_t WriteDataPacketRemainder;
    FRESULT rc;
    UINT br;

    // Initialize UART for console messages
    ConfigureUART();

    // Initialize the SSI controller
    ConfigureSSI();

    // Indicate start of update
#ifdef BL_START_FN_HOOK
    BL_START_FN_HOOK();
#endif

    // Attempt to mount the SD file system 10 times.

    j = 0;

    do {
#ifdef BL_MOUNT_FN_HOOK
        BL_MOUNT_FN_HOOK(0);
#endif
        rc = pf_mount(&fatfs);

        // try again up to ten times
        j++;

    } while(rc && j < 10);

    if (rc)
    {
#ifdef BL_MOUNT_FN_HOOK
        BL_MOUNT_FN_HOOK(rc);
#endif
    }
    else
    {
        j = 0;

        // The SD card was mounted successfully, now try 10
        // times opening bootld.bin file which is on the SD
        // card (if exist). Blink led on every try.

        do {
            // Notify handler attempt to open file
#ifdef BL_OPEN_FN_HOOK
            BL_OPEN_FN_HOOK(0);
#endif
            // attempt to open SD data file
            rc = pf_open(BL_IMAGE_FILENAME);

            // try again up to ten times
            j++;

        } while(rc && j < 10);

        // Continue if we opened the image file successfully, otherwise notify error handler and exit.

        if (rc)
        {
            // Notify handler that open failed
#ifdef BL_OPEN_FN_HOOK
            BL_OPEN_FN_HOOK(rc);
#endif
        }
        else
        {
            // if file size is not multiple of 4 exit otherwise continue
            if ((fatfs.fsize & 0x03) == 0)
            {
#ifdef BL_BEGIN_FN_HOOK
                BL_BEGIN_FN_HOOK();
#endif
                // Calculate page count that will erase according to app.bin file size
                EraseSize = fatfs.fsize / FLASH_PAGE_SIZE;

                if (fatfs.fsize % FLASH_PAGE_SIZE)
                    EraseSize++;

                // Erase necessary pages
                AppAddress = APP_START_ADDRESS;

                for(i=0; i < EraseSize; i++)
                {
                    ROM_FlashErase(AppAddress);
                    AppAddress += FLASH_PAGE_SIZE;
                }

                // Set app address to write
                AppAddress = APP_START_ADDRESS;

                // Calculate packet count according to write data packet size that user defined
                WriteDataPacketCount = fatfs.fsize / WRITE_DATA_PACKET_SIZE;

                // Calculate remainder of division
                WriteDataPacketRemainder = fatfs.fsize % WRITE_DATA_PACKET_SIZE;

                // Read number of WRITE_DATA_PACKET_SIZE bytes from app.bin file and
                // write it to the flash memory number of WriteDataPacketCount times.

                for(i=0; i < WriteDataPacketCount; i++)
                {
                    pf_read(bWriteBuffer, WRITE_DATA_PACKET_SIZE, &br);

                    ROM_FlashProgram((uint32_t*)bWriteBuffer, AppAddress, WRITE_DATA_PACKET_SIZE);

                    AppAddress += WRITE_DATA_PACKET_SIZE;

#ifdef BL_PROGRESS_FN_HOOK
                    BL_PROGRESS_FN_HOOK(AppAddress - APP_START_ADDRESS, WriteDataPacketCount);
#endif
                }

                // Read 4 bytes from app.bin file and
                // write it to the flash memory number of WriteDataPacketRemainder times.

                for(i=0; i < WriteDataPacketRemainder/4; i++)
                {
                    pf_read(bWriteBuffer, 4, &br);

                    ROM_FlashProgram((uint32_t*)bWriteBuffer, AppAddress, 4);

                    AppAddress += 4;
                }

                // Call flash end hook if specified
#ifdef BL_END_FN_HOOK
                BL_END_FN_HOOK();
#endif
            }
        }
    }

    // Reset and disable the SSI peripherals used by the boot loader.
    ROM_SysCtlPeripheralDisable(SD_SYSCTL_PERIPH_SSI);
    ROM_SysCtlPeripheralReset(SD_SYSCTL_PERIPH_SSI);

    ROM_SysCtlPeripheralDisable(SD_SYSCTL_PERIPH_GPIO_SCLK);
    ROM_SysCtlPeripheralReset(SD_SYSCTL_PERIPH_GPIO_SCLK);

    ROM_SysCtlPeripheralDisable(SD_SYSCTL_PERIPH_GPIO_MOSI);
    ROM_SysCtlPeripheralReset(SD_SYSCTL_PERIPH_GPIO_MOSI);

    ROM_SysCtlPeripheralDisable(SD_SYSCTL_PERIPH_GPIO_MISO);
    ROM_SysCtlPeripheralReset(SD_SYSCTL_PERIPH_GPIO_MISO);

    ROM_SysCtlPeripheralDisable(SD_SYSCTL_PERIPH_GPIO_FSS);
    ROM_SysCtlPeripheralReset(SD_SYSCTL_PERIPH_GPIO_FSS);

#ifdef BL_EXIT_FN_HOOK
    BL_EXIT_FN_HOOK();
#endif

    // Branch to the specified address. This should never return.
    // If it does, very bad things will likely happen since it is
    // likely that the copy of the boot loader in SRAM will have
    // been overwritten.
    //((int (*)(void))APP_START_ADDRESS)();

    // Reset
    HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ);

    // The microcontroller should have reset, so this should
    // never be reached.  Just in case, loop forever.

    while(1)
    {
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

