//*****************************************************************************
//
// STC-1200 Digital Transport Controller Boot Loader for Ampex MM-1200
//
// Copyright (C) 2016-2018, RTZ Professional Audio, LLC
// All Rights Reserved
//
// RTZ is registered trademark of RTZ Professional Audio, LLC
//
//*****************************************************************************/

#ifndef __CUSTOM_H
#define __CUSTOM_H

/* External Debug Function Prototypes */
extern void ConfigureUART(void);
extern void UARTPutch(char ch);
extern void UARTPuts(char* s);
extern void UARTprintf(const char *pcString, ...);

#endif /* __CUSTOM_H */
