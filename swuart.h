#ifndef SWUART_H
#define SWUART_H

#include    "stddef.h"

void    UARTInit();
void    UARTServ();
void    UARTPutChar(uchar ch);
uchar   UARTGetChar();
ushort  UARTRxReady();


#endif


