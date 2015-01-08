/*
 * File:		uart.h
 * Purpose:     Provide common ColdFire uart routines for polled serial IO
 *
 * Notes:
 */

#ifndef __uart_H__
#define __uart_H__

#include <MKL25Z4.h>
/********************************************************************/


void uart0_init (int sysclk, int baud);
char uart0_getchar (void);
void uart0_putchar (char ch);
int uart0_getchar_present (void);

/********************************************************************/

#endif /* __uart_H__ */
