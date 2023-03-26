#ifndef _STM8_UART_H
#define _STM8_UART_H
/*==================================================================
  File Name: uart.h
  Author   : Emile
  ------------------------------------------------------------------
  Purpose  : This is the header-file for uart.c
  ------------------------------------------------------------------
  This file is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this software.  If not, see <http://www.gnu.org/licenses/>.
  ================================================================== */ 
#include "stm8_hw_init.h"

#define UART_BUFLEN (50)
#define TX_BUF_SIZE (30)
#define RX_BUF_SIZE (30)
     
//---------------------------------------------------      
// BAUDRATE: desired baud-rate for UART1
// FHSE: Frequency of high-speed external oscillator
// FHSI: Frequency of high-speed internal oscillator
//---------------------------------------------------      
#define BAUDRATE      (38400)      
#define FHSE          (24000000)      
#define FHSI          (16000000)      

//-------------------------------------------------------------------
//  16 MHz, 115200: 139 = 0x008B, BRR1=0x08, BRR2=0x0B, err=-0.08%
//  24 MHz, 115200: 208 = 0x00D0, BRR1=0x0D, BRR2=0x00, err=+0.16%
//  16 MHz,  57600: 278 = 0x0116, BRR1=0x11, BRR2=0x06, err=-0.08%
//  24 MHz,  57600: 417 = 0x01A1, BRR1=0x1A, BRR2=0x01, err=-0.08%
//-------------------------------------------------------------------
#define UART_DIV_HSE  ((FHSE+(BAUDRATE/2))/BAUDRATE)
#define UART_DIV_HSI  ((FHSI+(BAUDRATE/2))/BAUDRATE)
#define UART1BRR2_HSE (((UART_DIV_HSE & 0xF000)>>8) | (UART_DIV_HSE & 0x000F))
#define UART1BRR1_HSE ((UART_DIV_HSE & 0x0FF0)>>4)
#define UART1BRR2_HSI (((UART_DIV_HSI & 0xF000)>>8) | (UART_DIV_HSI & 0x000F))
#define UART1BRR1_HSI ((UART_DIV_HSI & 0x0FF0)>>4)
      
//------------------------------
// Function prototypes
//------------------------------
void    uart_init(uint8_t clk);
void    uart_printf(char *s);
bool    uart_kbhit(void);
uint8_t uart_getc(void);
void    uart_putc(uint8_t ch);

#endif