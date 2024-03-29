/*==================================================================
  File Name: uart.c
  Author   : Emile
  ------------------------------------------------------------------
  Purpose  : This files contains functions for reading and writing
             to the STM8S UART.
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
#include <stdio.h>
#include "delay.h"
#include "uart.h"
#include "ring_buffer.h"

// buffers for use with the ring buffer (belong to the USART)
bool     ovf_buf_in; // true = input buffer overflow
uint16_t isr_cnt = 0;

struct ring_buffer ring_buffer_out;
struct ring_buffer ring_buffer_in;
uint8_t            out_buffer[TX_BUF_SIZE];
uint8_t            in_buffer[RX_BUF_SIZE];

//-----------------------------------------------------------------------------
// UART Transmit complete Interrupt.
//
// This interrupt will be executed when the TXE (Transmit Data Register Empty)
// bit in UART1_SR is set. The TXE bit is set by hardware when the contents of 
// the TDR register has been transferred into the shift register. An interrupt 
// is generated if the TIEN bit =1 in the UART_CR1 register. It is cleared by a
// write to the UART_DR register.
//-----------------------------------------------------------------------------
#pragma vector=UART1_T_TXE_vector
__interrupt void UART_TX_IRQHandler()
{
	if (!ring_buffer_is_empty(&ring_buffer_out))
	{   // if there is data in the ring buffer, fetch it and send it
		UART1_DR = ring_buffer_get(&ring_buffer_out);
	} // if
    else
    {   // no more data to send, turn off interrupt
        UART1_CR2_TIEN = 0;
    } // else
} /* UART_TX_IRQHandler() */

//-----------------------------------------------------------------------------
// UART Receive Complete Interrupt.

// This interrupt will be executed when the RXNE (Read Data-Register Not Empty)
// bit in UART1_SR is set. This bit is set by hardware when the contents of the 
// RDR shift register has been transferred to the UART1_DR register. An interrupt 
// is generated if RIEN=1 in the UART1_CR2 register. It is cleared by a read to 
// the UART1_DR register. It can also be cleared by writing 0.
//-----------------------------------------------------------------------------
#pragma vector=UART1_R_RXNE_vector
__interrupt void UART_RX_IRQHandler(void)
{
	volatile uint8_t ch;
	
	if (!ring_buffer_is_full(&ring_buffer_in))
	{
		ring_buffer_put(&ring_buffer_in, UART1_DR);
		ovf_buf_in = false;
	} // if
	else
	{
            ch    = UART1_SR; // Clear IDLE and Overrun errors
            ch    = UART1_DR;
	} // else
	isr_cnt++;
} /* UART_RX_IRQHandler() */

/*------------------------------------------------------------------
  Purpose  : This function initializes the UART to BAUDRATE (uart.h).
             Master clock is 24 MHz, baud-rate is 115200 Baud.
  Variables: clk: which clock is active: HSI (0xE1), HSE (0xB4) or LSI (0xD2)
  Returns  : -
  ------------------------------------------------------------------*/
void uart_init(uint8_t clk)
{
    //  Clear the Idle Line Detected bit in the status register by a read
    //  to the UART1_SR register followed by a Read to the UART1_DR register.
    UART1_SR; // read from status register
    UART1_DR; // Read from data register

    //  Reset the UART registers to the reset values.
    UART1_CR1  = 0;
    UART1_CR2  = 0;
    UART1_CR4  = 0;
    UART1_CR3  = 0;
    UART1_CR5  = 0;
    UART1_GTR  = 0;
    UART1_PSCR = 0;

    // initialize the in and out buffer for the UART
    ring_buffer_out = ring_buffer_init(out_buffer, TX_BUF_SIZE);
    ring_buffer_in  = ring_buffer_init(in_buffer , RX_BUF_SIZE);

    //  Now setup the port to BAUDRATE,N,8,1.
    UART1_CR1_M    = 0;     //  8 Data bits.
    UART1_CR1_PCEN = 0;     //  Disable parity.
    UART1_CR3_STOP = 0;     //  1 stop bit.
    if (clk == HSE)
    {   // external 24 MHz HSE oscillator
        UART1_BRR2 = UART1BRR2_HSE; // Set BRR2 register first
        UART1_BRR1 = UART1BRR1_HSE;
    } // if
    else
    {   // internal 16 MHz HSI oscillator
        UART1_BRR2 = UART1BRR2_HSI; // Set BRR2 register first
        UART1_BRR1 = UART1BRR1_HSI;
    } // else

    //  Disable the transmitter and receiver.
    UART1_CR2_TEN = 0;      //  Disable transmit.
    UART1_CR2_REN = 0;      //  Disable receive.

    //  Set the clock polarity, clock phase and last bit clock pulse.
    UART1_CR3_CPOL = 0;
    UART1_CR3_CPHA = 0;
    UART1_CR3_LBCL = 0;

    //  Turn on the UART transmit, receive and the UART clock.
    UART1_CR2_TIEN = 1; // Enable Transmit interrupt
    UART1_CR2_RIEN = 1; // Enable Receive interrupt
    UART1_CR2_TEN  = 1; // Enable Transmitter
    UART1_CR2_REN  = 1; // Enable Receiver
    UART1_CR3_CKEN = 0; // set to 0 or receive will not work!!
} // uart_init()

/*------------------------------------------------------------------
  Purpose  : This function writes one data-byte to the uart.	
  Variables: ch: the byte to send to the uart.
  Returns  : -
  ------------------------------------------------------------------*/
void uart_putc(uint8_t ch)
{    
    // At 19200 Baud, sending 1 byte takes a max. of 0.09 msec.
    while (ring_buffer_is_full(&ring_buffer_out)) delay_msec(1);
    __disable_interrupt(); // Disable interrupts to get exclusive access to ring_buffer_out
    if (ring_buffer_is_empty(&ring_buffer_out))
    {
        UART1_CR2_TIEN = 1; // First data in buffer, enable data ready interrupt
    } // if
    ring_buffer_put(&ring_buffer_out, ch); // Put data in buffer
    __enable_interrupt(); // Re-enable interrupts
} // uart_putc()

/*------------------------------------------------------------------
  Purpose  : This function writes a string to the UART, using
             the uart_putc() routine.
  Variables:
         s : The string to write to serial port 0
  Returns  : the number of characters written
  ------------------------------------------------------------------*/
void uart_printf(char *s)
{
    char *ch = s;
    while (*ch)
    {
        if (*ch == '\n')
        {
            uart_putc('\r'); // add CR
        } // if
        uart_putc(*ch);
        ch++;                //  Grab the next character.
    } // while
} // uart_printf()

/*------------------------------------------------------------------
  Purpose  : This function checks if a character is present in the
             receive buffer.
  Variables: -
  Returns  : 1 if a character is received, 0 otherwise
  ------------------------------------------------------------------*/
bool uart_kbhit(void)
{
    return !ring_buffer_is_empty(&ring_buffer_in);
} // uart_kbhit()

/*------------------------------------------------------------------
  Purpose  : This function reads one data-byte from the uart.	
  Variables: -
  Returns  : the data-byte read from the uart
  ------------------------------------------------------------------*/
uint8_t uart_getc(void)
{
    return ring_buffer_get(&ring_buffer_in);
} // uart_getch()

