#ifndef _STM8_HW_INIT_H
#define _STM8_HW_INIT_H
/*===================================================================================
  File Name: stm8_hw_init.h
  Author   : Emile
  ----------------------------------------------------------------------------------
  Purpose  : this is the header file for stm8_hw_init.c.
             It is based on hardware PCB version v4.01.
  ----------------------------------------------------------------------------------
  This file is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this software. If not, see <http://www.gnu.org/licenses/>.
  ----------------------------------------------------------------------------------
           Schematic of the connections to the STM8S207R8 MCU.
                                
      MCU pin-name            Function    |    MCU pin-name        Function
   ---------------------------------------|-----------------------------------------
   01 NRST                    <NRST>      | 64 PD7/TLI                -
   02 PA1/OSCIN               XTAL 24 MHz | 63 PD6/UART3_RX           RX
   03 PA2/OSCOUT              XTAL 24 MHz | 62 PD5/UART3_TX           TX
   04 VSSIO_1                 GND         | 61 PD4(HS)/TIM2_CH1[BEEP] BUZZER
   05 VSS                     GND         | 60 PD3(HS)/TIM2_CH2       BK_SSR3
   06 VCAP                    <VCAP>      | 59 PD2(HS)/TIM3_CH1       BK_SSR2
   07 VDD                     +5V         | 58 PD1(HS)/SWIM           SWIM
   08 VDDIO_1                 +5V         | 57 PD0(HS)/TIM3_CH2       BK_SSR1
   09 PA3/TIM2_CH3[TIME3_CH1] -           | 56 PE0(HS)/CLK_CCO        FLOW2
   10 PA4(HS)/UART1_RX        RX_DBG      | 55 PE1/I2C_SCL            SCL0
   11 PA5(HS)/UART1_TX        TX_DBG      | 54 PE2/I2C_SDA            SDA0
   12 PA6(HS)/UART1_CK        FLOW1       | 53 PE3/TIM1_BKIN          SCL1
   13 PF7/AIN15               HLT_SSR3    | 52 PE4                    SDA1
   14 PF6/AIN14               HLT_SSR2    | 51 PG7                    SDA2
   15 PF5/AIN13               PUMP_230V   | 50 PG6                    SCL2
   16 PF4/AIN12               PUMP2_230V  | 49 PG5                    BG_LED 
   ---------------------------------------|-----------------------------------------
   17 PF3/AIN11               HLT_SSR1    | 48 PI0                    SPI_RDY
   18 VREF+                   +5V filt.   | 47 PG4                    IRQ_LED
   19 VDDA                    +5V         | 46 PG3                    ALIVE_LED_B
   20 VSSA                    GND         | 45 PG2                    ALIVE_LED_G
   21 VREF-                   GND         | 44 PG1/CAN_RX             ALIVE_LED_R
   22 PF0/AIN10               LM35        | 43 PG0/CAN_TX             SPI_RESET
   23 PB7/AIN7                VALVE8      | 42 PC7(HS)/SPI_MISO       SPI_MISO
   24 PB6/AIN6                VALVE7      | 41 PC6(HS)/SPI_MOSI       SPI_MOSI
   25 PB5/AIN5                VALVE6      | 40 VDDIO_2                +5V
   26 PB4/AIN4                VALVE5      | 39 VSSIO_2                GND
   27 PB3/AIN3                VALVE4      | 38 PC5(HS)/SPI_SCK        SPI_CLK
   28 PB2/AIN2                VALVE3      | 37 PC4(HS)/TIM1_CH4       BK_230V
   29 PB1/AIN1                VALVE2      | 36 PC3(HS)/TIM1_CH3       HLT_230V
   30 PB0/AIN0                VALVE1      | 35 PC2(HS)/TIM1_CH2       BK_PWM
   31 PE7/AIN8                FLOW3       | 34 PC1(HS)/TIM1_CH1       HLT_PWM
   32 PE6/AIN9                FLOW4       | 33 PE5/SPI_NSS            SPI_SS
   --------------------------------------------------------------------------------
   NOTE  : PORTF and PORTG pins do NOT have interrupt capability!
=================================================================================== */
#include <iostm8s207r8.h>
#include <stdint.h>
#include <stdbool.h>
#include <intrinsics.h> 

//-----------------------------
// PORT A defines
//-----------------------------
#define FLOW1       (0x40) /* PA6 */
#define TX_DBG      (0x20) /* PA5 UART 1 */       
#define RX_DBG      (0x10) /* PA4 UART 1 */       
#define PA3         (0x08) /* FREE */

#define FLOW1b      (PA_IDR_IDR6)

//-----------------------------
// PORT B defines
//-----------------------------
#define VALVE8      (0x80)  /* PB7 */
#define VALVE7      (0x40)  /* PB6 */
#define VALVE6      (0x20)  /* PB5 */
#define VALVE5      (0x10)  /* PB4 */
#define VALVE4      (0x08)  /* PB3 */
#define VALVE3      (0x04)  /* PB2 */
#define VALVE2      (0x02)  /* PB1 */
#define VALVE1      (0x01)  /* PB0 */
#define VALVES      (0xFF)  /* PB7-PB0, valves 8-1 */

//-----------------------------
// PORT C defines
//-----------------------------
#define SPI_MISO    (0x80) /* PC7, controlled by SPI */
#define SPI_MOSI    (0x40) /* PC6, controlled by SPI */
#define SPI_CLK     (0x20) /* PC5, controlled by SPI */
#define BOIL_230V   (0x10) /* PC4 On/Off control modulating gas-valve 230V */
#define HLT_230V    (0x08) /* PC3 On/Off control modulating HLT gas-valve 230V */
#define BK_PWM      (0x04) /* PC2/TIM1_CH2 PWM output for Boil-kettle */       
#define HLT_PWM     (0x02) /* PC1/TIM1_CH1 PWM output for HLT */       

// use these defines to directly control the output-pins
#define BOIL_230Vb  (PC_ODR_ODR4) /* TODO PC4 On/Off control modulating gas-valve 230V */
#define HLT_230Vb   (PC_ODR_ODR3) /* TODO PC3 On/Off control modulating HLT gas-valve 230V */
      
//-----------------------------
// PORT D defines
//-----------------------------
#define PD7         (0x80) /* PD7 FREE*/
#define RX          (0x40) /* PD6 UART 3 */
#define TX          (0x20) /* PD5 UART 3 */
#define BUZZER      (0x10) /* PD4 */
#define BK_SSR3     (0x08) /* Boil-kettle slow SSR output 3, electric heating */
#define BK_SSR2     (0x04) /* Boil-kettle slow SSR output 2, electric heating */
#define SWIM        (0x02) /* PD1 SWIM */
#define BK_SSR1     (0x01) /* Boil-kettle slow SSR output 1, electric heating */

#define BK_SSR_ALL  (BK_SSR3 | BK_SSR2 | BK_SSR1)
// use these defines to directly control the output-pins
#define BK_SSR3b    (PD_ODR_ODR3) 
#define BK_SSR2b    (PD_ODR_ODR2) 
#define BK_SSR1b    (PD_ODR_ODR0) 

//-----------------------------
// PORT E defines
//-----------------------------
#define FLOW3       (0x80) /* PE7 */
#define FLOW4       (0x40) /* PE6 */
#define SPI_SS      (0x20) /* PE5 chip-select for Wiz550io */
#define SDA1        (0x10) /* PE4 SDA1 */
#define SCL1        (0x08) /* PE3 SCL1 */
#define SDA0        (0x04) /* PE2 SDA0 */
#define SCL0        (0x02) /* PE1 SCL0 */
#define FLOW2       (0x01) /* PE0 */

#define SPI_SSb     (PE_ODR_ODR5)
#define FLOW3b      (PE_IDR_IDR7)
#define FLOW4b      (PE_IDR_IDR6)
#define FLOW2b      (PE_IDR_IDR0)

//-----------------------------
// PORT F defines
//-----------------------------
#define HLT_SSR3    (0x80) /* HLT slow SSR output 3, electric heating */
#define HLT_SSR2    (0x40) /* HLT slow SSR output 2, electric heating */
#define PUMP_230V   (0x20) /* PF5, Main pump Triac/SSR */       
#define PUMP2_230V  (0x10) /* PF4, HLT-pump Triac/SSR */       
#define HLT_SSR1    (0x08) /* HLT slow SSR output 1, electric heating */       
#define LM35        (0x01) /* PF0, ADC-input for LM35 */       

#define HLT_SSR_ALL (HLT_SSR3 | HLT_SSR2 | HLT_SSR1)

#define LM35_ADC_CH (10)   /* PF0/AIN10 */
#define HLT_SSR3b   (PF_ODR_ODR7)
#define HLT_SSR2b   (PF_ODR_ODR6)
#define PUMP_230Vb  (PF_ODR_ODR5)
#define PUMP2_230Vb (PF_ODR_ODR4)
#define HLT_SSR1b   (PF_ODR_ODR3)
       
//-----------------------------
// PORT G defines
//-----------------------------
#define SDA2        (0x80) /* PG7 */       
#define SCL2        (0x40) /* PG6 */       
#define BG_LED      (0x20) /* PG5, shows background process status */       
#define IRQ_LED     (0x10) /* PG4, shows interrupt status */       
#define ALIVE_LED_B (0x08) /* PG3 */
#define ALIVE_LED_G (0x04) /* PG2 */
#define ALIVE_LED_R (0x02) /* PG1 */
#define ALIVE_LEDS  (ALIVE_LED_B | ALIVE_LED_G | ALIVE_LED_R)
#define SPI_NRESET  (0x01) /* PG0, hardware reset for WIZ550io module */

// use these defines to directly control the output-pins
#define BG_LEDb      (PG_ODR_ODR5)
#define IRQ_LEDb     (PG_ODR_ODR4)
#define ALIVE_LED_Bb (PG_ODR_ODR3)
#define ALIVE_LED_Gb (PG_ODR_ODR2)
#define ALIVE_LED_Rb (PG_ODR_ODR1)
#define SPI_NRESETb  (PG_ODR_ODR0)

//-----------------------------
// Buzer STD modes
//-----------------------------
#define BZ_OFF   (0)
#define BZ_ON    (1)
#define BZ_ON2   (2)
#define BZ_BURST (3)
#define BZ_SHORT (4)

#define FREQ_1KHZ (0)
#define FREQ_2KHZ (1)
#define FREQ_4KHZ (2)

//-----------------------------
// Defines for system clock
//-----------------------------
#define HSI (0xE1) /* internal 16 MHz oscillator */
#define LSI (0xD2) /* internal 128 kHz oscillator */
#define HSE (0xB4) /* external 24 MHz oscillator */

// Function prototypes
void     buzzer(void);
uint8_t  initialise_system_clock(uint8_t clk);
void     setup_timers(uint8_t clk);
void     setup_gpio_ports(void);
#endif // _STM8_HW_INIT_H