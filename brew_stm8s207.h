#ifndef _BREW_STM8S207_H_
#define _BREW_STM8S207_H_
/*==================================================================
  File Name: brew_stm8s207.h
  Author   : Emile
  ------------------------------------------------------------------
  Purpose  : This is the header-file for brew_stm8s207.c
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
#include <string.h>
#include "delay.h"         /* for delay_msec() */
#include "adc.h"
#include "i2c_bb.h"
#include "pwm.h"
#include "spi.h"
#include "uart.h"
#include "command_interpreter.h"
#include "misc.h"
#include "scheduler.h"
#include "eep.h"

//-----------------------------
// E-brew System Mode
//-----------------------------
#define GAS_MODULATING     (0) /* Modulating gas-valve */
#define GAS_NON_MODULATING (1) /* Non-modulating gas-valve */
#define ELECTRICAL_HEATING (2) /* Electrical heating */

//-----------------------------
// Delayed Start STD modes
//-----------------------------
#define DEL_START_INIT   (0)
#define DEL_START_TMR    (1)
#define DEL_START_BURN   (2)
#define DEL_START_MAX_DELAY_TIME (54000) /* Max. time is 30 hours * 60 minutes * 30 * 2 seconds */
#define DEL_START_MAX_BURN_TIME   (3600) /* Max. time is 120 minutes * 30 * 2 seconds */

//-----------------------------
// pwm_2_time() States
//-----------------------------
#define IDLE       (0)
#define EL_HTR_OFF (1)
#define EL_HTR_ON  (2)

// INIT_TEMP  20 °C
// INIT_VOL10 80 E-1 L
// LM35_CONV: 11000 E-2 °C / 1023
#define INIT_TEMP  (20)
#define INIT_VOL10 (80)
#define LM35_CONV  (10.75268817)

//---------------------------------------------------
// Q = 5.5 Hz for 1 L, 11 pulses per L per sec.
// Per minute: 5.5 x 1 rising pulse x 60 sec. = 330
// An interrupt is generated for a rising AND
// falling edge, but only the rising edge is counted.
//---------------------------------------------------
#define FLOW_PER_L     (330)
#define FLOW_ROUND_OFF (FLOW_PER_L>>1)

void    print_ebrew_revision(char *ver);
uint8_t init_WIZ550IO_module(void);
void    print_IP_address(uint8_t *ip);

#endif /* _BREW_STM8S207_H_ */