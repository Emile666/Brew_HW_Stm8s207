#ifndef _COMMAND_INTERPRETER_H
#define _COMMAND_INTERPRETER_H
/*==================================================================
  File Name: command_interpreter.h
  Author   : Emile
  ------------------------------------------------------------------
  Purpose  : This is the header-file for command_interpreter.c
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
#include "scheduler.h"
#include "i2c_bb.h"
      
//------------------------------------
// Defines for UART/ETH communications
//------------------------------------
#define RS232_USB     (true)
#define ETHERNET_UDP  (false)
#define EBREW_PORT_NR (8888)

void    i2c_scan(enum I2C_CH ch, bool rs232_udp);
uint8_t rs232_command_handler(void);
uint8_t ethernet_command_handler(char *s);
uint8_t set_parameter(uint8_t num, uint16_t val);
void    find_OW_device(enum I2C_CH ch, uint8_t i2c_addr);
void    process_pwm_signal(uint8_t pwm_ch, uint8_t pwm_val, uint8_t enable);
void    process_temperature(uint8_t err, char *name, int16_t val_87);
void    process_flow(uint32_t flow_val, char *name);
void    list_all_tasks(bool rs232_udp);
uint8_t execute_single_command(char *s, bool rs232_udp);

#endif
