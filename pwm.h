#ifndef _PWM_H_
#define _PWM_H_
/*==================================================================
  File Name: pwm.h
  Author   : Emile
  ------------------------------------------------------------------
  Purpose  : Header file for STM8 PWM routines
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

#define PWM_HLT (0)
#define PWM_BK  (1)

void pwm_write(uint8_t pwm_ch, uint8_t duty_cycle); // duty_cyle varies 0 % and 100 %

#endif /* _PWM_H_ */