#ifndef STM8_ADC_H
#define STM8_ADC_H
/*==================================================================
  File Name: adc.h
  Author   : Emile
  ------------------------------------------------------------------
  Purpose  : This is the header-file for adc.c
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

// LM35 is connected to ADC-channel PF0/AIN10
#define AD_LM35 (10)
#define ADC_AVG (16)

// Function prototypes
uint16_t read_adc(uint8_t ch);
#endif