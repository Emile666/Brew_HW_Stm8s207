/*==================================================================
  File Name    : max7219.h
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This file is the header-file for max7219.c and contains
            the routines to write to the MAX7219.
  ------------------------------------------------------------------
  This is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This file is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this file. If not, see <http://www.gnu.org/licenses/>.
  ==================================================================*/
#ifndef _MAX7219_H_
#define _MAX7219_H_
#include <stdint.h>
      
//--------------------------------------------
// Defines for the MAX7219 registers (D15-D08)
//--------------------------------------------
#define MAX7219_NO_OP (0x0000)
#define MAX7219_DIG0  (0x0100)
#define MAX7219_DIG1  (0x0200)
#define MAX7219_DIG2  (0x0300)
#define MAX7219_DIG3  (0x0400)
#define MAX7219_DIG4  (0x0500)
#define MAX7219_DIG5  (0x0600)
#define MAX7219_DIG6  (0x0700)
#define MAX7219_DIG7  (0x0800)
#define MAX7219_DECD  (0x0900)
#define MAX7219_INTNS (0x0A00)
#define MAX7219_SCANL (0x0B00)
#define MAX7219_SHUTD (0x0C00)
#define MAX7219_TEST  (0x0F00)

void max7219_init(void);
void max7219_write(uint16_t dat);

#endif
