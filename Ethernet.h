#ifndef ETHERNET5500_H
#define ETHERNET5500_H
/*==================================================================
  File Name: Ethernet.h
  Author   : Soohwan Kim (suhwan@wiznet.co.kr), mod. 12 Aug 2013,
             C-version by Emile
  ------------------------------------------------------------------
  Purpose  : This is the header-file for Ethernet.c
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
#include "Dhcp.h"
#include <string.h>
#include <stdio.h>

// Initialize function when use the ioShield series (included WIZ550io)
// WIZ550io has a MAC address which is written after reset.
// Default IP, Gateway and subnet address are also written.
// so, It needs some initial time. please refer WIZ550io Datasheet for details.
int  Ethernet_begin(void);
void Ethernet_begin_ip(uint8_t *local_ip);
int  Ethernet_maintain(void);

#endif
