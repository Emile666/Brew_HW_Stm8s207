/*==================================================================
  File Name: command_interpreter.c
  Author   : Emile
  ------------------------------------------------------------------
  Purpose  : This files contains command handling from both UART and
             Ethernet.
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
  along with this software. If not, see <http://www.gnu.org/licenses/>.
  ================================================================== */ 
#include <ctype.h>
#include "command_interpreter.h"
#include "brew_stm8s207.h"
#include "misc.h"
#include "Udp.h"
#include "one_wire.h"
#include "eep.h"
#include "uart.h"
#include "pwm.h"
#include "scheduler.h"
      
extern uint8_t      remoteIP[4]; // remote IP address for the incoming packet whilst it's being processed
extern uint8_t      ROM_NO[];    // One-wire hex-address
extern uint8_t      crc8;

extern bool       ethernet_WIZ550i;
extern uint8_t    local_ip[4];         // local IP address received from dhcp_begin()
extern const char *ebrew_revision;     // ebrew CVS revision number

extern uint8_t    hlt_elec1_pwm;       // PWM signal (0-100 %) for HLT Electric heating-element 1
extern uint8_t    hlt_elec2_pwm;       // PWM signal (0-100 %) for HLT Electric heating-element 2
extern pwmtime    pwmhlt1;             // Struct for HLT Electrical Heater 1 Slow SSR signal
extern pwmtime    pwmhlt2;             // Struct for HLT Electrical Heater 1 Slow SSR signal

extern uint8_t    bk_elec1_pwm;        // PWM signal (0-100 %) for Boil-kettle Electric heating-element 1
extern uint8_t    bk_elec2_pwm;        // PWM signal (0-100 %) for Boil-kettle Electric heating-element 2
extern pwmtime    pwmbk1;              // Struct for Boil-kettle Electrical Heater 1 Slow SSR signal
extern pwmtime    pwmbk2;              // Struct for Boil-kettle Electrical Heater 1 Slow SSR signal

extern uint16_t   lm35_temp;           // LM35 Temperature in E-2 °C

//------------------------------------------------------
// The extension _87 indicates a signed Q8.7 format!
// This is used for all temperature sensors
//------------------------------------------------------
extern int16_t    thlt_temp_87;        // THLT Temperature in °C * 128
extern uint8_t    thlt_err;			   // 1 = error reading sensor

extern int16_t    tmlt_temp_87;        // TMLT Temperature in °C * 128
extern uint8_t    tmlt_err;            // 1 = error reading sensor

extern int16_t    tcfc_temp_87;        // TCFC Temperature in °C * 128
extern uint8_t    tcfc_err;            // 1 = error reading sensor

extern int16_t    tboil_temp_87;       // TBOIL Temperature in °C * 128
extern uint8_t    tboil_err;           // 1 = error reading sensor

extern int16_t    thlt_ow_87;          // THLT Temperature in °C * 128
extern uint8_t    thlt_ow_err;         // 1 = error reading sensor

extern int16_t    tmlt_ow_87;          // TMLT Temperature in °C * 128
extern uint8_t    tmlt_ow_err;         // 1 = error reading sensor

extern uint32_t   flow_hlt_mlt;     // Count from flow-sensor between HLT and MLT
extern uint32_t   flow_mlt_boil;    // Count from flow-sensor between MLT and boil-kettle
extern uint32_t   flow_cfc_out;     // Count from flow-sensor at output of CFC
extern uint32_t   flow4;            // Count from FLOW4 (future use)

extern bool       bz_on;      // true = buzzer-on
extern uint8_t    bz_rpt_max; // number of buzzer repeats

extern bool     delayed_start_enable;  // true = delayed start is enabled
extern uint16_t delayed_start_time;    // delayed start time in 2 sec. counts
extern uint16_t delayed_start_timer1;  // timer to countdown until delayed start

extern  task_struct task_list[];      // struct with all tasks
extern  uint8_t     max_tasks;

char    rs232_inbuf[UART_BUFLEN];     // buffer for RS232 commands
uint8_t rs232_ptr = 0;                // index in RS232 buffer

/*-----------------------------------------------------------------------------
  Purpose  : helper function to print to either RS232/USB or Ethernet/Udp
  Variables: 
  rs232_udp: [RS232_USB, ETHERNET_UDP] Response via RS232/USB or Ethernet/Udp
          s: the string to print
 Returns  : -
  ---------------------------------------------------------------------------*/
void pr(bool rs232_udp, char *s)
{
    if (rs232_udp == RS232_USB) 
         uart_printf(s);
    else udp_write((uint8_t *)s,strlen(s));
} // pr

/*-----------------------------------------------------------------------------
  Purpose  : Scan all devices on the I2C bus on all channels of the PCA9544
  Variables: 
         ch: the I2C channel number, 0 is the main channel
  rs232_udp: [RS232_USB, ETHERNET_UDP] Response via RS232/USB or Ethernet/Udp
 Returns  : -
  ---------------------------------------------------------------------------*/
void i2c_scan(enum I2C_CH ch, bool rs232_udp)
{
    char    s[50]; // needed for printing to serial terminal
    uint8_t x = 0;
    int     i;     // Leave this as an int!
    
    sprintf(s,"I2C[%1d]: ",ch);
    pr(rs232_udp,s); // print to UART or ETH
    for (i = 0x02; i < 0xff; i+=2)
    {
        if (i2c_start_bb(ch,i) == I2C_ACK)
        {
            sprintf(s,"0x%0x ",i);
            pr(rs232_udp,s); // print to UART or ETH
            x++;
        } // if
        i2c_stop_bb(ch);
    } // for
    if (!x) 
    {
        pr(rs232_udp,"-"); // print to UART or ETH
    } // if	
    pr(rs232_udp,"\n");
} // i2c_scan()

/*-----------------------------------------------------------------------------
  Purpose  : Non-blocking RS232 command-handler via the USB port
  Variables: -
  Returns  : [NO_ERR, ERR_CMD, ERR_NUM, ERR_I2C]
  ---------------------------------------------------------------------------*/
uint8_t rs232_command_handler(void)
{
  char    ch;
  static uint8_t cmd_rcvd = 0;
  
  if (!cmd_rcvd && uart_kbhit())
  { // A new character has been received
    ch = tolower(uart_getc()); // get character as lowercase
	switch (ch)
	{
            case '\r': break;
            case '\n': cmd_rcvd  = 1;
                       rs232_inbuf[rs232_ptr] = '\0';
                       rs232_ptr = 0;
                       break;
            default  : if (rs232_ptr < UART_BUFLEN-1)
			    rs232_inbuf[rs232_ptr++] = ch;
		       else rs232_ptr = 0; // remove inputs	
                       break;
	} // switch
  } // if
  if (cmd_rcvd)
  {
      cmd_rcvd = 0;
      return execute_single_command(rs232_inbuf, RS232_USB);
  } // if
  else return NO_ERR;
} // rs232_command_handler()

/*-----------------------------------------------------------------------------
  Purpose  : Non-blocking command-handler via the Ethernet UDP port
  Variables: -
  Returns  : [NO_ERR, ERR_CMD, ERR_NUM, ERR_I2C]
  ---------------------------------------------------------------------------*/
uint8_t ethernet_command_handler(char *s)
{
  uint8_t rval = NO_ERR;
  char    *s1;
  uint8_t i;
  
  s1 = strtok(s,"\n"); // get the first command
  while (s1 != NULL)
  {   // walk through other commands
      for (i = 0; i < strlen(s1); i++) s1[i] = tolower(s1[i]);
      //sprintf(s2,"eth%1d=[%s]\n",cnt++,s1); 
      //uart_printf(s2);
      rval = execute_single_command(s1, ETHERNET_UDP);
      s1 = strtok(NULL, "\n");
  } // while
  return rval;
} // ethernet_command_handler()

/*-----------------------------------------------------------------------------
  Purpose  : Finds a One-Wire device
  Variables: i2c_addr: I2C-address of DS2482 bridge
  Returns  : -
  ---------------------------------------------------------------------------*/
void find_OW_device(enum I2C_CH ch, uint8_t i2c_addr)
{
    char    s2[40]; // Used for printing to RS232 port
    uint8_t i,rval;
    
    rval = OW_first(ch, i2c_addr); // Find ROM ID of first DS18B20
    if (rval) 
    {
        for (i= 0; i < 8; i++)
        {
            sprintf(s2,"%02X ",ROM_NO[i]); // global array
            uart_printf(s2);
        } // for								 
    } // if
    else uart_printf("-");
    uart_printf("\n");							 
} // find_OW_device()

/*-----------------------------------------------------------------------------
  Purpose  : Process PWM signal for all system-modes. 
             Executed when a Bxxx or Hxxx command is received.
             MODULATING GAS BURNER:     the 230V-signal energizes the gas-valve 
					and the PWM signal is generated by a timer.
	     ELECTRICAL HEATING:        The 230V signal for the SSR/Triac is switched 
                                        with a 5 sec. period, the PWM signal is 
					converted into a time-division signal of 
					100 * 50 msec.
Variables: 
     pwm_ch: [PWM_BK,PWM_HLT]. Selects the PWM channel (Boil-kettle or HLT)
    pwm_val: the PWM signal [0%..100%]
  Returns  : -
  ---------------------------------------------------------------------------*/
void process_pwm_signal(uint8_t pwm_ch, uint8_t pwm_val, uint8_t enable)
{
    //----------------------------------------------------------------
    // Modulating Gas-burner: create an 230V ON/OFF signal to enable
    // the gas-burner and generate a 25 kHz PWM signal with a timer.
    //----------------------------------------------------------------
    if (enable & GAS_MODU)
    {	// Modulating gas-burner is enabled
        if (pwm_ch == PWM_BK)
             BOIL_230Vb = true;
        else HLT_230Vb  = true;
    } // if	
    else 
    {   // Modulating gas-burner is disabled
        if (pwm_ch == PWM_BK) 
             BOIL_230Vb = false;
        else HLT_230Vb  = false;
    } // else
    pwm_write(pwm_ch, pwm_val);  // write PWM value to Timer 1 and 3 channels
    
    //----------------------------------------------------------------
    // Electric Heating: send the PWM signal as a SLOW SSR signal,
    // with T = 5 sec., to the heating-elements.
    //----------------------------------------------------------------
    if (pwm_ch == PWM_HLT)
    {	// Currently, only the HLT has 2 electric heating-elements
            if (enable & ELEC_HTR1)
            {   // HLT Electric heating-element 1
                    hlt_elec1_pwm = pwm_val; // set value for pwm_task() / pwm_2_time()
            } 	// if
            else hlt_elec1_pwm = 0; // disable electric heater
            if (enable & ELEC_HTR2)
            {   // HLT Electric heating-element 2
                    hlt_elec2_pwm = pwm_val; // set value for pwm_task() / pwm_2_time()
            } 	// if
            else hlt_elec2_pwm = 0; // disable electric heater
    } // if
    else 
    { // Boil-kettle: support here for 2 electric heating elements
            if (enable & ELEC_HTR1)
            {   // BK Electric heating-element 1
                    bk_elec1_pwm = pwm_val; // set value for pwm_task() / pwm_2_time()
            } 	// if
            else bk_elec1_pwm = 0; // disable electric heater
            if (enable & ELEC_HTR2)
            {   // BK Electric heating-element 2
                    bk_elec2_pwm = pwm_val; // set value for pwm_task() / pwm_2_time()
            } 	// if
            else bk_elec2_pwm = 0; // disable electric heater
    } // else		    
} // process_pwm_signal()

/*-----------------------------------------------------------------------------
  Purpose  : Builds a string that can be returned via RS232/Ethernet.
  Variables:
		err: 1 = error when reading the temperature sensor
       name: the string with the sensor name. Also used to return the result.
	 val_87: the actual value of the temperature sensor in Q8.7 format
  Returns  : -
  ---------------------------------------------------------------------------*/
void process_temperatures(uint8_t err, char *name, int16_t val_87, uint8_t last)
{
   uint16_t temp, frac_16;
   char     s2[20];
   
   if (err)
   {   // error
       if (last) strcat(name,"-99.9\n");
       else      strcat(name,"-99.9,");
   }
   else
   {
       temp     = val_87 >> 7;     // The integer part of the sensor value
       frac_16  = val_87 & 0x007f; // The fractional part of the sensor value
       frac_16 *= 25;              // 100 / 128 = 25 / 32
       frac_16 += 16;              // 0.5 for rounding
       frac_16 >>= 5;              // SHR 5 = divide by 32
       if (last) sprintf(s2,"%d.%02d\n",temp,frac_16);
       else      sprintf(s2,"%d.%02d," ,temp,frac_16);
       strcat(name,s2);            // store result back in *name
   } // else
} // process_temperatures()

/*-----------------------------------------------------------------------------
  Purpose  : Builds a string that can be returned via RS232/Ethernet.
  Variables:
  	 flownr: [1..4] Number of flowsensor
       name: the string with the sensor name. Also used to return the result.
  Returns  : -
  ---------------------------------------------------------------------------*/
void process_flows(uint32_t flow_val, char *name, uint8_t last)
{
   uint16_t temp, frac_16;
   char     s2[20];
   
   temp     = (uint16_t)((flow_val * 100 + FLOW_ROUND_OFF) / FLOW_PER_L); // Flow in E-2 Litres
   frac_16  = temp / 100;
   frac_16  = temp - 100 * frac_16;
   if (last)
        sprintf(s2,"%d.%02d\n",temp/100,frac_16);
   else sprintf(s2,"%d.%02d," ,temp/100,frac_16);
   strcat(name,s2);  // store result back in *name
} // process_flows()

/*-----------------------------------------------------------------------------
  Purpose  : list all tasks and send result to the UART or ETH
  Variables: -
 Returns   : -
  ---------------------------------------------------------------------------*/
void list_all_tasks(bool rs232_udp)
{
    uint8_t index = 0;
    char    s[50];
    
    //uart_printf("Task-Name,T(ms),Stat,T(ms),M(ms)\n");
    //go through the active tasks
    if(task_list[index].Period != 0)
    {
        while ((index < MAX_TASKS) && (task_list[index].Period != 0))
        {
            sprintf(s,"%s,%d,%x,%d,%d\n", task_list[index].Name, 
                    task_list[index].Period  , task_list[index].Status, 
                    task_list[index].Duration, task_list[index].Duration_Max);
            pr(rs232_udp,s); // print to USB or ETH
            index++;
        } // while
    } // if
} // list_all_tasks()

/*-----------------------------------------------------------------------------
  Purpose: interpret commands which are received via the USB serial terminal:
   - A0           : Read Temperature sensors: THLT / TMLT / TBOIL / TCFC
   - A9           : Read flow sensors: HLT->MLT, MLT->BOIL, CFC-out, Flow4
   - B0...B100    : PID-output for Boil-kettle, needed for:
                    - PWM output for modulating gas-valve (N0=0)
                    - Time-Division ON/OFF signal for Non-Modulating gas-valve (N0=1)
                    - Time-Division ON/OFF signal for Electrical heating-element (N0=2)
   - D0           : Disable delayed-start
     D1           : Set time in minutes for delayed-start and enable delayed-start
     D2           : Get remaining time in minutes for delayed-start
   - E0 / E1      : Ethernet with WIZ550io Disabled / Enabled
   - H0...H100    : PID-output for HLT, needed for:
                    - PWM output for modulating gas-valve (N0=0)
                    - Time-Division ON/OFF signal for Non-Modulating gas-valve (N0=1)
                    - Time-Division ON/OFF signal for Electrical heating-element (N0=2)
   - L0 / L1      : ALIVE Led OFF / ON
   - P0 / P1      : set Pump OFF / ON
   - R0           : Reset all flows
   - S0           : Ebrew hardware revision number (also disables delayed-start)
     S1		  : List value of parameters that can be set with Nx command
     S2           : List all connected I2C devices  
     S3           : List all tasks
     S4           : List One-Wire devices
   - V0...V255    : Output bits for valves V1 until V8
   - X0...X8      : Sound buzzer x times
 
  Variables: 
          s: the string that contains the command from RS232 serial port 0
  rs232_udp: [RS232_USB, ETHERNET_UDP] Response via RS232/USB or Ethernet/Udp
  Returns  : [NO_ERR, ERR_CMD, ERR_NUM, ERR_I2C] or ack. value for command
  ---------------------------------------------------------------------------*/
uint8_t execute_single_command(char *s, bool rs232_udp)
{
   uint8_t  num  = atoi(&s[1]); // convert number in command (until space is found)
   uint8_t  rval = NO_ERR;
   uint16_t temp;
   char     s2[40]; // Used for printing to RS232 port
   
   switch (s[0])
   {
	   case 'a': // Read Temperatures and Flows
               if (rs232_udp == ETHERNET_UDP) 
               {
                   udp_beginPacketIP(remoteIP, EBREW_PORT_NR); // send response back
               } // if				 
               switch (num)
               {
               case 0: // Temperature Processing, send all Temperatures to PC
                   temp = lm35_temp / 100;
                   sprintf(s2,"T=%d.%02d,",temp,lm35_temp-100*temp);   // LM35
                   process_temperatures(thlt_err,s2,thlt_temp_87,0);   // Thlt-i2c
                   process_temperatures(tmlt_err,s2,tmlt_temp_87,0);   // Tmlt-i2c
                   process_temperatures(tboil_err,s2,tboil_temp_87,0); // Tboil-ow
                   process_temperatures(tcfc_err,s2,tcfc_temp_87,0);   // Tcfc-ow
                   process_temperatures(thlt_ow_err,s2,thlt_ow_87,0);  // Thlt_ow
                   process_temperatures(tmlt_ow_err,s2,tmlt_ow_87,1);  // Tmlt_ow
                   break;
               case 9: // FLOW Processing, send all flows to PC
                   strcpy(s2,"F=");
                   process_flows(flow_hlt_mlt ,s2,0);
                   process_flows(flow_mlt_boil,s2,0);
                   process_flows(flow_cfc_out ,s2,0);
                   process_flows(flow4        ,s2,1);
                   break;
               default: 
                   rval = ERR_NUM;
                   break;
               } // switch
               if (rval != ERR_NUM)
               {		 
                   pr(rs232_udp,s2); // print to UART or ETH
               } // if
               if (rs232_udp == ETHERNET_UDP)
               {
                   udp_endPacket(); // send response
               } // if	
               break;
               
	   case 'b': // PWM signal for Boil-Kettle Modulating Gas-Burner
               temp = atoi(&s[3]); // convert PWM signal to number
               if      (num > 15)   rval = ERR_CMD; // [GAS_MODU,GAS_ONOFF,ELEC_HTR1,ELEC_HTR2]
               else if (temp > 100) rval = ERR_NUM; // error if pwm > 100
               else process_pwm_signal(PWM_BK,temp,num);
               break;
           
	   case 'd': // Delayed-start option
               if (num > 2) rval = ERR_NUM;
               else
               {   // D0 (disable), D1 (set) or D2 (get) command
                   if (num > 1)
                   {   // D2 command: show remaining time until HLT-burner start
                       if (rs232_udp == ETHERNET_UDP)
                       {
                           udp_beginPacketIP(remoteIP, EBREW_PORT_NR); // send response back
                       } // if
                       sprintf(s2,"delayed-start:[%d]%d/%d min.\n",delayed_start_enable,delayed_start_timer1/30,delayed_start_time/30);
                       pr(rs232_udp,s2); // print to UART or ETH
                       if (rs232_udp == ETHERNET_UDP)
                       {
                           udp_endPacket(); // send response
                       } // if
                   } // if
                   else if (num)
                   {   // D1 command: set delayed_start timer
                       if ((s[2] != ' ') || (strlen(s) < 4))
                       {  // check for error in command: 'd1 yy'
                           rval = ERR_CMD;
                       } // if
                       else
                       {   // (s[2] == ' ') and (strlen(s) >= 4)
                           temp = atoi(&s[2]) * 30;        // convert minutes to 2-second time counts
                           if (temp > DEL_START_MAX_DELAY_TIME)
                           {    // limit delayed-start to 30 hours
                               rval = ERR_NUM; 
                           } // if						   
                           else 
                           {
                               delayed_start_time = temp; // delayed-start time
                               eep_write16(EEPARB_DEL_START_TIME,delayed_start_time);
                               eep_write16(EEPARB_DEL_START_TMR1,0);    // reset timer
                               eep_write8(EEPARB_DEL_START_ENA,true);  // write enable to eeprom
                               delayed_start_enable = true; // and go...
                           } // else
                       } // else
                   } // if
                   else delayed_start_enable = false; // D0 command cancels delayed-start
               } // else
               break;
               
	   case 'e': // Enable/Disable Ethernet with WIZ550io
               if (num < 2)
               {
                   if (num) ethernet_WIZ550i = true;
                   else     ethernet_WIZ550i = false;
                   write_eeprom_parameters(); // save value in eeprom
                   if (ethernet_WIZ550i) init_WIZ550IO_module(); // this hangs the system if no WIZ550io is present!!!
               }
               else if (num ==2)
               {
                   if (rs232_udp == ETHERNET_UDP)
                   {
                       udp_beginPacketIP(remoteIP, EBREW_PORT_NR); // send response back
                   } // if
                   if (ethernet_WIZ550i)
                   {
                       sprintf(s2,"ETH mode (E1): %d.%d.%d.%d\n",local_ip[0],local_ip[1],local_ip[2],local_ip[3]);
                   } // if
                   else sprintf(s2,"USB mode (E0)\n");
                   pr(rs232_udp,s2);  // print to UART or ETH
                   if (rs232_udp == ETHERNET_UDP)
                   {
                       udp_endPacket(); // send response
                   } // if
               } // else if
               else rval = ERR_NUM;
               break;
               
	   case 'h': // PWM signal for HLT Modulating Gas-Burner
               temp = atoi(&s[3]); // convert PWM signal to number
               if      (num > 15)   rval = ERR_CMD; // [GAS_MODU,GAS_ONOFF,ELEC_HTR1,ELEC_HTR2]
               else if (temp > 100) rval = ERR_NUM; // error if pwm > 100
               else process_pwm_signal(PWM_HLT,temp,num);
               break;
               
	   case 'l': // ALIVE-Led
               if (num > 1) rval = ERR_NUM;
               else
               {
                   if (num) ALIVE_LED_Bb = 1;
                   else     ALIVE_LED_Bb = 0;
               } // else
               break;
               
           
	   case 'p': // Pump
               if (num > 3) rval = ERR_NUM;
               else 
               {
                   if (num & 0x01) PUMP_230Vb  = 1;  // Main Brew-Pump
                   else            PUMP_230Vb  = 0;
                   if (num & 0x02) PUMP2_230Vb = 1; // Pump 2 for HLT heat-exchanger
                   else            PUMP2_230Vb = 0;
               } // else
               break;
               
	   case 'r': // Reset flows
               if (num > 0) rval = ERR_NUM;
               else
               {
                   flow_hlt_mlt = flow_mlt_boil = flow_cfc_out = flow4 = 0L;
               } // else
               break;
               
	   case 's': // System commands
               if (rs232_udp == ETHERNET_UDP)
               {
                   udp_beginPacketIP(remoteIP, EBREW_PORT_NR); // send response back
               } // if
               rval = 67 + num;
               switch (num)
               {
                   case 0: // Ebrew revision
                       print_ebrew_revision(s2); // print CVS revision number
                       pr(rs232_udp,s2); // print to UART or ETH
                       delayed_start_enable = false;  // disable delayed-start when PC program is powering-up
                       break;
                   case 1: // List all I2C devices
                       i2c_scan(I2C_CH0, rs232_udp);  // Start with main I2C channel
                       i2c_scan(I2C_CH1, rs232_udp); // I2C channel 1 (THLT)
                       i2c_scan(I2C_CH2, rs232_udp); // I2C channel 2 (TMLT)
                       break;
                   case 2: // List all tasks
                       list_all_tasks(rs232_udp); 
                       break;	
                   case 3: // List all One-Wire Devices (finding a sensor costs approx. 350 msec.)
                       find_OW_device(I2C_CH0,DS2482_THLT_BASE);  // Find ROM ID of HLT  DS18B20
                       find_OW_device(I2C_CH0,DS2482_TBOIL_BASE); // Find ROM ID of BOIL DS18B20
                       find_OW_device(I2C_CH0,DS2482_TCFC_BASE);  // Find ROM ID of CFC  DS18B20
                       find_OW_device(I2C_CH0,DS2482_TMLT_BASE);  // Find ROM ID of MLT  DS18B20
                       break;
                   default: rval = ERR_NUM;
                   break;
               } // switch
               if (rs232_udp == ETHERNET_UDP)
               {
                   udp_endPacket(); // send response
               } // if
               break;
               
	   case 'v': // Output Valve On-Off signals to hardware
               rval = 78;
               PB_ODR = num; // write valve bits to PORTB
               break;
               
	   case 'x': // Sound Buzzer
               if (num > 8) rval = ERR_NUM;
               else
               {
                   bz_rpt_max = num;
                   bz_on      = true;
               } // else				 
               break;
               
	   default: rval = ERR_CMD;
           break;
   } // switch
   return rval;	
} // execute_single_command()