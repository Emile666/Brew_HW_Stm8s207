/*==================================================================
  File Name: brew_stm8s207.c
  Author   : Emile
  ------------------------------------------------------------------
  Purpose  : This is the main file for brew_stm8s207.c
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
  ==================================================================
  Revision 2.00  2021/03/31 12:20  Emile
  - Ported from Brew_Arduino rev. r1.39 for e-brew PCB v4.01 

  Revision 1.39  2021/03/14 11:25:00  Emile
  - Green Alive LED now shows interrupt activity

  Revision 1.38  2021/03/14 11:25:00  Emile
  - Larger reset times for WIZ550io
  - E2 command now with IP-address

  Revision 1.37  2021/01/27 21:44:00  Emile
  - thlt_ow and tmlt_ow now sent separately to PC

  Revision 1.36  2020/08/17 17:33:00  Emile
  - Bug-fix HLT PWM output: delayed-start function blocked this.

  Revision 1.35  2020/05/10 14:38:00  Emile
  - delayed-start function added with eeprom save
  - D0, D1 and D2 commands for delayed-start added

  Revision 1.34  2018/10/25 11:12:00  Emile
  - Bugfix MCP23017 addressing, IOCON definition was for BANK==1. Now all
    definitions are for BANK==0, which is the default at power-up.
  - Buzzer alarm added to PD3 + Xx command added

  Revision 1.33  2018/03/11 15:40:00  Emile
  - Bugfix: for some reason, the MCP23017_init() routine needs to be called twice.
            With Ethernet operations, there's no reset anymore from the USB, and
            setting IO on the MCP23017 did not work. Corrected by calling the
            MCP23017_init() twice. Not nice, but it works.

  Revision 1.32  2018/02/18 15:55:00  Emile
  - Better debugging info for ETH start, DHCP timeouts larger, E2 command added.
  - Bug-fix Ethernet_begin(): SHAR register should be used for reading MAC address.
  - R0 (reset flows) command added

  Revision 1.31  2018/02/10 16:24:12  Emile
  - LocalPort and LocalIP no longer stored in EEPROM: LocalIP is init by DHCP
    and LocalPort should always be set to 8888.

  Revision 1.30  2016/10/30 11:01:21  Emile
  - More relaxed I2C addressing per channel. Any LM92 address is now accepted
    for HLT and MLT temperature sensor.

  Revision 1.29  2016/08/07 13:46:51  Emile
  - Pump 2 (HLT heat-exchanger pump) is now supported with Px command.

  Revision 1.28  2016/06/11 16:50:07  Emile
  - I2C_start() performance improved, one-wire duration from 22 -> 6 msec.
  - Network communication now works using DHCP

  Revision 1.28  2016/05/22 12:14:58  Emile
  - Baud-rate to 38400 Baud.
  - Temperature error value now set to '-99.99'.

  Revision 1.27  2016/05/15 12:24:20  Emile
  - I2C clock speed now adjustable
  - IP address and port now stored in eeprom

  Revision 1.26  2016/04/16 19:39:04  Emile
  - Bugfix reading flowsensor values
  - Bugfix Tcfc and Tboil reading tasks.

  Revision 1.25  2016/04/16 11:22:58  Emile
  - One temperature slope parameter for all temps. Now fixed value (2 degrees/second).
  - Temp. Offset parameters removed.
  - All parameters > 12 removed from parameter list. Now only pars 1-6 left.
  - Tasks for CFC and Boil Temperature added.

  Revision 1.24  2016/04/01 14:06:08  Emile
  - Bugfix PWM generation. Now both PWM signals work with PCB v3.30.

  Revision 1.23  2016/01/10 16:00:23  Emile
  First version (untested!) for new HW PCB 3.30 with 4 x temperature, 4 x flowsensor and 2 PWM outputs.
  - Added: owb_task(), owc_task(), tcfc_ and tboil_ variables. Removed: vhlt_ and vhlt_ variables.
  - A5..A8 commands added (flowsensors), A1..A4 commands re-arranged.
  - Wxxx command is now Hxxx command, new Bxxx command added.
  - pwm_write() and pwm_2_time() now for 2 channels (HLT and Boil): OCR1A and OCR1B timers used.
  - SPI_SS now from PB2 to PD7 (OC1B/PB2 used for 2nd PWM signal). PWM freq. now set to 25 kHz.
  - PCINT1_vect now works with 4 flowsensors: flow_cfc_out and flow4 added.
  - MCP23017 instead of MCP23008: PORTB used for HLT_NMOD, HLT_230V, BOIL_NMOD, BOIL_230V and PUMP_230V.
  - set_parameter(): parameters 7-12 removed.

  Revision 1.22  2015/08/07 13:32:04  Emile
  - OW_task() split into owh_task() and owm_task(). Blocking time now reduced
    from 34 msec. to 24 msec.

  Revision 1.21  2015/08/06 14:41:16  Emile
  - Adapted for MCP23008 instead of MCP23017.

  Revision 1.20  2015/07/01 21:03:46  Emile
  - Bug-fix in scheduler time-measurement. Now reads proper time in msec
  - Usart comm. now IRQ driven, so that all receiving commands are handled
  - DS18B20 reads only 2 bytes (instead of 9). Total time taken is now 28 msec.
    This was 60 msec. and caused multiple reads at PC side.

  Revision 1.19  2015/06/28 12:27:35  Emile
  - Moving_average filters now work with Q8.7 instead of Q8.4 format
  - One-wire functions now work with DS18B20
  - Separate ow_task() added for one-wire communication
  - I2C clock made adjustable

  Revision 1.18  2015/06/05 13:51:04  Emile
  - Headers added to one_wire sources

  Revision 1.17  2015/05/31 10:28:57  Emile
  - Bugfix: Flowsensor reading counted rising and falling edges.
  - Bugfix: Only valve V8 is written to at init (instead of all valves).

  Revision 1.16  2015/05/12 14:18:37  Emile
  - HW-bugfix: MCP23017 output latches needs to be written first with 0xFF.

  Revision 1.15  2015/05/09 14:37:37  Emile
  - I2C Channel & HW-address update for MCP23017 to reflect changes in HW PCB V3.01

  Revision 1.14  2015/05/09 13:36:54  Emile
  - MCP23017 Port B now always input. Port A Pull-up resistors enabled.
  - Bug-fix for HW PCB V3.01

  Revision 1.13  2014/11/30 20:44:45  Emile
  - Vxxx command added to write valve output bits
  - mcp23017 (16 bit I2C IO-expander) routines + defines added

  Revision 1.12  2014/11/09 15:38:34  Emile
  - PUMP_LED removed from PD2, PUMP_230V has same function
  - Interface for 2nd waterflow sensor added to PD2
  - Command A6 (read waterflow in E-2 L) added
  - FLOW_PER_L changed to 330 (only rising edge is counted)

  Revision 1.11  2014/10/26 12:44:47  Emile
  - A3 (Thlt) and A4 (Tmlt) commands now return '99.99' in case of I2C HW error.

  Revision 1.10  2014/06/15 14:52:19  Emile
  - Commands E0 and E1 (Disable/Enable Ethernet module) added
  - Interface for waterflow sensor added to PC3/ADC3
  - Command A5 (read waterflow in E-2 L) added

  Revision 1.9  2014/06/01 13:51:49  Emile
  - Update CVS revision number

  Revision 1.8  2014/05/03 11:27:43  Emile
  - Ethernet support added for W550io module
  - No response for L, N, P, W commands anymore
  - All source files now have headers

  Revision 1.7  2013/07/24 13:46:39  Emile
  - Minor changes in S1, S2 and S3 commands to minimize comm. overhead.
  - Version ready for Integration Testing with PC program!

  Revision 1.6  2013/07/23 19:33:17  Emile
  - Bug-fix slope-limiter function. Tested on all measurements.

  Revision 1.5  2013/07/21 13:10:43  Emile
  - Reading & Writing of 17 parameters now fully works with set_parameter()
  - VHLT and VMLT tasks added
  - Scheduler: actual & max. times now printed in msec. instead of usec.
  - THLT and TMLT now in E-2 Celsius for PC program
  - All lm92 test routines removed, only one lm92_read() remaining

  Revision 1.4  2013/07/20 14:51:59  Emile
  - LM35, THLT and TMLT tasks are now working
  - Max. duration added to scheduler
  - slope_limiter & lm92_read() now work with uint16_t instead of float

  Revision 1.3  2013/07/19 10:51:01  Emile
  - I2C frequency 50 50 kHz to get 2nd LM92 working
  - Command Mx removed, command N0 x added, commands N0..N3 renamed to N1..N4
  - Command S3 added, list_all_tasks. To-Do: get timing-measurement working
  - Scheduler added with 3 tasks: lm35, led_blink and pwm_2_time

  Revision 1.2  2013/06/23 08:56:03  Emile
  - Working version (with PC program) with new command-set.
  ================================================================== */ 
#include "brew_stm8s207.h"
#include "stm8_hw_init.h"
#include "w5500.h"
#include "Ethernet.h"
#include "Udp.h"
#include "one_wire.h"

extern char rs232_inbuf[];

// Global variables
uint8_t      local_ip[4]      = {0,0,0,0}; // local IP address, gets a value from init_WIZ550IO_module() -> dhcp_begin()
uint16_t     local_port;                   // local port number read back from wiz550i module
const char  *ebrew_revision   = "$Revision: 2.02 $"; // ebrew CVS revision number
bool         ethernet_WIZ550i = true;		     // true = start WIZ550i at power-up

// The following variables are defined in Udp.c
extern uint8_t  remoteIP[4]; // remote IP address for the incoming packet whilst it's being processed 
extern uint16_t remotePort;  // remote port for the incoming packet whilst it's being processed 
extern uint8_t  _sock;       // socket ID for Wiz5100
unsigned char   udp_rcv_buf[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet]

// The following variables are needed for the buzzer
extern bool     bz_on;      // true = buzzer is enabled
extern uint8_t  bz_rpt_max; // number of beeps to make

//-----------------------------------
// Electric Heating variables
//-----------------------------------
uint8_t  hlt_elec1_pwm = 0;       // PWM signal (0-100 %) for HLT Electric Heating 1
uint8_t  hlt_elec2_pwm = 0;       // PWM signal (0-100 %) for HLT Electric Heating 2
pwmtime  pwmhlt1;                 // Struct for HLT Electric Heater 1 Slow SSR signal
pwmtime  pwmhlt2;                 // Struct for HLT Electric Heater 2 Slow SSR signal
uint8_t  bk_elec1_pwm = 0;        // PWM signal (0-100 %) for Boil-kettle Electric Heating 1
uint8_t  bk_elec2_pwm = 0;        // PWM signal (0-100 %) for Boil-kettle Electric Heating 2
pwmtime  pwmbk1;                  // Struct for Boil-kettle Electric Heater 1 Slow SSR signal
pwmtime  pwmbk2;                  // Struct for Boil-kettle Electric Heater 2 Slow SSR signal
uint8_t  elec_htrs = 0x00;        // Bit-define for every electrical phase of HLT and BK

//-----------------------------------
// LM35 parameters and variables
//-----------------------------------
ma       lm35_ma;                   // struct for LM35 moving_average filter
uint16_t lm35_temp;                 // LM35 Temperature in E-2 °C

//------------------------------------------------
// THLT parameters and variables (I2C)
//------------------------------------------------
int16_t  temp_slope_87  = 512;      // Temperature slope-limiter is 4 °C/ 2 sec. * 128
ma       thlt_ma;                   // struct for THLT moving_average filter
int16_t  thlt_old_87;               // Previous value of thlt_temp_87
int16_t  thlt_temp_87;              // THLT Temperature from LM92 in °C * 128
uint8_t  thlt_err = 0;              // 1 = Read error from LM92

//-----------------------------------------------------
// THLT parameters and variables (One-Wire). Used for:
// 1) Backup for THLT-I2C
// 2) Can be used as extra temp. sensor
//-----------------------------------------------------
ma       thlt_ow_ma;                // struct for THLT_OW moving_average filter
int16_t  thlt_ow_old_87;            // Previous value of thlt_ow_87
int16_t  thlt_ow_87;                // THLT Temperature from DS18B20 in °C * 128
uint8_t  thlt_ow_err = 0;	    // 1 = Read error from DS18B20
 
//------------------------------------------------
// TMLT parameters and variables (I2C)
//------------------------------------------------
ma       tmlt_ma;                   // struct for TMLT moving_average filter
int16_t  tmlt_old_87;               // Previous value of tmlt_temp_87
int16_t  tmlt_temp_87;              // TMLT Temperature from LM92 in °C * 128
uint8_t  tmlt_err = 0;              // 1 = Read error from LM92

//-----------------------------------------------------
// TMLT parameters and variables (One-Wire). Used for:
// 1) Backup for TMLT-I2C
// 2) Can be used as extra temp. sensor
//-----------------------------------------------------
ma       tmlt_ow_ma;                // struct for TMLT_OW moving_average filter
int16_t  tmlt_ow_old_87;            // Previous value of tmlt_ow_87
int16_t  tmlt_ow_87;                // TMLT Temperature from DS18B20 in °C * 128
uint8_t  tmlt_ow_err = 0;	        // 1 = Read error from DS18B20

//------------------------------------------------
// TCFC parameters and variables (One-Wire only)
//------------------------------------------------
ma       tcfc_ma;                   // struct for TCFC moving_average filter
int16_t  tcfc_old_87;               // Previous value of tcfc_temp_87
int16_t  tcfc_temp_87;              // TCFC Temperature in °C * 128
uint8_t  tcfc_err = 0;              // 1 = Read error from DS18B20  

//------------------------------------------------
// TBOIL parameters and variables (One-Wire only)
//------------------------------------------------
ma       tboil_ma;                  // struct for TBOIL moving_average filter
int16_t  tboil_old_87;              // Previous value of tboil_temp_87
int16_t  tboil_temp_87;             // TBOIL Temperature in °C * 128
uint8_t  tboil_err = 0;             // 1 = Read error from DS18B20

uint32_t flow_hlt_mlt  = 0UL;
uint32_t flow_mlt_boil = 0UL;
uint32_t flow_cfc_out  = 0UL; // Count from flow-sensor at output of CFC
uint32_t flow4         = 0UL; // Count from FLOW4 (future use)

//------------------------------------------------
// Delayed-start variables
//------------------------------------------------
bool     delayed_start_enable  = false;          // true = delayed start is enabled
uint16_t delayed_start_time    = 0;              // delayed start time in 2 sec. counts 
uint16_t delayed_start_timer1;                   // timer to countdown until delayed start
uint8_t  delayed_start_std     = DEL_START_INIT; // std number, start in INIT state

/*------------------------------------------------------------------
  Purpose  : This is the delayed-start routine which is called by 
             lm35_task() every 2 seconds. 
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void process_delayed_start(void)
{
    uint8_t         led_tmr_max = 0; // blink every x seconds  
    static uint8_t  led_tmr     = 0; // Timer for blinking of RED led
    static uint16_t eep_tmr;         // minute counter for eeprom write
    static uint16_t timer2;          // timer for max. burn in minutes (max. 120 minutes)
    
    switch (delayed_start_std)
    {
        case DEL_START_INIT: // Delayed-start is not enabled
            if (delayed_start_enable)
            {
                delayed_start_timer1 = eep_read16(EEPARB_DEL_START_TMR1); // read value from eeprom
                delayed_start_time   = eep_read16(EEPARB_DEL_START_TIME); // read value from eeprom
                eep_tmr              = 0;                      // reset eeprom minute timer
                delayed_start_std    = DEL_START_TMR;          // start countdown timer
            } // if				 
            led_tmr_max = 0; // no blinking
            break;
            
        case DEL_START_TMR: // The delayed-start timer is running, no time-out yet
            if (!delayed_start_enable)
            {
                eep_write8(EEPARB_DEL_START_ENA,false); // reset enable in eeprom
                delayed_start_std = DEL_START_INIT;
            } // if				 
            else if (++delayed_start_timer1 >= delayed_start_time)
            {   // delayed-start counted down
                timer2            = 0;              // init. burn-timer 
                delayed_start_std = DEL_START_BURN; // start HLT burner
            } // if
            if (++eep_tmr >= 30)
            {   // save in eeprom every minute
                eep_tmr = 0;
                eep_write16(EEPARB_DEL_START_TMR1,delayed_start_timer1);
            } // if
            led_tmr_max = 5; // blink once every 10 seconds
            break;
            
        case DEL_START_BURN: // Delayed-start time-out, HLT-burner is burning
            if (!delayed_start_enable)
            {
                eep_write8(EEPARB_DEL_START_ENA,false); // reset enable in eeprom
                delayed_start_std = DEL_START_INIT;
            } // if
            else 
            {	// For delayed start: only enable HLT electric heaters with fixed percentage	 
                process_pwm_signal(PWM_HLT,DEL_START_ELEC_PWM,ELEC_HTR1|ELEC_HTR2);
                if (++timer2 >= DEL_START_MAX_BURN_TIME) 
                {   // Safety feature, set burn-time to max. of 2 hours
                    delayed_start_enable = false; // prevent another burn
                    delayed_start_std    = DEL_START_INIT;
                    eep_write8(EEPARB_DEL_START_ENA,false); // reset enable in eeprom
                } // if
            } // else				 
            led_tmr_max = 1; // blink once every 4 seconds
            break;
            
        default: delayed_start_std = DEL_START_INIT;
        break;
    } // switch
    if ((led_tmr_max > 0) && (++led_tmr > led_tmr_max))
    {    // blink once every led_tmr_max seconds
        led_tmr = 0;
        ALIVE_LED_Rb = 1; // red LED on
    } // if
    else ALIVE_LED_Rb = 0; // red LED off
} // process_delayed_start()

/*------------------------------------------------------------------
  Purpose  : This function initializes the pwm_2_time structs that
             are used for the HLT electric heating-elements.
  Variables: 
          p: pointer to struct to initialize
	   mask: bit-defines of electric heaters in elec_htrs variable.
	  on1st: phase of PWM signal, true = 1 first, false = 0 first
  Returns  : -
  ------------------------------------------------------------------*/
void init_pwm_time(pwmtime *p, uint8_t mask, bool on1st)
{
	p->std    = EL_HTR_OFF; // default state
	p->mask   = mask;       // bit-define for elec_htrs
	p->on1st  = on1st;      // true = high first, then low
} // init_pwm_time()

/*-----------------------------------------------------------------------------
  Purpose  : Converts a PWM signal into a time-division signal of 100 * 50 msec.
             This is used to control the electric heating-elements.
             This routine should be called from pwm_task() every 50 msec.
			 It uses the pwm values which are set by process_pwm_signal().
  Variables: p: pointer to pwmtime struct. There are 2 structs, one for each
                HLT electric heating-element.
		  cntr: time-division counter, counts from 1 to 100 and back again.
           pwm: this is PWM signal for the 
  Returns  : -
  ---------------------------------------------------------------------------*/
void pwm_2_time(pwmtime *p, uint8_t cntr, uint8_t pwm)
{
	int8_t  bt,et;
	
	bt  = p->on1st ? 25 : 75; // time center-point
	bt -= (pwm>>1);           // start-time for a 1
	et  = bt + pwm;           // end-time for a 1
	if      (bt < 0)   { bt = 0        ; et = pwm; }
	else if (et > 100) { bt = 100 - pwm; et = 100; }

	switch (p->std)
	{
		case EL_HTR_OFF:
			//------------------------------------------------------------
			// Goto ON-state if cntr in [bt,et] && pwm>0
			//------------------------------------------------------------
			elec_htrs &= ~p->mask;
			if ((pwm > 0) && (cntr >= bt) && (cntr < et))
			{  
				p->std = EL_HTR_ON; // go to ON-state
			} // if
			// else: pwm=0 or outside 1-time, continue in this state
			break;

		case EL_HTR_ON:
			//---------------------------------------------------------
			// Goto OFF-state if pwm=0 or cntr>et && pwm<100
			//---------------------------------------------------------
			elec_htrs |= p->mask;       // set electric heater ON
			if ((pwm == 0) || ((cntr >= et) && (pwm < 100)))
			{   
				p->std = EL_HTR_OFF;   // go to 'OFF' state
			} // if
			// else: pwm>0 && ((cntr < et) || (pwm>=100))
			break;

		default: 
		    p->std = EL_HTR_OFF;
		    break;	
	} // switch 
} // pwm_2_time()

/*-----------------------------------------------------------------------------
  Purpose  : This task converts a PWM signal into a time-division signal of 
             100 * 50 msec. This routine should be called from the timer-interrupt 
			 every 50 msec. It uses the hlt_elecx_pwm and bk_elecx_pwm signals 
			 which were set by the process_pwm_signal(). This variable will only 
			 get a non-zero number when the corresponding electrical 
			 heating-element is enabled.
			 
			 Priority: the HLT and BK heating-elements can be sourced from the
			           same outlet. If the BK starts to heat, it may be possible
					   for the HLT to draw current too. In order to prevent this,
					   the BK has priority over the HLT. In other words, if the
					   BK heating-element is energized, the HLT heating-element
					   on the same outlet is disabled.
  Variables: - 
  Returns  : -
  ---------------------------------------------------------------------------*/
void pwm_task(void)
{
   static uint8_t cntr = 1; // time-division counter
   
   pwm_2_time(&pwmbk1 ,cntr,bk_elec1_pwm);   // BK heating-element 1
   pwm_2_time(&pwmbk2 ,cntr,bk_elec2_pwm);   // BK heating-element 2
   pwm_2_time(&pwmhlt1,cntr,hlt_elec1_pwm);  // HLT heating-element 1
   pwm_2_time(&pwmhlt2,cntr,hlt_elec2_pwm);  // HLT heating-element 2
   // If both heaters of the same phase are on, disable the HLT phase
   if ((elec_htrs & (HTR_BK1 | HTR_HLT1)) == (HTR_BK1 | HTR_HLT1)) elec_htrs &= ~HTR_HLT1;
   if ((elec_htrs & (HTR_BK2 | HTR_HLT2)) == (HTR_BK2 | HTR_HLT2)) elec_htrs &= ~HTR_HLT2;
   if ((elec_htrs & (HTR_BK3 | HTR_HLT3)) == (HTR_BK3 | HTR_HLT3)) elec_htrs &= ~HTR_HLT3;
   
   if (elec_htrs & pwmhlt1.mask)
        HLT_SSR1b = true;  // Enable  HLT heater 1
   else HLT_SSR1b = false; // Disable HLT heater 1
   if (elec_htrs & pwmhlt2.mask)
        HLT_SSR2b = true;  // Enable  HLT heater 2
   else HLT_SSR2b = false; // Disable HLT heater 2
   // HLT electric heater 3 not implemented yet
   
   if (elec_htrs & pwmbk1.mask)
        BK_SSR1b = true;  // Enable  BK heater 1
   else BK_SSR1b = false; // Disable BK heater 1
   if (elec_htrs & pwmbk2.mask)
        BK_SSR2b = true;  // Enable  BK heater 2
   else BK_SSR2b = false; // Disable BK heater 2
   // BK electric heater 3 not implemented yet
   if (++cntr > 100) cntr = 1;
} // pwm_task()

/*--------------------------------------------------------------------
   Purpose  : This task is called every 2 seconds and does the following:
             1) processes the temperature from the Hot-Liquid Tun (HLT). 
	        The first sensor to read is the LM92 (I2C sensor). If 
		that one is not present, the DS18B20 (One-Wire sensor) 
		is tried. It uses the output from ow_task(), which delivers
		a new DS18B20 reading every 2 seconds.
		Since both sensors have 4 fractional bits (1/2, 1/4, 1/8, 1/16), 
		a signed Q8.4 format would be sufficient. However all variables 
		are stored in a Q8.7 format for accuracy reasons when filtering. 
		All variables with this format have the extension _87. The 
		HLT temperature is both filtered and slope-limited.
	     2) processes the temperature from the LM35 temperature sensor. 
		This sensor is connected to ADC0. The LM35 outputs 10 mV/°C,
		VREF=1.1 V, therefore => Max. Temp. = 11000 E-2 °C
		Conversion constant = 11000 / 1023 = 10.7526882
             3) executes the delayed-start function, which will fire the
		HLT burner after a number of minutes set by the D0 command.

  Variables: thlt_old_87   : previous value of thlt_temp_87
             temp_slope_87 : the slope-limit: 2 °C/sec.
             thlt_err      : 1 = error reading from LM92 (I2C)
             thlt_ow_err   : 1 = error reading from DS18B20 (One-Wire)
             thlt_ow_87    : Temperature value from DS18B20 (One-Wire)
             thlt_ma       : the moving-average filter struct for THLT
             thlt_temp_87  : the processed HLT temperature in °C
             lm35_temp     : contains temperature in E-2 °C
             lm35_frac     : contains fractional temperature in E-2 °C
             lm35_ma       : moving-average filter struct for lm35_temp
  Returns  : -
  --------------------------------------------------------------------*/
void thlt_task(void)
{
    int16_t  tmp; // temporary variable (signed Q8.7 format)
    float   tmpf; // temporary variable
    
    thlt_old_87 = thlt_temp_87; // copy previous value of thlt_temp
    tmp         = lm92_read(I2C_CH1, &thlt_err); // returns a signed Q8.7 format
    if (!thlt_err) 
    {	// filter if I2C-sensor has been read successfully
        slope_limiter(temp_slope_87, thlt_old_87, &tmp);
        thlt_temp_87 = (int16_t)(moving_average(&thlt_ma, (float)tmp) + 0.5);
    } // if
    
    tmpf      = read_adc(AD_LM35) * LM35_CONV;
    lm35_temp = (uint16_t)moving_average(&lm35_ma, tmpf);
    process_delayed_start(); // process delayed-start function
} // thlt_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperature from
	     the Mash-Lauter Tun (MLT). The first sensor to read is the
	     LM92 (I2C sensor). If that one is not present, the DS18B20
	     (One-Wire sensor) is tried. Since both sensors have 4
	     fractional bits (1/2, 1/4, 1/8, 1/16), a signed Q8.4 format
	     would be sufficient. However all variables are stored in a
	     Q8.7 format for accuracy reasons when filtering. All variables
	     with this format have the extension _87.
	     The MLT temperature is both filtered and slope-limited.
	     This task is called every 2 seconds and uses the output from
	     ow_task(), which delivers a new DS18B20 reading every 2 seconds.
  Variables: tmlt_old_87   : previous value of tmlt_temp_87
	     temp_slope_87 : the slope-limit: 2 °C/sec.
	     tmlt_err      : 1 = error reading from LM92 (I2C)
	     tmlt_ow_err   : 1 = error reading from DS18B20 (One-Wire)
	     tmlt_ow_87    : Temperature value from DS18B20 (One-Wire)
	     tmlt_ma       : the moving-average filter struct for TMLT
	     tmlt_temp_87  : the processed MLT temperature in °C
  Returns  : -
  --------------------------------------------------------------------*/
void tmlt_task(void)
{
    int16_t tmp; // temporary variable (signed Q8.7 format)
    
    tmlt_old_87 = tmlt_temp_87; // copy previous value of tmlt_temp
    tmp         = lm92_read(I2C_CH2, &tmlt_err); // returns a signed Q8.7 format
    if (!tmlt_err)
    {	// filter if I2C-sensor has been read successfully
        slope_limiter(temp_slope_87, tmlt_old_87, &tmp);
        tmlt_temp_87 = (int16_t)(moving_average(&tmlt_ma, (float)tmp) + 0.5);
    } // else
} // tmlt_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperatures from
             the HLT One-Wire sensor. The sensor has its
             own DS2482 I2C-to-One-Wire bridge. Since this sensor has 4
             fractional bits (1/2, 1/4, 1/8, 1/16), a signed Q8.4 format
             would be sufficient. However all variables are stored in a
             Q8.7 format for accuracy reasons when filtering. All variables
             with this format have the extension _87.
             This task is called every second so that every 2 seconds a new
             temperature is present.
  Variables: thlt_ow_87 : HLT temperature read from sensor in Q8.7 format
	     thlt_ow_err: 1=error
  Returns  : -
  --------------------------------------------------------------------*/
void owh_task(void)
{
    static int owh_std = 0; // internal state
    int16_t    tmp;         // temporary variable (signed Q8.7 format)
    
    switch (owh_std)
    {   
    case 0: // Start Conversion
        ds18b20_start_conversion(I2C_CH0, DS2482_THLT_BASE);
        owh_std = 1;
        break;
    case 1: // Read Thlt_ow device
        thlt_ow_old_87 = thlt_ow_87; // copy previous value of thlt_ow
        tmp = ds18b20_read(I2C_CH0, DS2482_THLT_BASE, &thlt_ow_err,1);
        if (!thlt_ow_err)
        {
            slope_limiter(temp_slope_87, thlt_ow_old_87, &tmp);
            thlt_ow_87 = (int16_t)(moving_average(&thlt_ow_ma, (float)tmp) + 0.5);
        } // if
        owh_std = 0;
        break;
    } // switch
    BG_LEDb = !BG_LEDb; // alive-led for background-processes
} // owh_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperatures from
             the MLT One-Wire sensor. The sensor has its
             own DS2482 I2C-to-One-Wire bridge. Since this sensor has 4
             fractional bits (1/2, 1/4, 1/8, 1/16), a signed Q8.4 format
             would be sufficient. However all variables are stored in a
             Q8.7 format for accuracy reasons when filtering. All variables
             with this format have the extension _87.
             This task is called every second so that every 2 seconds a new
             temperature is present.
  Variables: tmlt_ow_87 : MLT temperature read from sensor in Q8.7 format
	     tmlt_ow_err: 1=error
  Returns  : -
  --------------------------------------------------------------------*/
void owm_task(void)
{
    static int owm_std = 0; // internal state
    int16_t    tmp;         // temporary variable (signed Q8.7 format)
    
    switch (owm_std)
    {   
    case 0: // Start conversion
        ds18b20_start_conversion(I2C_CH0, DS2482_TMLT_BASE);
        owm_std = 1;
        break;
    case 1: // Read Tmlt_ow device
        tmlt_ow_old_87 = tmlt_ow_87; // copy previous value of tmlt_ow
        tmp = ds18b20_read(I2C_CH0, DS2482_TMLT_BASE, &tmlt_ow_err,1);
        if (!tmlt_ow_err)
        {
            slope_limiter(temp_slope_87, tmlt_ow_old_87, &tmp);
            tmlt_ow_87 = (int16_t)(moving_average(&tmlt_ow_ma, (float)tmp) + 0.5);
        } // if
        owm_std = 0;
        break;
    } // switch
} // owm_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperatures from
             the Boil-kettle One-Wire sensor. The sensor has its
             own DS2482 I2C-to-One-Wire bridge. Since this sensor has 4
             fractional bits (1/2, 1/4, 1/8, 1/16), a signed Q8.4 format
             would be sufficient. However all variables are stored in a
             Q8.7 format for accuracy reasons when filtering. All variables
             with this format have the extension _87.
             This task is called every second so that every 2 seconds a new
             temperature is present.
  Variables: tboil_temp_87 : Boil-kettle temperature read from sensor in Q8.7 format
	     tboil_err     : 1=error
  Returns  : -
  --------------------------------------------------------------------*/
void owb_task(void)
{
    static int owb_std = 0; // internal state
    int16_t    tmp;         // temporary variable (signed Q8.7 format)
    
    switch (owb_std)
    {   
    case 0: // Start conversion
        ds18b20_start_conversion(I2C_CH0, DS2482_TBOIL_BASE);
        owb_std = 1;
        break;
    case 1: // Read Tboil device
        tboil_old_87 = tboil_temp_87; // copy previous value of tboil_temp
        tmp = ds18b20_read(I2C_CH0, DS2482_TBOIL_BASE, &tboil_err,1);
        if (!tboil_err)
        {
            slope_limiter(temp_slope_87, tboil_old_87, &tmp);
            tboil_temp_87 = (int16_t)(moving_average(&tboil_ma, (float)tmp) + 0.5);
        } // if
        owb_std = 0;
        break;
    } // switch
} // owb_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperatures from
             the Counterflow Chiller (CFC) One-Wire sensor. The sensor has
             its own DS2482 I2C-to-One-Wire bridge. Since this sensor has 4
             fractional bits (1/2, 1/4, 1/8, 1/16), a signed Q8.4 format
             would be sufficient. However all variables are stored in a
             Q8.7 format for accuracy reasons when filtering. All variables
             with this format have the extension _87.
             This task is called every second so that every 2 seconds a new
             temperature is present.
  Variables: tcfc_temp_87 : CFC temperature read from sensor in Q8.7 format
	     tcfc_err     : 1=error
  Returns  : -
  --------------------------------------------------------------------*/
void owc_task(void)
{
    static int owc_std = 0; // internal state
    int16_t    tmp;         // temporary variable (signed Q8.7 format)
    
    switch (owc_std)
    {   
    case 0: // Start conversion
        ds18b20_start_conversion(I2C_CH0, DS2482_TCFC_BASE);
        owc_std = 1;
        break;
    case 1: // Read Tcfc device
        tcfc_old_87 = tcfc_temp_87; // copy previous value of tcfc_temp
        tmp = ds18b20_read(I2C_CH0, DS2482_TCFC_BASE, &tcfc_err,1);
        if (!tcfc_err)
        {
            slope_limiter(temp_slope_87, tcfc_old_87, &tmp);
            tcfc_temp_87 = (int16_t)(moving_average(&tcfc_ma, (float)tmp) + 0.5);
        } // if
        owc_std = 0;
        break;
    } // switch
} // owc_task()

/*------------------------------------------------------------------
  Purpose  : This function prints a welcome message to the serial
             port together with the current CVS revision number.
  Variables: 
        ver: string with version info
  Returns  : -
  ------------------------------------------------------------------*/
void print_ebrew_revision(char *ver)
{
    uint8_t len;
    char s[20];
    
    strcpy(ver, "E-Brew V3.0 rev.");  // welcome message, assure that all is well!
    len = strlen(ebrew_revision) - 13;  // just get the rev. number
    strncpy(s,&ebrew_revision[11],len); // example: " 1.3"
    s[len]   = '\n';
    s[len+1] = '\0';
    strcat(ver,s);
} // print_ebrew_revision()

/*------------------------------------------------------------------
  Purpose  : This function prints the IP address to the serial port.
  Variables: The IP-address to print
  Returns  : -
  ------------------------------------------------------------------*/
void print_IP_address(uint8_t *ip)
{
    char s[30];
    uint8_t i;
    
    for (i = 0; i < 4; i++)
    {
        sprintf(s,"%d",ip[i]);
        uart_printf(s);
        if (i < 3) uart_printf(".");
    } // for
} // print_IP_address()

/*-----------------------------------------------------------------------
  Purpose  : This function inits the WIZ550i Ethernet module by
             calling w5500_init() and spi_init(). If a valid MAC address
			 is detected, then dhcp_begin() is started.
  Variables: The IP-address to print
  Returns  : 1: success, 0: WIZ550IO not present
  -----------------------------------------------------------------------*/
uint8_t init_WIZ550IO_module(void)
{
	char     s[30];   // Needed for uart_printf() and sprintf()
	uint8_t  bufr[8]; // Needed for w5500_read_common_register()
	uint8_t  ret,x;
	uint16_t y;
	
	//---------------------------------------------------------------
	// Reset WIZ550IO Ethernet module
	//---------------------------------------------------------------
        SPI_NRESETb = 0;           // Hardware reset for WIZ550io
	delay_msec(2);             // At least 500 usec according to datasheet
	SPI_NRESETb = 1;           // Disable hardware reset for WIZ550io
	delay_msec(300);           // Giver W5500 time to configure itself (at least 150 msec)

	ret = Ethernet_begin();    // includes w5500_init() & dhcp_begin()
	if (ret == 0)              // Error, no WIZ550IO module found
	{
            ethernet_WIZ550i = false; // No ETH mode, switch back to USB
            return 0;
	} // if	
	
	x = udp_begin(EBREW_PORT_NR);           // init. UDP protocol
	y = w5500_read_common_register(VERSIONR,bufr);
	sprintf(s,"Version: 0x%02x\n",y);          uart_printf(s);
	sprintf(s,"udp_begin():%d, sock=%d\n",x,_sock);   uart_printf(s);
	x = w5500_read_socket_register(_sock, Sn_MR, bufr);
	sprintf(s,"Sn_MR=%d, ",x);                        uart_printf(s);
	local_port = w5500_read_socket_register(_sock, Sn_PORT, bufr);
	sprintf(s,"Sn_PORT=%d, ",local_port);		  uart_printf(s);
	y = w5500_getTXFreeSize(_sock);
	sprintf(s,"Sn_TXfree=%d, ",y);			  uart_printf(s);
	y = w5500_getRXReceivedSize(_sock);
	sprintf(s,"Sn_RXrecv=%d\n",y);			  uart_printf(s);
	w5500_read_common_register(SIPR, bufr);
	uart_printf("Local IP   :"); print_IP_address(bufr);
	for (x = 0; x < 4; x++) local_ip[x] = bufr[x]; // copy IP address
	w5500_read_common_register(GAR, bufr);
	uart_printf("Gateway IP :"); print_IP_address(bufr);
	w5500_read_common_register(SUBR, bufr);
	uart_printf("\nSubnet Mask:"); print_IP_address(bufr);	
	uart_printf("\n");
	return 1; // success
} // init_WIZ550IO_module()

/*------------------------------------------------------------------
  Purpose  : This is the main() function for the E-brew hardware.
  Variables: -
  Returns  : should never return
  ------------------------------------------------------------------*/
int main(void)
{
    char    s[25];     // Needed for uart_printf() and sprintf()
    int	    udp_packet_size;
    uint8_t clk;       // which clock is active
    
    __disable_interrupt();
    //-------------------------------------------------------------------
    // For 24 MHz, set ST-LINK->Option Bytes...->Flash_Wait_states to 1 !
    //-------------------------------------------------------------------
    clk = initialise_system_clock(HSE); // Set system-clock to 24 MHz

    uart_init(clk);           // UART init. to BAUDRATE (uart.h),8,N,1
    setup_timers(clk);        // Set Timer 2 to 1 kHz and timer1 and timer3 for PWM output
    setup_gpio_ports();       // Init. needed output-ports for LED and keys
    i2c_init_bb(I2C_CH0);     // Init. I2C bus 0 for bit-banging
    i2c_init_bb(I2C_CH1);     // Init. I2C bus 1 for bit-banging
    i2c_init_bb(I2C_CH2);     // Init. I2C bus 2 for bit-banging
    //pwm_write(PWM_BK ,0);      // Start with 0 % duty-cycle for Boil-kettle
    //pwm_write(PWM_HLT,0);      // Start with 0 % duty-cycle for HLT
    spi_init();               // Init. SPI module: 4 MHz clock, Master mode, SPI mode 1

    check_and_init_eeprom();  // EEPROM init.
    read_eeprom_parameters(); // Read EEPROM value for ETHUSB and delayed-start
	
    //---------------------------------------------------------------
    // Init. Moving Average Filters for Measurements
    //---------------------------------------------------------------
    init_moving_average(&lm35_ma   ,MAX_MA, (float)INIT_TEMP * 100.0); // Init. MA-filter with 20 °C
    init_moving_average(&thlt_ma   ,MAX_MA, (float)INIT_TEMP * 128.0); // Init. MA-filter with 20 °C
    init_moving_average(&tmlt_ma   ,MAX_MA, (float)INIT_TEMP * 128.0); // Init. MA-filter with 20 °C
    init_moving_average(&tcfc_ma   ,MAX_MA, (float)INIT_TEMP * 128.0); // Init. MA-filter with 20 °C
    init_moving_average(&tboil_ma  ,MAX_MA, (float)INIT_TEMP * 128.0); // Init. MA-filter with 20 °C
    init_moving_average(&thlt_ow_ma,MAX_MA, (float)INIT_TEMP * 128.0); // Init. MA-filter with 20 °C
    init_moving_average(&tmlt_ow_ma,MAX_MA, (float)INIT_TEMP * 128.0); // Init. MA-filter with 20 °C
    lm35_temp     = INIT_TEMP * 100;
    thlt_temp_87  = INIT_TEMP << 7;
    tmlt_temp_87  = INIT_TEMP << 7;
    tcfc_temp_87  = INIT_TEMP << 7;
    tboil_temp_87 = INIT_TEMP << 7;
    thlt_ow_87    = INIT_TEMP << 7;
    tmlt_ow_87    = INIT_TEMP << 7;

    init_pwm_time(&pwmhlt1,HTR_HLT1,ON1ST);  // HLT Electric heating element 1
    init_pwm_time(&pwmhlt2,HTR_HLT2,OFF1ST); // HLT Electric heating element 2
    init_pwm_time(&pwmbk1 ,HTR_BK1 ,OFF1ST); // BK  Electric heating element 1
    init_pwm_time(&pwmbk2 ,HTR_BK2 ,ON1ST);  // BK  Electric heating element 2

    // Initialize all the tasks for the E-Brew system
    add_task(pwm_task  ,"pwm_task"  , 10,   50); // Electrical Heating Time-Division every 50 msec.
    add_task(owh_task  ,"owh_task"  ,120, 1000); // Process Temperature from DS18B20 HLT sensor
    add_task(owm_task  ,"owm_task"  ,220, 1000); // Process Temperature from DS18B20 MLT sensor
    add_task(owb_task  ,"owb_task"  ,320, 1000); // Process Temperature from DS18B20 Boil-kettle sensor
    add_task(owc_task  ,"owc_task"  ,420, 1000); // Process Temperature from DS18B20 CFC-output sensor
    add_task(thlt_task ,"thlt_task" ,520, 2000); // Process Temperature from THLT sensor (I2C and/or OW)
    add_task(tmlt_task ,"tmlt_task" ,620, 2000); // Process Temperature from TMLT sensor (I2C and/or OW)
	
    __enable_interrupt();       // set global interrupt enable, start task-scheduler
    print_ebrew_revision(s);    // print revision number
    uart_printf(s);		// Output to COM-port for debugging
    if (ethernet_WIZ550i)       // Initialize Ethernet adapter
    {
        uart_printf("init Wiz550IO:");
        if (init_WIZ550IO_module())
        {   // 1 = ok, DHCP-server found
                bz_rpt_max = 1; // Sound buzzer once to indicate ethernet connection ready
                bz_on      = true;
        } // if
        uart_printf("starting main()\n");
        print_IP_address(local_ip);
        sprintf(s,":%d\n",local_port); // add local port as read back from wiz550io module
        uart_printf(s);
    } // if
    sprintf(s,"CLK: 0x%X ",clk);
    uart_printf(s);
    if      (clk == HSI) uart_printf("HSI\n");
    else if (clk == LSI) uart_printf("LSI\n");
    else if (clk == HSE) uart_printf("HSE\n");

    while(1)
    {
        dispatch_tasks(); // run the task-scheduler
        switch (rs232_command_handler()) // run command handler continuously
        {
            case ERR_CMD: uart_printf("Cmd Error\n"); 
                          break;
            case ERR_NUM: sprintf(s,"Num Error (%s)\n",rs232_inbuf);
                          uart_printf(s);  
                          break;
            case ERR_I2C: break; // do not print anything 
            default     : break;
        } // switch
        if (ethernet_WIZ550i) // only true after an E1 command
        {
            udp_packet_size = udp_parsePacket();
            if (udp_packet_size)
            {
                udp_read(udp_rcv_buf, UDP_TX_PACKET_MAX_SIZE);
                udp_rcv_buf[udp_packet_size] = '\0';
                ethernet_command_handler((char *)udp_rcv_buf);
            } // if	
        } // if
    } // while()
} // main()