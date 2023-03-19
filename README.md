# Home-Brewery Automation
This is the firmware for the STM8S207 microcontroller on the Brew hardware PCB.

![frontpanel](img/brew_hw_stm8s207_v401.png)<br>
*Top view of v4.01 prototype PCB*

# Features
The current PCB and firmware have the following features:
- Reading a maximum of **6 x temperature sensors**: one each for the three brewing-kettles, one at the output of the counterflow-chiller, one at the entry of the MLT return-manifold and a spare one that 
  can be used as a backup for the HLT or MLT sensor. Both one-wire (4) and I2C sensors (2) are supported.
- Reading of the hardware temperature: this is to protect the Solid State Relays (SSR) from overheating.
- Reading a maximum of **4 x flowsensors**: between HLT and MLT, between MLT and boil-kettle, one at the output of the counterflow-chiller and one at the entry of the MLT return-manifold.
- Control of **8 x solenoid ball-valves** at 24 V DC.
- Control of **4 x SSR at 230 V AC**: the pump, a second pump (for the HLT counterflow-chiller), 230 V enable for the HLT gasburner and 230 V enable for the boil-kettle gasburner.
- **2 x PWM signals (25 kHz, 28 V DC)** for the modulating gasburners (HLT and boil-kettle burners).
- **2 x On/Off signal (24 V AC)** for two non-modulating gasburners (HLT and boil-kettle burners).
- **6 x SSR outputs** for three phase heating-elements in HLT and boil-kettle. With this, heating of the HLT is possible with a gas-burner, with 1, 2 or 3 heating-elements of with any combination of these.
- Synced SSR outputs for HLT and boil-kettle, so that you only need one three-phase connection. The boil-kettle outputs have priority over the HLT outputs, meaning that if (on any phase of 230 V) both HLT and boil-kettle are heating, only the boil-kettle is activated.
- Ethernet and USB connection to PC: USB-connection is used for debugging, main connection between PC-program and the firmware is Ethernet.

More hardware and firmware design details: see my website: http://www.vandelogt.nl/uk_hardware.php

# Software Development Environment
Use with IAR-STM8.

# Interface with PC
The Brew-Hardware uses a network cable as its main-connection to the PC. This is powered through a WIZ550IO module. It uses DHCP to obtain an IP address automatically.
The standard port number for the firmware is set to 8888. 

The Brew-hardware also uses two virtual COM ports as a backup connection to the PC. Virtual COM port settings are (115200,N,8,1).

Typically the PC-program sends commands to the Brew-Hardware, like **P0** (Pump Off) or **P1** (Pump On). These commands are then executed by the firmware of the Brew-Hardware.
Although you can type in the commands manually, it is more efficient to use a dedicated PC-program for it, with a Graphical User Interface.

If you install the Brew-Hardware PCB for the first time, it is best to use a terminal program that can handle a virtual COM port (I had good results with Realterm). At initial power-up, you can type **S0** to retrieve
the version number of the firmware (and to check that everything is working). The **E1** command enabled the Ethernet module, if everything goes well, the IP address of the Brew-Hardware is returned.

More information about the PC-Interface can be found at my website: http://www.vandelogt.nl/uk_hardware.php#PC_INTF



