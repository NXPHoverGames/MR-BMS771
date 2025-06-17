# MR-BMS771 | RDDRONE-BMS772
This is NuttX source code for a BMS example application using the MR-BMS771 all-in-1 BMS board (7-14 cells) .
Can be used with the RDDRONE-BMS772 board (3-6 cells) as well.

> [!TIP]
> Please download and read the release notes (version 7.0 or higher) to and carefully review all information on the software and the board setup.

> [!IMPORTANT]
> the board as-shipped requires soldering of connectors to configure the appropriate cell count connections.

> [!NOTE]
> To power up the BMS771 or to cycle power  on the MCU, press the small button (SW1) on top side of the MR-BMS771 board.

SW1 is located diagonally across the POWER_IN (J4). 
With the default software configuration the cell under voltage protection (v-cell-uv) is active. This will automatically power down the MCU and the board after 60 seconds when any of the 14 cell measurements are low (Vcell < 3V).
The user may press the button SW1 again to re-power the MCU or to reset this countdown. This protection can be disabled.

See the quickstart guide and helpfull commands at the bottom of this readme

## Short description of the code
The C/C++ based RTOS BMS example code provides an example on how to use the MC33771C BCC IC with the S32K146 MCU for an all-in-one BMS solution.
As it is a MCU based solution, you can configure this BMS example to be tuned for the desired cell chemistry type and other configuration the hardware supports.
The BMS can measure all cell voltages, input and output voltage, the current as well as coulomb counting for better SOC estimation and 1 to 3 external temperature sensors can be measured in addtion to 3 on-board temperature sensors.
The software example provides basic configurable protections (OV, UV, OT, UT, OC) and will control the built-in power switch to protect the battery.
It also allows for passive balancing of the cells while charging. 
Furthermore this example will calculate and communicate a wide variety of parameters, such as State of Charge (SOC) and State of Health (SOH).
Given the software is opensource, you may wish to develop your own more advanced SOH algorithms.
NOTE: Contact NXP sales directly to access additional non-opensource libraries from NXP for the BCC chip which may be used in advanced applications.

For communication with this BMS board, a three examples have been provided. 
- CyphalCAN or DroneCAN protocol can be used to configure the BMS or provide the needed information.
- UART can be used to configure the BMS via a console or retrieve the needed information.
- SMBus can be used to get the information from the BMS.
- There is also an example on how to use the NTAG5 NFC IC (when populated) to be able to tap the antenna with a phone to read out some data.


> [!CAUTION]
> ## Disclaimer - CAUTION WARNING
> ### CAUTION - WARNING
> Lithium and other batteries are dangerous and must be treated with care. 
> Rechargeable Lithium Ion batteries are potentially hazardous and can present a serious FIRE HAZARD if damaged, defective or improperly used. Larger Lithium batteries and those used for industrial use involving high discharge current and frequent full discharge cycles require special precautions.
>
> Do not connect this BMS to a lithium ion battery without expertise and  training in handling and use of batteries of this type.
> Use appropriate test equipment and safety protocols during development.

> [!IMPORTANT]
> ### Important Notice 
> NXP provides the enclosed product(s) under the following conditions: 
> This reference design is intended for use of ENGINEERING DEVELOPMENT OR EVALUATION PURPOSES ONLY. It is provided as a sample IC pre-soldered to a printed circuit board to make it easier to access inputs, outputs, and supply terminals. This reference design may be used with any development system or > other source of I/O signals by simply connecting it to the host MCU or computer board via off-the-shelf cables. Final device in an application will be heavily dependent on proper printed circuit board layout and heat sinking design as well as attention to supply filtering, transient suppression, > and I/O signal quality. 
> The goods provided may not be complete in terms of required design, marketing, and or manufacturing related protective considerations, including product safety measures typically found in the end product incorporating the goods. 
> Due to the open construction of the product, it is the user's responsibility to take any and all appropriate precautions with regard to electrostatic discharge. In order to minimize risks associated with the customers applications, adequate design and operating safeguards must be provided by the customer to minimize inherent or procedural hazards. For any safety concerns, contact NXP sales and technical support services. Should this reference design not meet the specifications indicated in the kit, it may be returned within 30 days from the date of delivery and will be replaced by a new kit. 
> NXP reserves the right to make changes without further notice to any products herein. NXP makes no warranty, representation or guarantee regarding the suitability of its products for any particular purpose, nor does NXP assume any liability arising out of the application or use of any product or circuit, and specifically disclaims any and all liability, including without limitation consequential or incidental damages. 
> Typical parameters can and do vary in different applications and actual performance may vary over time. All operating parameters, including Typical, must be validated for each customer application by customer’s technical experts. 
> NXP does not convey any license under its patent rights nor the rights of others. NXP products are not designed, intended, or authorized for use as components in systems intended for surgical implant into the body, or other applications intended to support or sustain life, or for any other application in which the failure of the NXP product could create a situation where personal injury or death may occur. Should the Buyer purchase or use NXP products for any such unintended or unauthorized application, the Buyer shall indemnify and hold NXP and its officers, employees, subsidiaries, affiliates, and distributors harmless against all claims, costs, damages, and expenses, and reasonable attorney fees arising out of, directly or indirectly, any claim of personal injury or death associated with such unintended or unauthorized use, even if such claim alleges NXP was negligent regarding the design or manufacture of the part.

## Sources
* To view the design files and the product of the MR-BMS771 (7-14 cells) on the NXP webpage see https://www.nxp.com/design/design-center/development-boards-and-designs/MR-BMS771
* To view the design files and the product of the RDDRONE-BMS772 (3-6 cells) on the NXP webpage see https://www.nxp.com/design/design-center/development-boards-and-designs/smart-battery-management-for-mobile-robotics-3-6-cells:RDDRONE-BMS772. 
* To view the gitbook of the RDDRONE-BMS772 (3-6 cells) see https://nxp.gitbook.io/rddrone-bms772/.
  
NXP has battery emulators that may be used during testing:
https://www.nxp.com/design/development-boards/analog-toolbox/6-cell-battery-pack-to-supply-mc33772-evbs:BATT-6EMULATOR.
https://www.nxp.com/design/design-center/software/analog-expert-software-and-tools/sdk-analog-expert-drivers/14-cell-battery-pack-emulator-to-supply-mc33771c-bcc-evbs:BATT-14CEMULATOR

This readme files will explain how to get the right nuttx and nuttx-apps repository with the BMS7.0 patches and build the BMS software (create a binary file).
This will work best on a linux machine, you could use a virtual machine for it.

See this webpage for the NuttX quickstart guide: https://nuttx.apache.org/docs/10.0.0/quickstart/quickstart.html

### MBDT 
NXP's Model-Based Design Toolbox (MBDT) enables rapid prototyping of complex algorithms on NXP hardware from MATLAB/Simulink.
See https://www.nxp.com/design/design-center/software/automotive-software-and-tools/model-based-design-toolbox-mbdt:MBDT for more information.

Both BMS boards are supported in MBDT, for the MBDT bringup examples see:
* BMS771: https://community.nxp.com/t5/NXP-Model-Based-Design-Tools/MR-BMS771-Demo-Application/ta-p/1919177
* BMS772: https://community.nxp.com/t5/NXP-Model-Based-Design-Tools/Example-Model-RDDRONE-BMS772/ta-p/1550394 

There should be a full BMS example on nxp.com.

## Make sure git and zip are installed
If git is not installed, open a terminal and type the following commands:
```bash
sudo apt-get update
```
```bash
sudo apt-get install git
```
```bash
sudo apt-get install zip
```

## Get the incubator nuttx and apps
Make a usefull folder to place the files in, like drones.
```bash
mkdir -p drones
```
```bash
cd drones
```

Clone the nuttx and nuttx apps git repositories.
```bash
git clone https://github.com/apache/nuttx.git nuttx
```
```bash
git clone https://github.com/apache/nuttx-apps.git apps
```
## Checkout the right commit
In the apps directory, checkout the right commit and branch.
```bash
(cd apps; git checkout nuttx-11.0.0 -b bms77x)
```
## Get the BMS in the nuttx-apps
Make a nxp_bms folder in the apps directory and clone the BMS_v1 repository in that folder
```bash
mkdir -p apps/nxp_bms
```
Clone this repository in that folder
```bash
(cd apps/nxp_bms; git clone https://github.com/NXPHoverGames/MR-BMS771.git BMS_v1)
```
Checkout the public regulated data types.
```bash
(cd apps/nxp_bms/BMS_v1; git clone https://github.com/px4/public_regulated_data_types; cd public_regulated_data_types; git checkout 78b883891a582ff0eb41375433247b5ca0d44d21)
```
## Apply the patches
Apply the patch to the nuttx-apps.
```bash
(cd apps; patch -p1 < nxp_bms/BMS_v1/Patchfiles/0001-apps-patch-BMS7.0.patch)
```
Go the nuttx folder and checkout the right NuttX commit.
```bash
cd nuttx
```
```bash
git checkout nuttx-11.0.0 -b bms77x
```
Add the nuttx patch.
```bash
patch -p1 < ../apps/nxp_bms/BMS_v1/Patchfiles/0001-nuttx-patch-BMS7.0.patch
```
## Install the Kconfig tools and the crosscompiler if needed
When this is your first NuttX project, you need to install the Kconfig tools and the cross compiler. Otherwise you can skip this part and continue with "configure and make the binary".
### Install the Kconfig tools if needed
Install the build essentials and everything that is needed.
```bash
sudo apt-get install build-essential
```
```bash
sudo apt-get install flex
```
```bash
sudo apt-get install bison
```
```bash
sudo apt-get install gperf
```
```bash
sudo apt-get install libncurses5-dev
```
Install the kconfig-frontends. If this doesn't work, see below 'Installing kconfig-frontends'
```bash
sudo apt-get install kconfig-frontends
```

### Install the cross compiler if needed
Tested with version 9.3.1.
If not already installed, install the cross compiler with the following command:
```bash
sudo apt install gcc-arm-none-eabi
```

## Configure and make the binary
Configure for the MR-BMS771 board.
For normal use:
```bash
tools/configure.sh -e mr-bms771:bms
```
For debug purposes BMS771:
```bash
tools/configure.sh -e mr-bms771:bmsdebug
```
Or configure for the RDDRONE-BMS772 board.
For normal use:
```bash
tools/configure.sh -e rddrone-bms772:bms
```
For debug purposes BMS772:
```bash
tools/configure.sh -e rddrone-bms772:bmsdebug
```

Make the binary with: 
```bash
make
```
## Using the command line interface (CLI) of the BMS
Make sure the BMS is powered and everything is connected properly to the BMS like the TTL-232R-3V3.
Press the push button SW1 on top layer (diagonal across the POWER_IN (J4)) to (re)power the MCU.

Use a UART terminal like minicom on a linux machine or PuTTY or teraTerm on a windows machine and connect to the right COM port.
The settings are:
*	115200 Baud
*	8 data bits
*	1 stop bit

## Programming the BMS with the JLink debugger
See the release notes of the BMS771 or the BMS772 how to attach the debugger.
To program the BMS using a JLink debugger you need to have JLink installed. In this example the J-Link BASE debugger probe is used with an adapter board.

Make sure the BMS is powered.

Open a terminal where the nuttx.bin file is located (probably in the nuttx folder).
Open JLink:
```bash
JLinkExe
```
Connect to it:
```bash
connect
```
Enter the correct device (S32K146 for BMS771 and S32K144 for BMS772):
```bash
S32K146
```
Program it using SWD:
```bash
s
```
Use 1000kHz as target interface speed:
```bash
1000
```
Reset the device:
```bash
r
```
Reset the entire flash by sending these commands:
```bash
w1 0x40020007, 0x44     
```
```bash
w1 0x40020000, 0x80    
```
Load the nuttx binary at address 0
```bash
loadbin nuttx.bin 0
```
Reset the device
```bash
r
```
Run the program
```bash
g
```
Quit the JLinkExe 
```bash
q
```
## Installing kconfig-frontends
If the installation of kconfig-frontends did not work:
Go back from the nuttx directory and make a tools directory next to it.
From the nuttx directory:
```bash
cd ..
```
```bash
mkdir -p tools
```
```bash
cd tools
```
Clone the nuttx tools master.
```bash
git clone https://bitbucket.org/nuttx/tools/src/master
```
```bash
cd master
```
Apply the patch.
```bash
patch -p1 < ../../apps/nxp_bms/BMS_v1/Patchfiles/0001-aclocal-fix.patch
```
Configure and install it.
```bash
cd kconfig-frontends
```
```bash
./configure --enable-mconf --disable-nconf --disable-gconf --disable-qconf
```
```bash
make
```
```bash
sudo make install
```
```bash
sudo ldconfig
```

## Quick start guide
This chapter describes the things to consider when first starting or using the BMS771 HW and SW. This can be seen as a quick start guide.
For a full description, please read the release notes.

### Configure the HW
Start with configuring the HW.
1.	Solder the correct cell count selection board for the correct cell count. 
2.	Solder the correct balance header (JP1) for the correct cell count.
a.	Keep in mind, pin 15 is ground.
3.	Check how to connect the battery (power in) and rest of the system (power out) to the BMS771 board.
a.	You could solder the provided XT90 power connectors. 
4.	Check to connect the temperature sensor of the battery.
a.	You could add the provided NTC temperature sensor cable and connect this to J1.
> [!NOTE]
> you need to enable the temperature sensor in the SW (bms set sensor-enable 1). 
5.	Check to add a UART or (Drone)CAN connection to configure the BMS771.
a.	The UART connection is most easy way to configure the BMS parameters. 
b.	This can also be done via CAN using the DroneCAN protocol and the DroneCAN GUI tool.
6.	Check to add the provided display to the BMS771

### Power the BMS771
It is always advisable to start with a current limiting power supply (15-60V, e.g. 24V 100mA).
> [!NOTE]
> Keep in mind you need to press the button to start the BMS771 and (re)power the MCU.

With the default software configuration the cell under voltage protection (v-cell-uv) is active. This will automatically power down the MCU and the board after 60 seconds when any of the 14 cell measurements (Vcell < 3V).

### program the latest SW
It is always advisable to use the latest version of the software.
You can either program the nuttx.bin binary from the Binary folder or build and create the nuttx.bin using the explanation above.

### Configure the BMS771 parameters
See the release notes for the full list configurable parameters.

#### The most important BMS parameters
> [!IMPORTANT]
> Do not forget to save the parameter with the “bms save” command.

The most important variables to look at are:

* n-cells 
    * the amount of cells of the battery [8 ... 14]
> [!CAUTION]
> This should reflect the soldered configuration.
> solder the correct cell select board.

*	sensor-enable
    *	To enable the battery temperature sensor [0=disable, 1=enable]
*	battery-type
    *	Which battery type do you have?
[0=LiPo, 1=LiFePO4, 2=LiFeYPO4, 3=NMC, 4=Sodium-ion]
    *	This will change the under- and over-voltage, storage voltage, nominal voltage and the OCV curve it uses to correct the state of charge.
*	a-rem
    *	The remaining capacity [Ah]
    *	The BMS does an estimate on what the remaining capacity is based on an OCV(open cell voltage)/SoC (state of charge) table from one specific battery. Keep in mind that every battery is different.
    *	It is advisable to insert or adjust the correct OCV/SoC table for the used battery.
        *	This is done in the open-source code in bcc_monitoring.c.
    *	You can correct the state of charge (SoC) with this variable.
*	a-factory
    *	The factory capacity of the battery [Ah]
    *	This should be the capacity stated on the battery.
*	a-full
    *	The full charge capacity of the battery [Ah]
    *	This degrades over time. Less capacity can be stored in the battery.
    *	State of health is calculated based on this value.
    *	If unknown, set to the same value as factory capacity.
*	model-name
    *	The name of the battery
*	i-charge-full
    *	The end of charge current of the battery (could be 10% from i-charge-max) [mA]
*	i-charge-max 
    *	The maximum charge current [A]
*	i-peak-max 
    *	Maximum peak current threshold [A]





