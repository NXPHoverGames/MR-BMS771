# MR-BMS771 | RDDRONE-BMS772
NuttX source code for BMS example application using the MR-BMS771 all-in-1 BMS board (7-14 cells) .
Can be used with the RDDRONE-BMS772 board (3-6 cells) as well.

Please download and read the release notes (version 7.0 or higher) to see all the needed information on the software and the board, like how to start it up.

## short description of the code
The C/C++ based RTOS BMS example code provides an example on how to use the MC33771C BCC IC with the S32K146 MCU for an all-in-one BMS solution.
As it is a MCU based solution. You can configure this BMS example to be tuned for the wanted cell type and configuration if the hardware supports this.
It can measure all cell voltages, BMS input and BMS output voltage, the current as well as coulomb counting for better SOC estimation and 1 (up to 3) external temperature sensors can be measured as well as 3 on-board temperature sensors.
This SW example provides basic configurable protections (OV, UV, OT, UT, OC) and will control the build in power switch to protect the battery.
It shows a simple way balancing the cells while charging. 
Furthermore this example will calculate and communicate all kind of parameters, like State of Charge (SOC) and State of Health (SOH).

For communication with this BMS board, a couple example have been programmed. 
CyphalCAN or DroneCAN protocol can be used to configure the BMS or provide the needed information.
UART can be used to configure the BMS via a console or retrieve the needed information.
SMBus can be used to get the information from the BMS.
There is an example on how to use the NTAG5 NFC IC to be able to tap the antenna with a phone to read out some data.

## Disclaimer - CAUTION WARNING
### CAUTION - WARNING
Lithium and other batteries are dangerous and must be treated with care. 
Rechargeable Lithium Ion batteries are potentially hazardous and can present a serious FIRE HAZARD if damaged, defective or improperly used. Larger Lithium batteries and those used for industrial use involving high discharge current and frequent full discharge cycles require special precautions.

Do not connect this BMS to a lithium ion battery without expertise and  training in handling and use of batteries of this type.
Use appropriate test equipment and safety protocols during development. 

### Important Notice 
NXP provides the enclosed product(s) under the following conditions: 
This reference design is intended for use of ENGINEERING DEVELOPMENT OR EVALUATION PURPOSES ONLY. It is provided as a sample IC pre-soldered to a printed circuit board to make it easier to access inputs, outputs, and supply terminals. This reference design may be used with any development system or other source of I/O signals by simply connecting it to the host MCU or computer board via off-the-shelf cables. Final device in an application will be heavily dependent on proper printed circuit board layout and heat sinking design as well as attention to supply filtering, transient suppression, and I/O signal quality. 
The goods provided may not be complete in terms of required design, marketing, and or manufacturing related protective considerations, including product safety measures typically found in the end product incorporating the goods. 
Due to the open construction of the product, it is the user's responsibility to take any and all appropriate precautions with regard to electrostatic discharge. In order to minimize risks associated with the customers applications, adequate design and operating safeguards must be provided by the customer to minimize inherent or procedural hazards. For any safety concerns, contact NXP sales and technical support services. Should this reference design not meet the specifications indicated in the kit, it may be returned within 30 days from the date of delivery and will be replaced by a new kit. 
NXP reserves the right to make changes without further notice to any products herein. NXP makes no warranty, representation or guarantee regarding the suitability of its products for any particular purpose, nor does NXP assume any liability arising out of the application or use of any product or circuit, and specifically disclaims any and all liability, including without limitation consequential or incidental damages. 
Typical parameters can and do vary in different applications and actual performance may vary over time. All operating parameters, including Typical, must be validated for each customer application by customer’s technical experts. 
NXP does not convey any license under its patent rights nor the rights of others. NXP products are not designed, intended, or authorized for use as components in systems intended for surgical implant into the body, or other applications intended to support or sustain life, or for any other application in which the failure of the NXP product could create a situation where personal injury or death may occur. Should the Buyer purchase or use NXP products for any such unintended or unauthorized application, the Buyer shall indemnify and hold NXP and its officers, employees, subsidiaries, affiliates, and distributors harmless against all claims, costs, damages, and expenses, and reasonable attorney fees arising out of, directly or indirectly, any claim of personal injury or death associated with such unintended or unauthorized use, even if such claim alleges NXP was negligent regarding the design or manufacture of the part.

## sources
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
git checkout nuttx-11.0.0 -b bms771
```
Add the nuttx patch.
```bash
patch -p1 < ../apps/nxp_bms/BMS_v1/Patchfiles/0001-nuttx-patch-BMS7.0-MR-BMS771.patch
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
Make sure the BMS is powered and everything is connected properly to the BMS.

Use a UART terminal like minicom on a linux machine or PuTTY or teraTerm on a windows machine and connect to the right COM port.
The settings are:
*	115200 Baud
*	8 data bits
*	1 stop bit

## Programming the BMS with the JLink debugger
See the release notes of the BMS771 or the BMS772 how to attach the debugger.

To program the BMS using a JLink debugger you need to have JLink installed.
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
