# GNI-r3 Rotator Controller
The GNI-r3 rotator controller supports one rotor. It is designed for rotators of the type "RAU", "RAK" and similar (with a pulse every 1 degree). The rotation of the antenna is possible with a resolution of 1 degree in the range of 0-360 degrees with some reserve in each direction.

The current value of the antenna position is written into the GNI-r3 controller’s EEPROM non-volatile memory when the final value is reached, or when the "Stop" button is pressed.

The rotator controller GNI-r3 is powered from the computer's USB connector via the USB-mini cable and the USB-mini connector on the back panel. The current drawn from the USB output is typically 45 mA.

The rotor motor power supply given to the 2.1/5.5 socket can range from 12V to 24V. The motor power ground has galvanic separation from the GNI-r3 controller and computer ground.

The upper line of the display indicates by default the current azimuth of the antenna in the range of 0-359 degrees, the bottom line displays the current location of the antenna in the range defined by the lower and upper limits (for example -180 to +540 degrees).

The LED1 (green) flashes with every pulse from the rotor pulser (every 1 degree). It lights continuously after each use of the "Stop" button, as well as: after a successful calibration of the antenna setting, after switching off the display backlight, and when the lower or upper rotation limit is reached. This backlight automatically turns off after (by default) 3 minutes of inactivity. 

To manually control the rotor, three front panel buttons of the GNI-r3 rotator controller are used: "+" (start of rotation clockwise), "-" (start of rotation counter-clockwise) and "Stop" (immediate stop of the rotor after reaching the desired position or for other reasons). 

The "Stop" button also has the function of calibration, i.e. setting the initial position (zeroing) in any physical position of the antenna. If the "Stop" button is pressed for a few seconds when the power is turned on or the microcontroller is reset, the antenna's azimuth values and its actual position will be set to 180 degrees (direction to the south).

Controlling from a computer is possible using any program compatible with the AlfaSpid protocol. The GNI-r3 rotator controller has been checked with the following programs: DXView (part of DX Lab), HRD, N1MM Rotor, PstRotator, Spid Driver,  as well as with the Alfa Radio test program (ALFAMD.py).

When configuring the software, select the AlfaSpid protocol in the appropriate program location. Then indicate the correct COM port, and set: 1200 baud, 8 bits, 1 stop bit, no parity.

A rotation request below 3 degrees is ignored. The dead zone can be changed in the source code of the program in Arduino.

Warning - each activation of the COM port on the computer resets the microcontroller! This is not dangerous except when the antenna is currently in motion. Then the loss of information about the position of the antenna occurs, which can be dangerous for the coaxial cable. RAU/RAK rotors do not have any mechanical limit switch, but only software.

Schematic, layout, user manual and source code are available for free. Commercial use of this software, and associated documentation is not allowed without written permission from Miroslaw Sadowski SP5GNI.

Find the schematic "ENG-SP5GNI-r3 rotator controller schematic.pdf" and User Manual "ENG-SP5GNI-r3 rotator controller manual.pdf" at https://vot.pzk.org.pl/pliki/category/19-osprzet-stacji 

Look also at https://www.qrz.com/db/SP5GNI
