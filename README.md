CNC Control Panel: Stepper MCU firmware
=======================================

(C) 2012 Radu - Eosif Mihailescu <radu.mihailescu@linux360.ro>
Hereby licensed under the GNU Generic Public License v3, a copy of whose text
can be accessed here: http://www.gnu.org/licenses/gpl.html

This repository contains the source of the firmware that goes on the Stepper MCU
in the CNC Control Panel project.

The current incarnation of the Stepper MCU is three Atmel ATtiny84s SPI daisy-
chained and ran as a single SPI slave by the Main MCU.

!!!==WARNING--WARNING--WARNING==!!!
Unless and until explictily noted otherwise, this code is laboratory quality and
not intended for usage in connection with actual moving parts such as stepper
motors. Should you choose to test or use this software in a live environment,
you do so entirely at your own risk.