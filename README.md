# RC receiver
------
### Revision 0

This version only supports S.BUS protocol which is used by controllers
including Futaba R7008SB.

#### How it works
This device takes S.BUS protocol which has non-standard baudrate and uses inverted signal.
Then, it converts the signal into standard UART packets and transmits them to HCU via RS232.
Install the dependencies and devDependencies and start the server.

-------
### Revision 1
This version supports 6 channel PWM signals from RC controllers as well as S.BUS protocol.
Also, it can take standard RS232 protocol from external controller(that being its duplicate)
to override controlling signal.



#### How it works
Only one controller (6-channel PWM / S.BUS) can be installed onto this module,
but an external controller can be connected at any situation.

Priority of controllers are as following:
1. External controller
2. S.BUS controller
3. 6-channel PWM controller

This module will detect which controller is installed and take signal from the controller.
Then it will convert it to the standard packets and transmit it to HCU.
