# Arduino-Automatic-RFID-Cat-Feeder
Program to run on an arduino that opens a gate to allow a cat to eat when it approaches with the correct RFID tag on its collar. Useful if you have two or more cats that are on separate diets.
The Arduino Nano with chip ATMEGA328P is used in this program along with the following components:
HC-SR04 Distance Sensor;
125kHz RFID Module (RDM6300) and Tags;
28BYJ-48 Stepper Motor with ULN2003A Driver;
Limit Switches;
LED;
9V 1A Power Supply Connected to Vin of Arduino and Motor.

The program works by getting values from the distance sensor every couple of seconds. If there is an object within range, the system will then try to scan for an RFID tag. If it succesfully reads an RFID tag it will open the gate if the tag matches the stored tag in the EEPROM. The gate then closes once the cat is out of range of the distance sensor.
The button is used to configure a new RFID tag into the EEPROM of the nano, by holding it near the antenna, in case the original one is lost. It can be pressed a second time to cancel this process, otherwise it wil time out after 10 seconds and return to its normal running in case a cat accidently presses the button.
The program uses limit switches to detect when the gate has fully opened or fully closed. On startup the gate is closed until the limit switch is hit and then opens until it is just out of range of the switch. This ensures that the gate closes if the system lost power while the gate was half open.
