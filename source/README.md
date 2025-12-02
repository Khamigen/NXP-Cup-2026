# NXP Cup example code for Micro E Board
This is a MCUXpresso project for the Micro E board which has a MK64FN processor. It includes a basic functions for using the DF Robot ROB0167-A chassis with a Micro E Board and a Pixy2 Camera.

## How to use the project
Find the instructions to word with the example code here: [NXP Cup Gitbook](https://nxp.gitbook.io/nxp-cup/developer-guide/development-tools/arc-board-development/programm-the-arc-board)
And you can find the schematics of the board here: [Micro E Board Schematics](https://nxp.gitbook.io/nxp-cup/hardware-reference/arc-board-schematics)

## Functionality of the program
This project includes individual driver software for this custom board. The software is structured in the three layers Interface, Modules and Applications. The interface layer includes all low level driver. In Modules basic functions lke Timer oder ADC/DAC are realised. The Applications layer is thetop layer and includes all functions to control the rover and calculate some additional information. This code is from a participating university and therefore the code was deleted by them before sending us.
The main function (nxpup_ARC_c++.cpp) includes controlling examples for powering the motor with the potentiometers. First the switches has to be in the right position. If Switch 1 is activated, the motors can spin. With the push buttons the servo steering can be tested.
To show an exemplary application of the Pixy camera, an instance was created and the pixy is initialized. After succuessful initializing the RGB-led of the camera module is set to red. Within an endless-loop all lines from the Pixy are requested and printed to the console. This printing function should only be used for debugging because it needs to many resources and makes the system corrupted sometimes. No driving algorithm is included.

The example includes additional functions like using lidar or speed sensors.
