# FPGA Programming Final Project Proposal: FPGA Fan Controller

> Putz Martin (Martin-Pu) 

## Overview

I want to programm the Boolean Board to generate a PWM Signal dependent on the Temperature measured by a ADC. 
A digital interface (SPI/I2C/UART) to the ADC chip will be implented, and connected to the PMod Connector.

## Background

A while back, I developed a Fan Control Board for my Computer, which would control the Fan- and Pumpspeed via the watercoolings loop temperature.
For the control a MicroController was used, but due to the high number of necessary IO Ports (and IO Functions like PWM and interrupts) the choice of Microcontroller was strongly limited. An ESP32 was chosen, due to the embeeded Bluetooth capability for software acces and the relativly high IO count.
Still the IO-Pins (and PWM-Channels) ended up not being enough (10xFans with PWM and Tach Signal, 2x Pumps, Sensors, and so on). Status indication could also not be implemented.

Now that I have the knowledge of programming FPGA's I want to use one of their biggest advantages. 
The seemingly unlimited number of IO-Ports of an FPGA is able to handle unthinkable number of fans and even the most absurd ideas seem feasible. The bluetooth interface can be implemented via the onboard Bluethooth module of the Boolean Board.

This idea could fill a niche market in highend PC watercooling, or other Cooling operations where control over individual Sensors and Coolers is combined in one System.


## Implementation Strategy

Firstly the hardware for the project will be chosen (Sensor & ADC) , and the following segments will be adjusted if necesarry.

The project will be split into the Verilog Modules that will be needed to implement basic functionality of the Fan Controller. For basic funtionality 3-4 Modules are needed. 
  - A bus/communication interface to get Data from the ADC of the Temperature Sensor
  - (Depending on the Data Format of the ADC (Voltage or Temparture) an additional module to convert the Voltage to Temperature will be needed)
  - A module for the actuall control logic, that will generate a Duty Cycle according to the temperature
  - A module to convert the Duty Cycle into an 25kHz PWM Signal (according to standard pc fan specification), which will be the output to the fan

Testbenches will be created to test the functionality of the Code.

For hardwaretests a ciruit to support the ADC will be designed, built on a Breadboard and connected to the boolean board.

<!---Fancy features for user interaction (7 Segement, Buttons) could be implemented, but are not included in the main part of the project.-->


## Tasks
### Core
1. Choose & acquire Hardware
2. Update Strategy and Tasks if necesarry
3. Get information about the used Communication Protokoll and plan FPGA implementation
4. Implement Commumunication Module
(5. Implement Voltage to Temperature Module)
6. Develop & Implement a simple controll algorithm which will output a fan duty cycle
7. Implement the PWM Signal Generater Module
8. Build Testbenches for the individual Modules
9. Combine the Modules in one Programm
10. Hardware Test the combined module

### Stretch Goals: 
11. Implement a Seven Segment Display Output which will rotate between different status information (Temp., Duty Cycle, RPM (see next))
12. Implement a Module to read out the Tach Signal from the fan, calculate the RPM and include it in the status information
13. A state Machine can be implemented, to show the state of the Controller (e.g. High temperature, passive cooling, error, ...) -> not really needed except for error states

The most challenging Part of the project will be the implementation of the communication protokoll. In the worst case, a library can be used.


### Estimated Timeline

**(Core)**

* Task 1 (1 hours)
* Task 2 (0.5 hours)
* Task 3 (1.5 hours)
* Task 4 (10 hours)
* Task 5 (3 hours)
* Task 6 (2 hours)
* Task 7 (2 hours)
* Task 9 (2 hours)
* Task 10 (3hours)

Task 4-7 includes Time for Testbenches and testing

**(Stretch)**

* Task 11 (3 hours)
* Task 12 (4 hours)
* Task 13 (8 hours) 

Progress updates will be posted here.

## Resources
So far no resources have been used. A verilog SPI/I2C Module could be used later.
