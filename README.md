# Falcon drone based on the STM32F401RE

The Falcon drone is a project aimed to design an autonomous flight controller from the ground up. The project will use an STM32F401RE microcontroller for the flight corrections to achieve stable flight. On top of this I will use a Raspberry Pi 3 microprocessor as an autonomous pilot that will be able to guide send commands to the microcontroller autonomously. There will be a few peripherals that the Pi will use such as a camera so that the drone will follow you and a GPS/GLONASS system to allow a user to select a location for the drone to fly to as well as the ability for a user to take control of the drone via 4G connection, which allowed for nearly unlimited range, depending on battery life.


# Usage

## Build and run
* clone this repository
* connect the NUCLEO board
* `$ make install`
