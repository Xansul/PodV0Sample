# Pod V0

https://www.nucleic.dev/labs

### Intro:

This repo contains a sample of the V0 prototype for the Pod project. I started this partly because I've been needing a good alarm clock, but mainly because I wanted to grow some practical experience with embedded systems.

This is very much a WIP. There are ideas abound for the project, and they will be gradually implemented. The project is currently running on an STM32 NUCLEO-F446RE [evaluation board](https://www.st.com/en/evaluation-tools/nucleo-f446re.html), powered by the ARM Cortex-M4 STM32F446RE MCU. The codebase has been built thus far using the Eclipse based CubeIDE offered by ST.

A current feature set follows, as well as a list of planned features. Some of it is by no means needed, but I wanted to grow my exposure to as many systems as I could for this project. By that same logic, the codebase is currently setup with some functions being built purely using the CMSIS API and register interactions, and some functions built using ST's HAL API, as I wanted experience with different levels of abstraction.

Files currently included in the repo consist of the core `main.h` and `main.c`, some generic HAL hardware configuration in `stm32f4xx_hal_msp.c`, and a basic customized linker script.

### Feature set:

Current features:
- FreeRTOS implementation - currently built on top of the CMSIS FreeRTOS V2 wrapper.
- Experimental RTC usage - currently unused in favor of timer chain.
- DMA implementation - currently used for DAC conversions, WIP for SPI transfers.
- Misc. timer implementation - currently using two in a master/slave setup for driving the time tracking, as well as one driving DAC conversion timing. Timer ISRs currently increment time trackers, check for alarm matches, and periodically start ADC conversions.
- ADC implementation - currently used for either checking internal temps, or measuring ambient light levels. Light levels will be used for light adjustments, motor actions, etc.
- DAC implementation - currently outputting a sine wave at alarm event for fun, will eventually drive audio.
- USART implementation - currently communicating over serial through the ST-Link/V2 COM port for time updates and debugging messages.
- SPI implementation - currently debugging SPI communication to ILI9341 TFT display module. Setup, transmit functionality, and initial display configuration are all currently there, still a WIP overall.
- Customized linker script - currently setup to allocate specfic location for the FreeRTOS heap, as `new` and `malloc` are overridden to use the FreeRTOS thread-safe implementation.

Planned features:
- Motor control - usage of basic step motors with the ULN2003 driver for movement of the 3D printed housing.
- Further display work - a key part of this project is to give the alarm clock some "personality", this will be primarily implemented through bitmap converted animations on the TFT display module, along with previously listed step motors.
- BLE connectivity - most of the device customization will be ideally implemented with a companion app over BLE. Currently pursuing initial usage with a BlueNRG-M0 module [expansion board](https://www.st.com/en/ecosystems/x-nucleo-idb05a2.html), and a QT powered companion app.
- Audio output - as mentioned above, the current DAC setup will eventually be used to drive simple audio for alarms, interactions, etc.
