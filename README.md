# EV Dashboard & ADAS Warning System

## Overview
A real-time Electric Vehicle Dashboard and Advanced Driver Assistance System (ADAS) built using STM32F103C8T6.

## Current Progress
### Day 3 - ADC & UART
- Multi-channel ADC configuration
- Sensor value acquisition
- UART transmission

## Planned Features
- Ultrasonic Distance Detection
- Battery SOC Monitoring
- Motor Temperature Monitoring
- EV Dashboard
- ADAS Warning System

## Hardware
- STM32F103C8T6 (Blue Pill)
- HC-SR04 Ultrasonic Sensor
- LCD Display

## Software
- STM32CubeIDE
- STM32 HAL
- GitHub
- PICSimLab

## Author
Abhiram Kadarla





### Day 4 - Ultrasonic Distance Detection

- HC-SR04 ultrasonic sensor interfacing
- Distance measurement using timer capture
- UART distance transmission
- PICSimLab simulation validation




### Day 5 - Ultrasonic Distance Detection

- HC-SR04 ultrasonic sensor interfacing
- Distance measurement using timer capture for left , right and front
- UART distance transmission
- PICSimLab simulation validation



### Day 6 - UNDERSTANDING TIMERS AND BLINKING LED WITH TIMERS AND INTERRUPTS 

- TIMER  1 is used
- Respective Prescaling and ARR is calculated for required delay
- PICSimLab simulation validation



### Day 7 - EV DASH parameters 

- ev dashboard required parameters are calculated
- speed ,torque ,range ,soc 

 

### Day 8 - 3 Ultaarsonic sensors with Timers 

- front , left and right distances are calculated
- displayed in virtual I/O terminal
- Timers and Interrupt are used
- PICSimLab simulation validation


### Day 9 - EV Dashboard & ADAS Integration

- Integrated EV dashboard parameters with ADAS logic
- Real-time battery SOC monitoring
- Motor temperature monitoring
- Obstacle detection and warning generation
- PICSimLab simulation validation



### Day 10 - Fault Detection & Warning System

- Implemented EV fault detection logic
- Battery low warning
- Motor over-temperature warning
- Obstacle proximity warning
- Fault code generation and UART transmission
- Real-time warning monitoring



### Day 11 - UART Communication

- Optimized UART communication between STM32 and PC
- Real-time transmission of EV dashboard parameters
- Sensor data packet formatting
- Reliable serial communication
- Virtual terminal verification



### Day 12 (Final) - Python Dashboard Integration

- Developed Python-based real-time EV dashboard
- STM32 and Python integration using UART
- Live visualization of:
  - Accelerator position
  - Brake position
  - Battery State of Charge (SOC)
  - Motor temperature
  - Ultrasonic obstacle distances
  - EV fault codes
- End-to-end EV Dashboard & ADAS system validation
- Final project integration completed successfully
