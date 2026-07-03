# Day 11 — UART Shell & Complete EV-ADAS System Integration

## Objective

To integrate all previously developed modules into a complete real-time EV and ADAS simulation system and add an interactive UART command shell for testing and debugging.

## Features Implemented

- Real-time EV speed simulation
- Accelerator and brake input using ADC
- Motor torque calculation
- Regenerative braking
- Battery SOC and range estimation
- Motor temperature monitoring
- Three HC-SR04 ultrasonic sensors
- Forward Collision Warning (FCW)
- Time-To-Collision (TTC) calculation
- Blind Spot Detection (BSD)
- Alarm priority system
- Fault detection and handling
- UART telemetry output
- Interactive UART command shell

## Hardware / Simulation

- STM32F103C8T6 (Blue Pill)
- PICSimLab
- 3 × HC-SR04 Ultrasonic Sensors
- 4 × Analog Input Potentiometers
- Virtual UART Terminal
- LEDs for warning and fault indication


## System Flow

ADC Inputs  
↓  
EV Dynamics Model  
↓  
Speed, Torque, SOC and Temperature  
↓  
Ultrasonic Sensor Measurements  
↓  
ADAS Processing (FCW, TTC and BSD)  
↓  
Fault and Alarm Management  
↓  
UART Telemetry and Command Shell  
