# Autonomous Robot Car Project

## Overview

This project implements an autonomous robotic car using the Raspberry Pi Pico microcontroller with FreeRTOS. The car features both autonomous line-following capabilities and remote control operation, along with advanced sensor integration for navigation and obstacle avoidance.

## Features

- Dual-mode operation:
  - Autonomous line following with barcode scanning
  - Remote control via WiFi
- Real-time obstacle detection and avoidance
- PID-controlled motor system for precise movement
- Wheel encoder integration for accurate distance tracking
- WiFi connectivity for remote control and dashboard monitoring
- Comprehensive sensor suite integration

## Hardware Components

- Raspberry Pi Pico (RP2040)
- L298N Motor Driver
- DC Motors with Wheel Encoders
- IR Sensors for:
  - Line following
  - Barcode scanning
- Ultrasonic Sensor (HC-SR04) for obstacle detection
- WiFi Module for remote connectivity

### Pin Configuration

- **Motor Control**
  - Left Motor: PWM (GP2), DIR1 (GP0), DIR2 (GP1)
  - Right Motor: PWM (GP3), DIR1 (GP8), DIR2 (GP9)
- **Sensors**
  - Line Following IR Sensor: GP27 (ADC)
  - Wheel Encoders: Left and Right encoder pins
  - Ultrasonic Sensor: ECHO_PIN

## Software Architecture

### Core Modules

1. **Main Control (`main/`)**

   - System initialization
   - Task scheduling
   - Core logic coordination

2. **Motor Control (`motor/`)**

   - PID-based speed control
   - Multiple movement modes:
     - Forward/Backward
     - Pivot turns
     - Steering control
   - Speed and direction management

3. **Sensor Integration**

   - **IR Sensors (`ir_sensor/`)**
     - Line following capabilities
     - Barcode scanning functionality
   - **Ultrasonic Sensor (`ultrasonic_sensor/`)**
     - Obstacle detection
     - Safety threshold monitoring
   - **Wheel Encoders (`wheel_encoder/`)**
     - Distance tracking
     - Speed measurement

4. **WiFi Communication (`wifi/`)**

   - Client-server socket implementation
   - Remote control interface
   - Dashboard data transmission

5. **Dashboard (`dashboard/`)**
   - Real-time monitoring
   - Control interface
   - Status display

## Setup Instructions

1. **Prerequisites**

   - Raspberry Pi Pico SDK
   - FreeRTOS
   - CMake build system
   - Development environment (VS Code recommended)

2. **Building the Project**

   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

3. **Hardware Assembly**
   - Connect motors to L298N driver
   - Wire sensors according to pin configuration
   - Ensure proper power supply connections

## Usage Guide

### Remote Control Mode

- Connect to the car's WiFi network
- Use the dashboard interface for control
- Monitor real-time sensor data
- Emergency stop available for safety

### Autonomous Mode

- Place the car on the line track
- System automatically detects line and begins following
- Barcode scanning active during operation
- Obstacle detection ensures safe operation

## Development Notes

### Safety Features

- Obstacle detection threshold implementation
- Emergency stop capabilities
