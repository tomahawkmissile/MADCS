# MADCS
Modular Advanced Drone Control System - Version Core-1.0

This repository contains the software stack for the LR-01 hybrid UAV. It acts as the primary control interface for the drone.
It is designed to run as a service on top of a Linux operating system on the core control unit.

The software needs to handle:
- Rotor control
- IMU data
- System-critical voltage rails
- Communication with RADAR transceivers and communication controllers
- Communication with Supervisor Controller

All other system interfaces are routed through the supervisor controller, which can be read by the core control unit via:
- Shared primary CAN bus
- Shared secondary CAN bus
- Direct UART communication (should only be used for critical operations)
- Ethernet (for larger data packets such as file sharing, video feeds, etc.)

Status: CONCEPT
Software progress: 0%
Hardware progress: 25%
