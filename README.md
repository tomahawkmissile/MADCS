# MADCS
Modular Advanced Drone Control System - Version Core-1.0

**INCOMPLETE**

This repository contains the software stack for the LR-01 hybrid UAV. It acts as the primary control interface for the drone.
It is designed to run as a service on top of a Linux operating system on the core control unit.
Currently, the code is mainly just display and some sensor drivers, and its is meant to run on a Raspberry Pi Compute Module 3 for testing. When the Core Control Unit PCB is done, I will port the code to work with the Arm Cortex A8 and its peripherals.

Core Control Unit Specifications:
- Custom 10-layer PCB
- TI-AM3354 Arm Cortex A8
- 512MB DDR3 RAM
- 2x16GB eMMC flash
- Gigabit Ethernet
- 2x USB 2.0
- HDMI transceiver
- Application connector (contains numerous IO pins)

Control Unit progress: 100% (awaiting manufacture)

The software needs to handle:
- Rotor control
- IMU data
- System-critical voltage rails
- Communication with RADAR transceivers and communication controllers
- Communication with Supervisor Controller
- Primary coolant loop temperature

All other system interfaces are routed through the supervisor controller, which can be accessed by the core control unit via:
- Shared primary CAN bus
- Shared secondary CAN bus
- Direct UART communication (should only be used for critical operations)
- Ethernet (for larger data packets such as file sharing, video feeds, etc.)

Status: CONCEPT
- Software progress: 0%
- Hardware progress: 33%

**Repository will be updated shortly with data transferred from the main private repository.**
**I do not document my code and I also compact it, so it may be difficult to read**
