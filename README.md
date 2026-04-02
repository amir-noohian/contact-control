# Contact Control for Barrett WAM

This repository contains C++ implementations of contact-based control strategies for the Barrett WAM robotic arm using `libbarrett`.

The project focuses on:
- Hybrid force/motion control
- External force and torque estimation
- Task-space to joint-space transformations
- Interaction with simple environments (e.g., planar contact)

The code is designed for real-time execution and experimentation on physical WAM hardware.

## Structure
- `src/` – main control implementations  
- `include/` – headers (e.g., Jacobian, wrench estimation)  
- `config/` – robot configuration files  
- `scripts/` – utility scripts (e.g., CAN setup)

## Notes
This is an experimental research codebase for studying contact-rich manipulation and control on torque-controlled manipulators.
