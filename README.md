# Motor Control – STM32 B-G431B-ESC1

## Overview
Field-Oriented Control (FOC) implementation for a PMSM using the STM32G431
and the ST B-G431B-ESC1 evaluation board.

## Features
- Field-Oriented Control (Clarke/Park, SVM)
- Cascaded PI current and speed control
- Table-driven state machine
- Encoder-based position and speed estimation (Q16)
- On device Motor parameter identification (not finish yet)
- Safety mechanisms (OC/OV/OT, fault handling)

## Hardware
- STM32G431
- B-G431B-ESC1
- Magnetic encoder (ABZ mode)

## Software Architecture
The motor control firmware is structured into functional modules:
- `foc.c` – FOC core (Clarke/Park, inverse transforms)
- `motor_sm.c` – Table-driven state machine
- `motor_task.c` – Real-time task coordination
- `svm.c` – Space Vector Modulation
- `encoder.c` – Position and speed estimation
- `motor_safety.c` – Fault handling and protection

Detailed explanations can be found in `/docs`.

## Build & Flash
- STM32CubeIDE
- GCC ARM
- Tested with B-G431B-ESC1

## Status
Work in progress – some modules (parameter estimation)
are not fully implemented yet.

## License
MIT





