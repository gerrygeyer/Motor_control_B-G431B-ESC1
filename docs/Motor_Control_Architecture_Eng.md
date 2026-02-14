# Field-Oriented Control (FOC) with STM32 B-G431B-ESC1

## Overview of Source Code Modules

The motor control firmware is divided into several functional modules:

**`foc.c`** implements the field-oriented control including Clarke and Park transformations.  
**`control.c`** contains the cascaded PI controllers for current and speed (designed according to the technical and symmetrical optimum).  
**`motor_sm.c`** manages the table-driven state machine for the different operating modes.  
**`motor_task.c`** coordinates the time-critical control loops at different priority levels.  
**`svm.c`** generates the PWM signals using Space Vector Modulation.  
**`current_measurement.c`** acquires and processes the phase currents via ADC/DMA.  
**`encoder.c`** provides position and speed information.  
**`observer.c`** enables timing measurements of the control loops.  
**`motor_safety.c`** implements safety features such as automatic shutdown in case of connection loss.  
**`motor_param_est.c`** performs automatic motor parameter identification.  
**`communication.c`** handles UART communication for data transmission and command processing.  
**`overcurrent_overvoltage_protection.c`** monitors critical protection limits.

The helper modules **`motor_ctrl.c`**, **`motor_events.c`**, **`motor_types.c`**, and **`foc_math.c`** provide shared data structures, event handling, and optimized fixed-point mathematical functions.

---

### FOC Overview

(Chapter not yet completed.)

![FOC Structure](./picture_diagrams/FOC_structure.png)

The current control loop is designed using the **technical optimum tuning method**, where the closed-loop transfer function of the controlled system is shaped to behave like a first-order low-pass filter.

Further reading:  
[1] *Control of Electric Machine Drive Systems* | Wiley Online Books  
https://onlinelibrary.wiley.com/doi/book/10.1002/9780470876541  
(Accessed on 08/06/2022)

---

### State Machine

![State Diagram](./picture_diagrams/Ablaufdiagramm.drawio.svg)

---

### Motor Control State Machine

(The implementation is inspired by: https://www.codeproject.com/articles/State-Machine-Design-in-C)

The motor control state machine is implemented as a table-driven state machine with a lightweight kernel.  
The structure `SM_StateMachine` encapsulates the current state and the associated motor instance:

```c
typedef struct SM_StateMachine {
    Motor*  m;
    uint8_t currentState;
    uint8_t newState;
    bool    eventGenerated;
    void*   pEventData;
} SM_StateMachine;
```

`currentState` holds the active state, `newState` is set during transitions, and `eventGenerated` signals to the kernel that an internal event must be processed.  
The pointer `m` allows state functions to directly access the associated `Motor` instance.

During initialization, the motor is transferred into a safe start state:

```c
void MotorSM_Init(Motor* m)
{
    sm.m              = m;
    sm.currentState   = ST_STOP;
    sm.newState       = ST_STOP;
    sm.eventGenerated = false;
    sm.pEventData     = NULL;

    /* safe startup: enter STOP */
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, ST_STOP, NULL);
}
```

---

### States and State Map

The possible states are defined in the `MotorState` enum:

```c
typedef enum {
    ST_STOP = 0,
    ST_CLOSEDLOOP,
    ST_OPENLOOP,
    ST_GOTOSTART,
    ST_PARAMETER_ID,
    ST_FAULT,
    ST_MAX
} MotorState;
```

Each state has its own handler:

```c
static void State_Stop(SM_StateMachine* self, void* eventData);
static void State_ClosedLoop(SM_StateMachine* self, void* eventData);
static void State_OpenLoop(SM_StateMachine* self, void* eventData);
static void State_GotoStart(SM_StateMachine* self, void* eventData);
static void State_ParameterId(SM_StateMachine* self, void* eventData);
static void State_Fault(SM_StateMachine* self, void* eventData);
```

These handlers are registered in a state map:

```c
static const SM_StateStruct MotorStateMap[ST_MAX] = {
    [ST_STOP]         = { State_Stop },
    [ST_CLOSEDLOOP]   = { State_ClosedLoop },
    [ST_OPENLOOP]     = { State_OpenLoop },
    [ST_GOTOSTART]    = { State_GotoStart },
    [ST_PARAMETER_ID] = { State_ParameterId },
    [ST_FAULT]        = { State_Fault }
};
```

The kernel invokes state functions exclusively through this table.

---

### Mini Kernel: Internal and External Events

Internal events schedule a state transition:

```c
static void SM_InternalEvent(SM_StateMachine* self, uint8_t newState, void* eventData)
{
    self->newState       = newState;
    self->pEventData     = eventData;
    self->eventGenerated = true;
}
```

The execution is performed in the state engine:

```c
static void SM_StateEngine(SM_StateMachine* self,
                           const SM_StateStruct* stateMap,
                           uint8_t maxStates)
{
    while (self->eventGenerated)
    {
        self->eventGenerated = false;
        self->currentState   = self->newState;

        if (self->currentState >= maxStates) {
            return;
        }

        SM_StateFunc f = stateMap[self->currentState].fn;
        if (f == NULL) {
            return;
        }

        f(self, self->pEventData);
    }
}
```

---

## Rotor Position Estimation Using a Magnetic Encoder

An MT6701 magnetic encoder operating in ABZ mode is used to measure rotor position.

Position information is provided via two phase-shifted incremental signals (A and B).  
The rotation direction is determined from the edge sequence (A before B or B before A).

The signals are evaluated using a hardware timer configured in encoder mode.  
Depending on the direction, the timer counter is incremented or decremented.

<img src="./picture_diagrams/angle_representation.drawio.svg" alt="angle_representation" width="600">

---

### 16-bit Fixed-Point Angle Representation

The rotor position is represented as a 16-bit cyclic value (`uint16_t` or `int16_t`).  
The value range is mapped cyclically to 0°–360°.

A key advantage is that the natural overflow of the 16-bit counter corresponds to continuous angle wrapping.  
This allows efficient angle arithmetic without additional modulo operations.

At high rotational speeds combined with high sampling rates, apparent outliers may occur during differentiation when crossing overflow boundaries.

---

### Enlarged Counter Space (16× Overscaling)

To improve speed estimation robustness at high rotational speeds, the timer range is increased.

Instead of setting the auto-reload register (ARR) directly to $begin:math:text$N\_\{enc\} \- 1$end:math:text$, it is scaled:

$begin:math:display$
ARR \= 16 \\cdot N\_\{enc\} \- 1
$end:math:display$

Example (for $begin:math:text$N\_\{enc\} \\approx 4000$end:math:text$):

$begin:math:display$
ARR \\approx 16 \\cdot 4000 \- 1 \= 63999
$end:math:display$

This increases internal resolution and improves differentiation behavior across periodic wrap-around.

<img src="./picture_diagrams/Timer_to_rotations.drawio.svg" alt="Timer_to_rotations" width="600">

---

### Rotor Position Calculation

```c
void calc_rotor_position(void){
    encoder_count = encoder_count_large = __HAL_TIM_GET_COUNTER(&htim4);

    uint16_t rotation = encoder_count / ENCODER_PULS_PER_REVOLUTION;
    encoder_count = encoder_count - (rotation * ENCODER_PULS_PER_REVOLUTION);

    uint32_t x = encoder_count_large * (uint32_t)Q16;
    x /= (16 * ENCODER_PULS_PER_REVOLUTION);
    rotor_position = (int32_t)x;

    encoder_count_uint = (encoder_count * count_to_angle); 
    encoder_count = (uint16_t)encoder_count_uint;
}
```

The function:

1. Reads the encoder counter  
2. Normalizes the position within one mechanical revolution  
3. Converts the position to a Q16 fixed-point representation  
4. Computes the electrical angle  

---

### Circle Limitation

(Incomplete)

---

### Parameter Estimation

(Incomplete)