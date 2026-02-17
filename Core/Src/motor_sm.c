/**
  ******************************************************************************
  * @file           : motor_sm.c
  * @brief          : State machine for motor control
  ******************************************************************************
  */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "motor_sm.h"
#include "motor_types.h"
#include "motor_events.h"
#include "motor_param_est.h"
#include "communication.h"

/* External event flags */
// extern volatile bool oc_trip;
// extern volatile bool ot_trip;
// extern volatile bool btn_stop_edge;
// extern volatile bool btn_closed_edge;
// extern volatile bool btn_open_edge;
// extern volatile bool btn_gotostart_edge;

/* ======== Mini SM Kernel  ======== */
typedef struct SM_StateMachine SM_StateMachine;

typedef void (*SM_StateFunc)(SM_StateMachine* self, void* eventData);

typedef struct {
    SM_StateFunc fn;
} SM_StateStruct;

struct SM_StateMachine {
    Motor*  m;             
    uint8_t currentState;
    uint8_t newState;
    bool    eventGenerated;
    void*   pEventData;
};

/* Helper: Zugriff auf Motorinstanz */
#define SM_GetMotor() (self->m)

/* --- Engine --- */
static void SM_InternalEvent(SM_StateMachine* self, uint8_t newState, void* eventData)
{
    self->newState       = newState;
    self->pEventData     = eventData;
    self->eventGenerated = true;
}

static void SM_StateEngine(SM_StateMachine* self, const SM_StateStruct* stateMap, uint8_t maxStates)
{
    while (self->eventGenerated)
    {
        self->eventGenerated = false;
        self->currentState   = self->newState;

        if (self->currentState >= maxStates) {
            /* optional: assert/log */
            return;
        }

        SM_StateFunc f = stateMap[self->currentState].fn;
        if (f == NULL) {
            /* optional: assert/log */
            return;
        }

        f(self, self->pEventData);
    }
}

static void SM_ExternalEvent(SM_StateMachine* self,
                            const SM_StateStruct* stateMap,
                            uint8_t maxStates,
                            uint8_t newState,
                            void* eventData)
{
    if (newState == EVENT_IGNORED) {
        return;
    }
    if (newState == CANNOT_HAPPEN) {
        /* assert/log */
        return;
    }
    if (newState >= maxStates) {
        /* assert/log */
        return;
    }

    SM_InternalEvent(self, newState, eventData);
    SM_StateEngine(self, stateMap, maxStates);
}

/* ======== State-Handler Prototypen ======== */
static void State_Stop(SM_StateMachine* self, void* eventData);
static void State_ClosedLoop(SM_StateMachine* self, void* eventData);
static void State_OpenLoop(SM_StateMachine* self, void* eventData);
static void State_GotoStart(SM_StateMachine* self, void* eventData);
static void State_Fault(SM_StateMachine* self, void* eventData);
static void State_ParameterId(SM_StateMachine* self, void* eventData);

/* ======== State-Map ======== */
static const SM_StateStruct MotorStateMap[ST_MAX] = {
    [ST_STOP]       = { State_Stop },
    [ST_CLOSEDLOOP] = { State_ClosedLoop },
    [ST_OPENLOOP]   = { .fn = State_OpenLoop },
    [ST_GOTOSTART]  = { .fn = State_GotoStart },
    [ST_PARAMETER_ID]= { .fn = State_ParameterId },
    [ST_FAULT]      = { .fn = State_Fault }
};

/* ======== SM-Instanz (pro Motor eine Instanz) ======== */
static SM_StateMachine sm = {0};

/* ======== EventData ======== */
typedef struct {
    int16_t rpm;
} RpmData;

/* ======== Init ======== */
void MotorSM_Init(Motor* m)
{
    sm.m              = m;
    sm.currentState   = ST_STOP;
    sm.newState       = ST_STOP;
    sm.eventGenerated = false;
    sm.pEventData     = NULL;

    /* sicherer Start: in STOP gehen */
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, ST_STOP, NULL);
}

/* ======== Events (Public API) ======== */

void MTR_Stop(Motor* m)
{
    (void)m; 
        static const uint8_t TRANSITIONS[ST_MAX] = {
        [ST_STOP]      = EVENT_IGNORED,
        [ST_CLOSEDLOOP]= ST_STOP,
        [ST_OPENLOOP]  = ST_STOP,
        [ST_GOTOSTART] = ST_STOP,
        [ST_PARAMETER_ID] = ST_STOP,
        [ST_FAULT]     = CANNOT_HAPPEN
    };

    uint8_t next = TRANSITIONS[sm.currentState];
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, next, NULL);
}

void MTR_RunClosedLoop(Motor* m)
{
    (void)m;

    static const uint8_t TRANSITIONS[ST_MAX] = {
        [ST_STOP]      = ST_CLOSEDLOOP,
        [ST_CLOSEDLOOP]= EVENT_IGNORED,
        [ST_OPENLOOP]  = ST_CLOSEDLOOP,
        [ST_GOTOSTART] = EVENT_IGNORED,
        [ST_PARAMETER_ID] = EVENT_IGNORED,
        [ST_FAULT]     = CANNOT_HAPPEN
    };

    uint8_t next = TRANSITIONS[sm.currentState];
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, next, NULL);
}

void MTR_RunOpenLoop(Motor* m)
{
    (void)m;
        static const uint8_t TRANSITIONS[ST_MAX] = {
        [ST_STOP]      = ST_OPENLOOP,
        [ST_CLOSEDLOOP]= ST_OPENLOOP,
        [ST_OPENLOOP]  = EVENT_IGNORED,
        [ST_GOTOSTART] = EVENT_IGNORED,
        [ST_PARAMETER_ID] = EVENT_IGNORED,
        [ST_FAULT]     = CANNOT_HAPPEN
    };

    uint8_t next = TRANSITIONS[sm.currentState];
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, next, NULL);
}


void MTR_GotoStart(Motor* m)
{
    (void)m;

    static const uint8_t TRANSITIONS[ST_MAX] = {
        [ST_STOP]      = ST_GOTOSTART,
        [ST_CLOSEDLOOP]= EVENT_IGNORED,
        [ST_OPENLOOP]  = EVENT_IGNORED,
        [ST_GOTOSTART] = EVENT_IGNORED, 
        [ST_PARAMETER_ID] = EVENT_IGNORED,
        [ST_FAULT]     = CANNOT_HAPPEN
    };
    uint8_t next = TRANSITIONS[sm.currentState];
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, next, NULL);
}


void MTR_ParameterId(Motor* m)
{
    (void)m;
    static const uint8_t TRANSITIONS[ST_MAX] = {
        [ST_STOP]      = ST_PARAMETER_ID,
        [ST_CLOSEDLOOP]= EVENT_IGNORED,
        [ST_OPENLOOP]  = EVENT_IGNORED,
        [ST_GOTOSTART] = EVENT_IGNORED,
        [ST_PARAMETER_ID] = EVENT_IGNORED,
        [ST_FAULT]     = CANNOT_HAPPEN
    };

    uint8_t next = TRANSITIONS[sm.currentState];
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, next, NULL);
}

void MTR_Fault(Motor* m)
{
    (void)m;
    static const uint8_t TRANSITIONS[ST_MAX] = {
        [ST_STOP]      = ST_FAULT,
        [ST_CLOSEDLOOP]= ST_FAULT,
        [ST_OPENLOOP]  = ST_FAULT,
        [ST_GOTOSTART] = ST_FAULT,
        [ST_PARAMETER_ID] = ST_FAULT,
        [ST_FAULT]     = EVENT_IGNORED
    };

    uint8_t next = TRANSITIONS[sm.currentState];
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, next, NULL);
}

/* ======== State-Handler Implementierungen ======== */

static void State_Stop(SM_StateMachine* self, void* eventData)
{
    (void)eventData;
    Motor* m = SM_GetMotor();

    m->state     = ST_STOP;
    m->speed_ref = 0;
}

static void State_ClosedLoop(SM_StateMachine* self, void* eventData)
{
    Motor* m = SM_GetMotor();
    m->state = ST_CLOSEDLOOP;
}

static void State_OpenLoop(SM_StateMachine* self, void* eventData)
{
    Motor* m = SM_GetMotor();
    m->state = ST_OPENLOOP;

}

static void State_GotoStart(SM_StateMachine* self, void* eventData)
{
    (void)eventData;
    Motor* m = SM_GetMotor();
    m->state = ST_GOTOSTART;
}

static void State_ParameterId(SM_StateMachine* self, void* eventData)
{
    (void)eventData;
    Motor* m = SM_GetMotor();
    m->state = ST_PARAMETER_ID;

    MotorParamEst_Init(m);
    /* TODO: Parameter-ID-Mode initialisieren / gewünschte Aktion starten */
}

static void State_Fault(SM_StateMachine* self, void* eventData)
{
    (void)eventData;
    Motor* m = SM_GetMotor();
    m->state = ST_FAULT;
    m->speed_ref = 0;
}


void MotorSM_Service(Motor* m)
{
    if (oc_trip || ot_trip || ov_trip) {
        MTR_Fault(m);
        return;
    }
    if(m->stop_request_flag){
        m->stop_request_flag = false;
        m->start_request_flag = false;
        MTR_Stop(m);
        return;
    }
    if(m->recive_command_flag){
        m->recive_command_flag = false;
        m->speed_ref = get_speed_command();
    }

    if(m->start_request_flag){
        m->start_request_flag = false;
        MTR_RunClosedLoop(m);
    }
    if(m->gotostart_finish_flag){
        // m->gotostart_finish_flag = false; <-- do not reset here
        MTR_Stop(m);
    }

    if (btn_stop_edge) {
        btn_stop_edge = false;
        MTR_Stop(m);
    }
    if (btn_parameter_id_edge) {
        btn_parameter_id_edge = false;
        MTR_ParameterId(m);
    }

    if (btn_closed_edge) {
        btn_closed_edge = false;
        MTR_RunClosedLoop(m);
    }

    if (btn_open_edge) {
        btn_open_edge = false;
        MTR_RunOpenLoop(m);
    }

    if (btn_gotostart_edge) {
        btn_gotostart_edge = false;
        MTR_GotoStart(m);
    }
}