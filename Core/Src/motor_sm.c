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

/* External event flags */
extern volatile bool oc_trip;
extern volatile bool btn_stop_edge;
extern volatile bool btn_closed_edge;
extern volatile bool btn_open_edge;
extern volatile bool btn_gotostart_edge;

/* ======== Mini SM Kernel (wie endurodave, gekürzt) ======== */
typedef struct SM_StateMachine SM_StateMachine;

typedef void (*SM_StateFunc)(SM_StateMachine* self, void* eventData);

typedef struct {
    SM_StateFunc fn;
} SM_StateStruct;

struct SM_StateMachine {
    Motor*  m;             // pInstance direkt typisiert
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
    if (newState >= maxStates) {
        /* optional: assert/log */
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

/* ======== State-Map ======== */
static const SM_StateStruct MotorStateMap[ST_MAX] = {
    [ST_STOP]       = { State_Stop },
    [ST_CLOSEDLOOP] = { State_ClosedLoop },
    [ST_OPENLOOP]   = { .fn = State_OpenLoop },
    [ST_GOTOSTART]  = { .fn = State_GotoStart },
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
    (void)m; // wir nutzen die globale sm-Instanz; alternativ pro Motor eigene sm
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, ST_STOP, NULL);
}

void MTR_RunClosedLoop(Motor* m)
{
    (void)m;
    static RpmData d = { .rpm = 3000 };      // Default; später gern parametrieren
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, ST_CLOSEDLOOP, &d);
}

void MTR_RunOpenLoop(Motor* m)
{
    (void)m;
    static RpmData d = { .rpm = 3000 };      // Default; später gern parametrieren
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, ST_OPENLOOP, &d);
}

void MTR_RunSensorless(Motor* m)
{
    (void)m;
    /* noch nicht implementiert: entweder ignorieren oder FAULT */
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, ST_FAULT, NULL);
}

void MTR_GotoStart(Motor* m)
{
    (void)m;
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, ST_GOTOSTART, NULL);
}

void MTR_Fault(Motor* m)
{
    (void)m;
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, ST_FAULT, NULL);
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

    if (eventData) {
        const RpmData* d = (const RpmData*)eventData;
        m->speed_ref = d->rpm;
    }
}

static void State_OpenLoop(SM_StateMachine* self, void* eventData)
{
    Motor* m = SM_GetMotor();
    m->state = ST_OPENLOOP;

    if (eventData) {
        const RpmData* d = (const RpmData*)eventData;
        m->speed_ref = d->rpm;
    }
}

static void State_GotoStart(SM_StateMachine* self, void* eventData)
{
    (void)eventData;
    Motor* m = SM_GetMotor();
    m->state = ST_GOTOSTART;
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
    if (oc_trip) {
        oc_trip = false;
        MTR_Fault(m);
    }

    if (btn_stop_edge) {
        btn_stop_edge = false;
        MTR_Stop(m);
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