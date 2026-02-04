# Field-Oriented Control (FOC) with STM32 B-G431B-ESC1 



## Field-Oriented Control (FOC)

![image-20240925200805714](.\README_sources\picture\FOC_structure.png)

## B-G431B-ESC1 

### Top view

<img src=".\README_sources\picture\top_view_B_G431B_ESC1.png" alt="image-20240925195401867" style="zoom:50%;" />

### Bottom view

<img src=".\README_sources\picture\bottom_view_B_G431B_ESC1.png" alt="image-20240925195613769" style="zoom:50%;" />

## Short Overview Of The Code

[foc.h](.\Core\Inc\foc.h): 			Parameters can be changed and basic settings can be made here.

[stm32g4xx_it.c](.\Core\Src\stm32g4xx_it.c):	TIM1_UP_TIM16_IRQHandler Triggers the code with 10kHz.

[task.c](.\Core\Src\task.c):			This is where the sequence and status of the program is controlled. 

[foc.c](.\Core\Src\foc.c):  			Current regulation & transformations are coordinated.

[foc_math.c](.\Core\Src\foc_math.c):		Functions for quick calculations . (LookUpTable,...).

[encoder.c](.\Core\Src\encoder.c):  		Determination of position and speed by reading out the encoder value.

[current_measurement.c](.\Core\Src\current_measurement.c): Calculates the current by reading the ADCs.

[svm.c](.\Core\Src\svm.c): 			Calculates the output for the ESC.











![image-20240927091331188](C:\Users\xboxg\Desktop\image-20240927091331188.png)

20 ms

![image-20240927092623143](C:\Users\xboxg\Desktop\image-20240927092623143.png)

![image-20240927095420926](C:\Users\xboxg\Desktop\image-20240927095420926.png)

![image-20240927100056288](C:\Users\xboxg\Desktop\image-20240927100056288.png)

![image-20240927101931674](C:\Users\xboxg\Desktop\foc_step.png)





Der Strom Regelkreis wird mit der tuning method des technischen Optimums erstellt, bei der die Übertragungsfunktion des geregelten Systems als die eines Tiefpassfilter erster Ordnung eingestellt wird.



The current control loop is created using the technical optimum tuning method, where the transfer function of the controlled system is set as that of a first order low pass filter.

The controller settings for the inner current control loop are set according to magnitude optimum and the controller parameters of the outer speed control loop are set according to the symmetrical optimum.

Der Geschwindigkeitsreglekreis wird über die Bandbreite des 



The controller settings for the inner current loop are set according to the magnitude optimum and the controller parameters for the outer speed loop are set according to the symmetry optimum.[]

[] Control of Electric Machine Drive Systems | Wiley Online Books. https://onlinelibrary.wiley.com/doi/book/10.1002/ 9780470876541, . – (Accessed on 08/06/2022)





![image-20241004102954214](C:\Users\xboxg\Desktop\image-20241004102954214.png)

## Automat

![Ablaufdiagramm](Ablaufdiagramm.drawio.svg)






### Zustandsautomat für die Motorsteuerung

Der Zustandsautomat der Motorsteuerung ist als tabellengetriebene Zustandsmaschine mit einem kleinen Kernel aufgebaut. Die Struktur `SM_StateMachine` kapselt den aktuellen Zustand und die zugehörige Motorinstanz:

```c
typedef struct SM_StateMachine {
    Motor*  m;
    uint8_t currentState;
    uint8_t newState;
    bool    eventGenerated;
    void*   pEventData;
} SM_StateMachine;
```

`currentState` enthält den aktiven Zustand, `newState` wird bei einem Übergang gesetzt, und `eventGenerated` signalisiert dem Kernel, dass ein internes Ereignis zur Abarbeitung ansteht. Über den Zeiger `m` greifen die Zustandsfunktionen direkt auf die zugehörige `Motor`‑Instanz zu.

Beim Initialisieren der Zustandsmaschine wird der Motor in einen sicheren Startzustand überführt:

```c
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
```

### Zustände und State‑Map

Die möglichen Zustände des Automaten sind im `MotorState`‑Enum definiert:

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

Zu jedem Zustand existiert ein eigener State‑Handler:

```c
static void State_Stop(SM_StateMachine* self, void* eventData);
static void State_ClosedLoop(SM_StateMachine* self, void* eventData);
static void State_OpenLoop(SM_StateMachine* self, void* eventData);
static void State_GotoStart(SM_StateMachine* self, void* eventData);
static void State_ParameterId(SM_StateMachine* self, void* eventData);
static void State_Fault(SM_StateMachine* self, void* eventData);
```

Diese Funktionen werden in einer State‑Map zusammengefasst:

```c
static const SM_StateStruct MotorStateMap[ST_MAX] = {
    [ST_STOP]        = { State_Stop },
    [ST_CLOSEDLOOP]  = { State_ClosedLoop },
    [ST_OPENLOOP]    = { State_OpenLoop },
    [ST_GOTOSTART]   = { State_GotoStart },
    [ST_PARAMETER_ID]= { State_ParameterId },
    [ST_FAULT]       = { State_Fault }
};
```

Der Kernel ruft die jeweilige Zustandsfunktion ausschließlich über diese Tabelle auf. Die Implementierungen selbst sind bewusst schlank gehalten und setzen nur die relevanten Felder der Motorstruktur. Beispiel für den Stoppzustand:

```c
static void State_Stop(SM_StateMachine* self, void* eventData)
{
    (void)eventData;
    Motor* m = SM_GetMotor();

    m->state     = ST_STOP;
    m->speed_ref = 0;
}
```

Weitere Zustände setzen analog lediglich das logische Zustandsfeld:

```c
static void State_ClosedLoop(SM_StateMachine* self, void* eventData)
{
    Motor* m = SM_GetMotor();
    m->state = ST_CLOSEDLOOP;
}

static void State_Fault(SM_StateMachine* self, void* eventData)
{
    (void)eventData;
    Motor* m = SM_GetMotor();
    m->state     = ST_FAULT;
    m->speed_ref = 0;
}
```

### Mini‑Kernel: interne und externe Ereignisse

Der Kern der Zustandsmaschine besteht aus einer internen Ereignisfunktion und einer Ausführungsfunktion. Interne Ereignisse planen einen Zustandswechsel:

```c
static void SM_InternalEvent(SM_StateMachine* self, uint8_t newState, void* eventData)
{
    self->newState       = newState;
    self->pEventData     = eventData;
    self->eventGenerated = true;
}
```

Die eigentliche Abarbeitung erfolgt in der State‑Engine:

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

Solange `eventGenerated` gesetzt ist, übernimmt die Engine den neuen Zustand, prüft die Gültigkeit und ruft schließlich die zugehörige Zustandsfunktion aus der State‑Map auf. Dadurch können auch Kaskaden von internen Ereignissen abgearbeitet werden.

Die Funktion `SM_ExternalEvent` stellt die Schnittstelle nach außen dar und filtert Sonderfälle:

```c
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
```

Unerlaubte oder explizit ignorierte Übergänge werden hier verworfen. Nur gültige Zustandswechsel werden als internes Ereignis an die Engine weitergegeben.

### Ereignisfunktionen und Übergangstabellen

Die öffentlichen API‑Funktionen (`MTR_Stop`, `MTR_RunClosedLoop` usw.) repräsentieren die externen Ereignisse des Automaten. Für jedes Ereignis existiert eine Übergangstabelle, die vom aktuellen Zustand auf den Folgezustand abbildet. Beispiel für das Stopp‑Ereignis:

```c
void MTR_Stop(Motor* m)
{
    (void)m;
    static const uint8_t TRANSITIONS[ST_MAX] = {
        [ST_STOP]        = EVENT_IGNORED,
        [ST_CLOSEDLOOP]  = ST_STOP,
        [ST_OPENLOOP]    = ST_STOP,
        [ST_GOTOSTART]   = ST_STOP,
        [ST_PARAMETER_ID]= ST_STOP,
        [ST_FAULT]       = CANNOT_HAPPEN
    };

    uint8_t next = TRANSITIONS[sm.currentState];
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, next, NULL);
}
```

Für das Ereignis „RunClosedLoop“ sieht die Tabelle entsprechend anders aus:

```c
void MTR_RunClosedLoop(Motor* m)
{
    (void)m;
    static const uint8_t TRANSITIONS[ST_MAX] = {
        [ST_STOP]        = ST_CLOSEDLOOP,
        [ST_CLOSEDLOOP]  = EVENT_IGNORED,
        [ST_OPENLOOP]    = ST_CLOSEDLOOP,
        [ST_GOTOSTART]   = EVENT_IGNORED,
        [ST_PARAMETER_ID]= EVENT_IGNORED,
        [ST_FAULT]       = CANNOT_HAPPEN
    };

    uint8_t next = TRANSITIONS[sm.currentState];
    SM_ExternalEvent(&sm, MotorStateMap, ST_MAX, next, NULL);
}
```

Damit ist das komplette Übergangsverhalten des Automaten in kompakten Tabellen beschrieben, anstatt in verschachtelten `if`‑ oder `switch`‑Strukturen. Unerlaubte Übergänge werden über `CANNOT_HAPPEN` markiert, ignorierte Ereignisse über `EVENT_IGNORED`.

### Anbindung an die Motor‑Logik

Die zyklische Service‑Funktion `MotorSM_Service` koppelt physikalische Ereignisse (Flags, Taster, Schutzfunktionen) an die abstrakten Zustandsereignisse des Automaten:

```c
void MotorSM_Service(Motor* m)
{
    if(m->stop_request_flag){
        m->stop_request_flag  = false;
        m->start_request_flag = false;
        MTR_Stop(m);
        return;
    }
    if(m->start_request_flag){
        m->start_request_flag = false;
        MTR_RunClosedLoop(m);
    }
    if(m->gotostart_finish_flag){
        /* nicht hier zurücksetzen */
        MTR_Stop(m);
    }
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
```

Interne Flags (`start_request_flag`, `stop_request_flag`, `gotostart_finish_flag`), Schutzereignisse (`oc_trip`) und Tasterflanken (`btn_*_edge`) werden hier auf die entsprechenden API‑Funktionen (`MTR_*`) abgebildet. Der Zustandsautomat selbst arbeitet ausschließlich mit diesen abstrakten Ereignissen und bleibt dadurch unabhängig von den konkreten Eingangssignalen.



## Positionsbestimmung mittels magnetischem Encoder

Zur Erfassung der Rotorposition wird ein magnetischer Encoder vom Typ MT6701 verwendet, der im ABZ-Modus betrieben wird. Die Positionsinformation wird über zwei phasenversetzte Inkrementalsignale (A und B) bereitgestellt. Aus der Reihenfolge der Flanken (A vor B oder B vor A) lässt sich die Drehrichtung bestimmen.

Die A- und B-Signale werden mithilfe eines Hardware-Timers im Encoder-Modus ausgewertet. Abhängig von der Drehrichtung wird der Zählerstand des Timers inkrementiert oder dekrementiert. Wird die Periodendauer (Auto-Reload-Wert) des Timers auf die Auflösung des Encoders eingestellt, entspricht der aktuelle Zählerwert – bei bekanntem Startwinkel – direkt dem mechanischen Rotorwinkel.

<img src="angle_representation.drawio.svg" alt="angle_representation.drawio" style="zoom:150%;" />

#### Winkelrepräsentation in 16-Bit-Festkommadarstellung

Die Rotorposition wird als 16-Bit-Zahl dargestellt. Dabei kann sowohl ein vorzeichenloser (`uint16_t`) als auch ein vorzeichenbehafteter (`int16_t`) Datentyp verwendet werden, da beide binärtechnisch identisch sind. Der Wertebereich wird zyklisch auf den Winkelbereich von 0° bis 360° abgebildet.

Ein wesentlicher Vorteil dieser Darstellung ist, dass der natürliche Überlauf des 16-Bit-Zählers einer kontinuierlichen Winkeldarstellung entspricht. Dadurch lassen sich Winkeloperationen wie Differenzbildung oder Skalierung effizient und ohne zusätzliche Modulo-Operationen implementieren, was insbesondere für zeitkritische Regelalgorithmen vorteilhaft ist.

Bei hohen Drehzahlen in Kombination mit hohen Abtastraten kann es jedoch zu scheinbaren Ausreißern bei der Differenzierung kommen (beim Übergang von `UINT16_MAX` zu `0` beziehungsweise von `+INT16_MAX` zu `-INT16_MAX` ). 

#### Trick: Zählerraum vergrößern (16× Overscaling)

Um auch bei hohen Drehzahlen stabile Geschwindigkeitswerte zu erhalten, wird der Zählerbereich des Timers bewusst vergrößert. Der verwendete Encoder liefert in der Praxis eine periodische Positionsgröße mit einer effektiven Maximalzählzahl von etwa \(N_\text{enc}\approx 4000\) (je nach Betriebsmodus/Initialisierung kann dieser Standardwert auftreten).

Statt den Auto-Reload-Wert (ARR) von TIM4 direkt auf \(N_\text{enc}-1\) zu setzen, wird der Timer auf das 16-Fache skaliert:

\[
ARR = 16\cdot N_\text{enc} - 1 \approx 16\cdot 4000 - 1 = 63999
\]

Damit wird die Winkelrepräsentation „aufgespreizt“ (höhere interne Auflösung), und die Differenzbildung über den periodischen Überlauf wird deutlich unempfindlicher. In der Praxis führt das zu wesentlich saubereren Drehzahlschätzungen bei hohen Drehzahlen.

<img src="Timer_to_rotations.drawio.svg" alt="Timer_to_rotations.drawio" style="zoom:150%;" />



Die Funktion `calc_rotor_position()` liest den aktuellen Zählerstand des Encoders aus dem Timer aus und berechnet daraus die mechanische Rotorposition innerhalb einer Umdrehung. Zunächst wird der absolute Zählerstand in volle Umdrehungen und einen Restwert innerhalb einer Periode zerlegt, sodass ein normierter Encoderwert für eine einzelne mechanische Umdrehung vorliegt. Anschließend wird die Position auf eine Q16-Festkommadarstellung skaliert, wodurch eine hochauflösende, zyklische Winkelrepräsentation entsteht, die sich besonders gut für die spätere Geschwindigkeitsberechnung eignet. Abschließend wird aus der mechanischen Position der elektrische Winkel berechnet.

```c
void calc_rotor_position(void){
	encoder_count = encoder_count_large = __HAL_TIM_GET_COUNTER(&htim4);
	uint16_t rotation = encoder_count/ENCODER_PULS_PER_REVOLUTION;
	encoder_count = encoder_count - (rotation * ENCODER_PULS_PER_REVOLUTION);
	// now we have the real encoder count within one revolution

	// ############## TRANSFORM POSITION TO Q16 SCALE  #################
	uint32_t x 		= encoder_count_large * (uint32_t)Q16; // Q16 = 2^16
	x 				/= (16 * ENCODER_PULS_PER_REVOLUTION); // ENCODER_PULS_PER_REVOLUTION = 4000
	rotor_position 	= (int32_t)x;	// we need the global variable rotor_position for speed calculation

	// ############## TRANSFORM TO ELECTRICAL ANGLE #################
	encoder_count_uint = (encoder_count * count_to_angle); 
	encoder_count = (uint16_t)encoder_count_uint;
}
```



