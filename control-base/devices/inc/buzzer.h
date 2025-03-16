#ifndef BUZZER_H
#define BUZZER_H

#include "bsp_pwm.h"

#define Note_C4 (262) // 261.63Hz, 3822us
#define Note_D4 (294) // 293.66Hz, 3405us
#define Note_E4 (330) // 329.63Hz, 3034us
#define Note_F4 (349) // 349.23Hz, 2863us
#define Note_G4 (392) // 392.00Hz, 2551us
#define Note_A4 (440) // 440.00Hz, 2272us
#define Note_B4 (494) // 493.88Hz, 2052us

#define Note_C5 (523) // 523.25Hz, 1911us
#define Note_D5 (587) // 587.33Hz, 1703us
#define Note_E5 (659) // 659.26Hz, 1517us
#define Note_F5 (698) // 698.46Hz, 1432us
#define Note_G5 (784) // 784.00Hz, 1276us
#define Note_A5 (880) // 880.00Hz, 1136us
#define Note_B5 (988) // 987.77Hz, 1012us

#define Note_C6 (1047) // 1046.50Hz, 956us
#define Note_D6 (1175) // 1174.66Hz, 851us
#define Note_E6 (1319) // 1318.51Hz, 758us
#define Note_F6 (1397) // 1396.91Hz, 716us
#define Note_G6 (1568) // 1567.98Hz, 638us
#define Note_A6 (1760) // 1760.00Hz, 568us
#define Note_B6 (1976) // 1975.53Hz, 506us

#define Note_C7 (2093) // 2093.00Hz, 478us
#define Note_D7 (2349) // 2349.32Hz, 426us
#define Note_E7 (2637) // 2637.02Hz, 379us
#define Note_F7 (2794) // 2793.83Hz, 358us
#define Note_G7 (3136) // 3135.96Hz, 319us
#define Note_A7 (3520) // 3520.00Hz, 284us
#define Note_B7 (3951) // 3951.07Hz, 253us

#define Note_00 (0)

#define EIGHTH_NOTE_DURATION (125)
#define FOURTH_NOTE_DURATION (500)
#define HALF_NOTE_DURATION (1000)
#define WHOLE_NOTE_DURATION (2000)

#define SYSTEM_INITIALIZING {{Note_C4, EIGHTH_NOTE_DURATION }, \
                             {Note_E4, EIGHTH_NOTE_DURATION }, \
                             {Note_G4, EIGHTH_NOTE_DURATION }, \
                             {Note_C5, EIGHTH_NOTE_DURATION }, \
                             {Note_E5, EIGHTH_NOTE_DURATION }, \
                             {Note_G5, EIGHTH_NOTE_DURATION }, \
                             {Note_C6, FOURTH_NOTE_DURATION }}

#define SYSTEM_READY {{Note_G4, EIGHTH_NOTE_DURATION }, \
                      {Note_E5, EIGHTH_NOTE_DURATION }, \
                      {Note_E5, EIGHTH_NOTE_DURATION }, \
                      {Note_F5, EIGHTH_NOTE_DURATION }, \
                      {Note_D5, EIGHTH_NOTE_DURATION }, \
                      {Note_C5, FOURTH_NOTE_DURATION }}

#define SYSTEM_ERROR {{Note_C4, EIGHTH_NOTE_DURATION }, \
                      {Note_G4, EIGHTH_NOTE_DURATION }, \
                      {Note_E4, EIGHTH_NOTE_DURATION }, \
                      {Note_C4, EIGHTH_NOTE_DURATION }, \
                      {Note_G4, EIGHTH_NOTE_DURATION }, \
                      {Note_E4, EIGHTH_NOTE_DURATION }, \
                      {Note_C4, FOURTH_NOTE_DURATION }}

#define MARIO_THEME {\
    {Note_E6, EIGHTH_NOTE_DURATION},\
    {Note_E6, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_E6, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_C6, EIGHTH_NOTE_DURATION},\
    {Note_E6, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_G6, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_G5, EIGHTH_NOTE_DURATION},\
    {Note_00, FOURTH_NOTE_DURATION},\
    {Note_C6, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_G5, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_E5, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_A5, EIGHTH_NOTE_DURATION},\
    {Note_00, FOURTH_NOTE_DURATION},\
    {Note_B5, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_A5, EIGHTH_NOTE_DURATION},\
    {Note_A5, EIGHTH_NOTE_DURATION},\
    {Note_00, FOURTH_NOTE_DURATION},\
    {Note_G5, EIGHTH_NOTE_DURATION},\
    {Note_E6, EIGHTH_NOTE_DURATION},\
    {Note_G6, EIGHTH_NOTE_DURATION},\
    {Note_A6, EIGHTH_NOTE_DURATION},\
    {Note_00, FOURTH_NOTE_DURATION},\
    {Note_F6, EIGHTH_NOTE_DURATION},\
    {Note_G6, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_E6, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_C6, EIGHTH_NOTE_DURATION},\
    {Note_D6, EIGHTH_NOTE_DURATION},\
    {Note_B5, EIGHTH_NOTE_DURATION},\
    {Note_00, FOURTH_NOTE_DURATION}}

#define SYSTEM_INITIALIZING_NOTE_NUM (7)
#define SYSTEM_READY_NOTE_NUM (6)
#define SYSTEM_ERROR_NOTE_NUM (7)
#define MARIO_NOTE_NUM (44)

typedef struct _Melody_t
{
    float notes[50][2];
    float loudness;
    uint16_t note_num;
} Melody_t;

typedef enum
{
    BUZZER_OFF = 0,
    BUZZER_ON,
} Buzzer_State_t;

typedef struct
{
    uint16_t frequency;
    Buzzer_State_t alarm_state;
    float loudness;
} Buzzzer_Instance_t;

void Buzzer_Init(void);
void Buzzer_Set_State(Buzzzer_Instance_t *buzzer, Buzzer_State_t state);
void Buzzer_Set_Loudness(Buzzzer_Instance_t *buzzer, float loudness);
void Buzzer_Set_Note(Buzzzer_Instance_t *buzzer);
void Buzzer_Play_Melody(Melody_t melody);
#endif // BUZZER_H