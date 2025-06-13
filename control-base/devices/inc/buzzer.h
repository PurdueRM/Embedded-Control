#ifndef BUZZER_H
#define BUZZER_H

#include "bsp_pwm.h"

#define Note_C3  (131)  // 130.81 Hz
#define Note_D3  (147)  // 146.83 Hz
#define Note_E3  (165)  // 164.81 Hz
#define Note_F3  (175)  // 174.61 Hz
#define Note_G3  (196)  // 196.00 Hz
#define Note_A3  (220)  // 220.00 Hz
#define Note_B3  (247)  // 246.94 Hz

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

// C3 to B3 with sharps and flats
#define Note_CSharp3  (139)  // C#3 / Db3, ~138.59Hz
#define Note_DFlat3   Note_CSharp3

#define Note_DSharp3  (156)  // D#3 / Eb3, ~155.56Hz
#define Note_EFlat3   Note_DSharp3

#define Note_FSharp3  (185)  // F#3 / Gb3, ~185.00Hz
#define Note_GFlat3   Note_FSharp3

#define Note_GSharp3  (208)  // G#3 / Ab3, ~207.65Hz
#define Note_AFlat3   Note_GSharp3

#define Note_ASharp3  (233)  // A#3 / Bb3, ~233.08Hz
#define Note_BFlat3   Note_ASharp3

// C4 to B4 with sharps and flats
#define Note_CSharp4 (277) // C#4 / Db4, ~277.18Hz
#define Note_DFlat4  Note_CSharp4

#define Note_DSharp4 (311) // D#4 / Eb4, ~311.13Hz
#define Note_EFlat4  Note_DSharp4

#define Note_FSharp4 (370) // F#4 / Gb4, ~369.99Hz
#define Note_GFlat4  Note_FSharp4

#define Note_GSharp4 (415) // G#4 / Ab4, ~415.30Hz
#define Note_AFlat4  Note_GSharp4

#define Note_ASharp4 (466) // A#4 / Bb4, ~466.16Hz
#define Note_BFlat4  Note_ASharp4

// C5 to B5
#define Note_CSharp5 (554) // C#5 / Db5, ~554.37Hz
#define Note_DFlat5  Note_CSharp5

#define Note_DSharp5 (622) // D#5 / Eb5, ~622.25Hz
#define Note_EFlat5  Note_DSharp5

#define Note_FSharp5 (740) // F#5 / Gb5, ~739.99Hz
#define Note_GFlat5  Note_FSharp5

#define Note_GSharp5 (831) // G#5 / Ab5, ~830.61Hz
#define Note_AFlat5  Note_GSharp5

#define Note_ASharp5 (932) // A#5 / Bb5, ~932.33Hz
#define Note_BFlat5  Note_ASharp5

// C6 to B6
#define Note_CSharp6 (1109) // C#6 / Db6, ~1108.73Hz
#define Note_DFlat6  Note_CSharp6

#define Note_DSharp6 (1245) // D#6 / Eb6, ~1244.51Hz
#define Note_EFlat6  Note_DSharp6

#define Note_FSharp6 (1480) // F#6 / Gb6, ~1479.98Hz
#define Note_GFlat6  Note_FSharp6

#define Note_GSharp6 (1661) // G#6 / Ab6, ~1661.22Hz
#define Note_AFlat6  Note_GSharp6

#define Note_ASharp6 (1865) // A#6 / Bb6, ~1864.66Hz
#define Note_BFlat6  Note_ASharp6

// C7 to B7
#define Note_CSharp7 (2217) // C#7 / Db7, ~2217.46Hz
#define Note_DFlat7  Note_CSharp7

#define Note_DSharp7 (2489) // D#7 / Eb7, ~2489.02Hz
#define Note_EFlat7  Note_DSharp7

#define Note_FSharp7 (2960) // F#7 / Gb7, ~2959.96Hz
#define Note_GFlat7  Note_FSharp7

#define Note_GSharp7 (3322) // G#7 / Ab7, ~3322.44Hz
#define Note_AFlat7  Note_GSharp7

#define Note_ASharp7 (3729) // A#7 / Bb7, ~3729.31Hz
#define Note_BFlat7  Note_ASharp7

#define SMALL_SPACER (50)
#define SIXTEENTH_NOTE_DURATION (62)
#define EIGHTH_NOTE_DURATION (125)
#define FOURTH_NOTE_DURATION (500)
#define HALF_NOTE_DURATION (1000)
#define DOT_HALF_NOTE_DURATION (1500)
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

#define PVZ_THEME {\
    {Note_E5, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_G5, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_F5, EIGHTH_NOTE_DURATION},\
    {Note_E5, EIGHTH_NOTE_DURATION},\
    {Note_C5, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_E5, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_G5, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_A5, EIGHTH_NOTE_DURATION},\
    {Note_G5, EIGHTH_NOTE_DURATION},\
    {Note_E5, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_C5, FOURTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_C5, EIGHTH_NOTE_DURATION},\
    {Note_E5, EIGHTH_NOTE_DURATION},\
    {Note_G5, EIGHTH_NOTE_DURATION},\
    {Note_F5, EIGHTH_NOTE_DURATION},\
    {Note_E5, EIGHTH_NOTE_DURATION},\
    {Note_D5, EIGHTH_NOTE_DURATION},\
    {Note_E5, HALF_NOTE_DURATION},\
}

#define NOTE_WALK {\
    {Note_C4, FOURTH_NOTE_DURATION},\
    {Note_CSharp4, FOURTH_NOTE_DURATION},\
    {Note_D4, FOURTH_NOTE_DURATION},\
    {Note_DSharp4, FOURTH_NOTE_DURATION},\
    {Note_E4, FOURTH_NOTE_DURATION},\
    {Note_F4, FOURTH_NOTE_DURATION},\
    {Note_FSharp4, FOURTH_NOTE_DURATION},\
    {Note_G4, FOURTH_NOTE_DURATION},\
    {Note_GSharp4, FOURTH_NOTE_DURATION},\
    {Note_A4, FOURTH_NOTE_DURATION},\
    {Note_ASharp4, FOURTH_NOTE_DURATION},\
    {Note_B4, FOURTH_NOTE_DURATION},\
    \
    {Note_C5, FOURTH_NOTE_DURATION},\
    {Note_CSharp5, FOURTH_NOTE_DURATION},\
    {Note_D5, FOURTH_NOTE_DURATION},\
    {Note_DSharp5, FOURTH_NOTE_DURATION},\
    {Note_E5, FOURTH_NOTE_DURATION},\
    {Note_F5, FOURTH_NOTE_DURATION},\
    {Note_FSharp5, FOURTH_NOTE_DURATION},\
    {Note_G5, FOURTH_NOTE_DURATION},\
    {Note_GSharp5, FOURTH_NOTE_DURATION},\
    {Note_A5, FOURTH_NOTE_DURATION},\
    {Note_ASharp5, FOURTH_NOTE_DURATION},\
    {Note_B5, FOURTH_NOTE_DURATION},\
    \
    {Note_C6, FOURTH_NOTE_DURATION},\
    {Note_CSharp6, FOURTH_NOTE_DURATION},\
    {Note_D6, FOURTH_NOTE_DURATION},\
    {Note_DSharp6, FOURTH_NOTE_DURATION},\
    {Note_E6, FOURTH_NOTE_DURATION},\
    {Note_F6, FOURTH_NOTE_DURATION},\
    {Note_FSharp6, FOURTH_NOTE_DURATION},\
    {Note_G6, FOURTH_NOTE_DURATION},\
    {Note_GSharp6, FOURTH_NOTE_DURATION},\
    {Note_A6, FOURTH_NOTE_DURATION},\
    {Note_ASharp6, FOURTH_NOTE_DURATION},\
    {Note_B6, FOURTH_NOTE_DURATION},\
    \
    {Note_C7, FOURTH_NOTE_DURATION},\
    {Note_CSharp7, FOURTH_NOTE_DURATION},\
    {Note_D7, FOURTH_NOTE_DURATION},\
    {Note_DSharp7, FOURTH_NOTE_DURATION},\
    {Note_E7, FOURTH_NOTE_DURATION},\
    {Note_F7, FOURTH_NOTE_DURATION},\
    {Note_FSharp7, FOURTH_NOTE_DURATION},\
    {Note_G7, FOURTH_NOTE_DURATION},\
    {Note_GSharp7, FOURTH_NOTE_DURATION},\
    {Note_A7, FOURTH_NOTE_DURATION},\
    {Note_ASharp7, FOURTH_NOTE_DURATION},\
    {Note_B7, FOURTH_NOTE_DURATION}\
}

#define NOTE_WALK_C3_CHROMATIC { \
    {Note_C3, EIGHTH_NOTE_DURATION}, \
    {Note_CSharp3, EIGHTH_NOTE_DURATION}, \
    {Note_D3, EIGHTH_NOTE_DURATION}, \
    {Note_DSharp3, EIGHTH_NOTE_DURATION}, \
    {Note_E3, EIGHTH_NOTE_DURATION}, \
    {Note_F3, EIGHTH_NOTE_DURATION}, \
    {Note_FSharp3, EIGHTH_NOTE_DURATION}, \
    {Note_G3, EIGHTH_NOTE_DURATION}, \
    {Note_GSharp3, EIGHTH_NOTE_DURATION}, \
    {Note_A3, EIGHTH_NOTE_DURATION}, \
    {Note_ASharp3, EIGHTH_NOTE_DURATION}, \
    {Note_B3, EIGHTH_NOTE_DURATION}, \
    {Note_ASharp3, EIGHTH_NOTE_DURATION}, \
    {Note_A3, EIGHTH_NOTE_DURATION}, \
    {Note_GSharp3, EIGHTH_NOTE_DURATION}, \
    {Note_G3, EIGHTH_NOTE_DURATION}, \
    {Note_FSharp3, EIGHTH_NOTE_DURATION}, \
    {Note_F3, EIGHTH_NOTE_DURATION}, \
    {Note_E3, EIGHTH_NOTE_DURATION}, \
    {Note_DSharp3, EIGHTH_NOTE_DURATION}, \
    {Note_D3, EIGHTH_NOTE_DURATION}, \
    {Note_CSharp3, EIGHTH_NOTE_DURATION}, \
    {Note_C3, EIGHTH_NOTE_DURATION} \
}

// 别看我只是一只羊：https://musescore.com/user/22434151/scores/5151487
#define PLEASANT_GOAT_AND_WOLF_THEME {\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_ASharp5, EIGHTH_NOTE_DURATION},\
    {Note_ASharp5, EIGHTH_NOTE_DURATION},\
    {Note_ASharp5, EIGHTH_NOTE_DURATION},\
    {Note_ASharp5, EIGHTH_NOTE_DURATION},\
    {Note_GSharp5, EIGHTH_NOTE_DURATION},\
    {Note_FSharp5, EIGHTH_NOTE_DURATION},\
    {Note_F5, EIGHTH_NOTE_DURATION},\
    {Note_B4, FOURTH_NOTE_DURATION},\
    {Note_GSharp5, DOT_HALF_NOTE_DURATION},\
    \
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_F5, EIGHTH_NOTE_DURATION},\
    {Note_F5, EIGHTH_NOTE_DURATION},\
    {Note_F5, EIGHTH_NOTE_DURATION},\
    {Note_F5, EIGHTH_NOTE_DURATION},\
    {Note_FSharp5, EIGHTH_NOTE_DURATION},\
    {Note_GSharp5, FOURTH_NOTE_DURATION},\
    {Note_FSharp5, WHOLE_NOTE_DURATION},\
    \
    {Note_ASharp4, FOURTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    {Note_GSharp4, FOURTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    \
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_ASharp4, EIGHTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    {Note_GSharp4, FOURTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    \
    {Note_ASharp4, FOURTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    {Note_00, EIGHTH_NOTE_DURATION},\
    {Note_GSharp4, FOURTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    \
    {Note_ASharp4, FOURTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    {Note_GSharp4, FOURTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    {Note_CSharp5, EIGHTH_NOTE_DURATION},\
    \
    {Note_FSharp5, SIXTEENTH_NOTE_DURATION},\
    {Note_F5, SIXTEENTH_NOTE_DURATION}, \
    {Note_E5, SIXTEENTH_NOTE_DURATION}, \
    {Note_DSharp5, SIXTEENTH_NOTE_DURATION},\
    {Note_D5, SIXTEENTH_NOTE_DURATION},\
    {Note_CSharp5, SIXTEENTH_NOTE_DURATION},\
    {Note_C5, SIXTEENTH_NOTE_DURATION},\
    {Note_B4, SIXTEENTH_NOTE_DURATION},\
    {Note_ASharp4, SIXTEENTH_NOTE_DURATION},\
    {Note_A4, SIXTEENTH_NOTE_DURATION},\
    {Note_GSharp4, SIXTEENTH_NOTE_DURATION},\
    {Note_G4, SIXTEENTH_NOTE_DURATION},\
    {Note_FSharp4, FOURTH_NOTE_DURATION},\
}

// https://www.bilibili.com/video/BV1LM4y1R7Gp/?vd_source=7e89cec769f049f9952459436503cfd4
#define RM_MAIN_THEME { \
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_C4, EIGHTH_NOTE_DURATION},\
    {Note_C4, EIGHTH_NOTE_DURATION},\
    {Note_GSharp3, EIGHTH_NOTE_DURATION},\
    {Note_GSharp3, EIGHTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    \
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_C4, EIGHTH_NOTE_DURATION},\
    {Note_C4, EIGHTH_NOTE_DURATION},\
    {Note_GSharp3, EIGHTH_NOTE_DURATION},\
    {Note_GSharp3, EIGHTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    \
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_D4, EIGHTH_NOTE_DURATION},\
    {Note_D4, EIGHTH_NOTE_DURATION},\
    {Note_GSharp3, EIGHTH_NOTE_DURATION},\
    {Note_GSharp3, EIGHTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    \
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_D4, EIGHTH_NOTE_DURATION},\
    {Note_D4, EIGHTH_NOTE_DURATION},\
    {Note_GSharp3, EIGHTH_NOTE_DURATION},\
    {Note_GSharp3, EIGHTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    \
    {Note_GSharp3, EIGHTH_NOTE_DURATION},\
    {Note_GSharp3, SIXTEENTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_A3, EIGHTH_NOTE_DURATION},\
    {Note_B3, EIGHTH_NOTE_DURATION},\
    {Note_B3, EIGHTH_NOTE_DURATION},\
    {Note_C4, EIGHTH_NOTE_DURATION},\
    {Note_C4, EIGHTH_NOTE_DURATION},\
    \
    {Note_D4, EIGHTH_NOTE_DURATION},\
    {Note_D4, EIGHTH_NOTE_DURATION},\
    {Note_E4, EIGHTH_NOTE_DURATION},\
    {Note_E4, EIGHTH_NOTE_DURATION},\
    {Note_B3, EIGHTH_NOTE_DURATION},\
    {Note_B3, EIGHTH_NOTE_DURATION},\
    {Note_C4, EIGHTH_NOTE_DURATION},\
    {Note_C4, EIGHTH_NOTE_DURATION},\
}

#define SYSTEM_INITIALIZING_NOTE_NUM (7)
#define SYSTEM_READY_NOTE_NUM (6)
#define SYSTEM_ERROR_NOTE_NUM (7)
#define MARIO_NOTE_NUM (44)
#define PVZ_NOTE_NUM (25)
#define NOTE_WALK_NOTE_NUM (48)
#define PLEASANT_GOAT_AND_WOLF_THEME_NOTE_NUM (58)
#define NOTE_WALK_C3_CHROMATIC_NOTE_NUM (23)
#define RM_MAIN_THEME_NOTE_NUM (48)

typedef struct _Melody_t
{
    float notes[60][2];
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