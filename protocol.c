/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2018 Simon Hailes <btsimonh@googlemail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"
#ifdef CONTROL_SENSOR
    #include "sensorcoms.h"
#endif
#include "protocol.h"
#ifdef HALL_INTERRUPTS
    #include "hallinterrupts.h"
#endif
#include "bldc.h"
#include "comms.h"
#ifndef EXCLUDE_DEADRECKONER
    #include "deadreckoner.h"
#endif

#include <string.h>
#include <stdlib.h>


//////////////////////////////////////////////
// things needed by main.c
int control_type = 0;


#if (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL2)


// forward declaration
extern int paramcount;
extern PARAMSTAT params[];
//////////////////////////////////////////////
// variables you want to read/write here. Only common used ones, specific ones below.

extern volatile uint32_t timeout; // global variable for timeout

// gather two separate speed variables togther,
typedef struct tag_SPEEDS{
    int speedl;
    int speedr;
} SPEEDS;
static SPEEDS speedsx = {0,0};




////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
// Default function, wipes receive memory before writing (and readresponse is just a differenct type of writing)


void fn_preWriteClear ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
    switch (fn_type) {
        case FN_TYPE_PRE_WRITE:
        case FN_TYPE_PRE_READRESPONSE:
            // ensure clear in case of short write
            memset(param->ptr, 0, param->len);
            break;
    }
}


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x00 version

static int version = 1;


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x09 enable

extern uint8_t enable; // global variable for motor enable

//////////////////////////////////////////////
// make values safe before we change enable...

void fn_enable ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
    switch (fn_type) {
        case FN_TYPE_PRE_WRITE:
            if (!enable) {
                #ifdef HALL_INTERRUPTS
                    // assume we will enable,
                    // set wanted posn to current posn, else we may rush into a wall
                    PosnData.wanted_posn_mm[0] = HallData[0].HallPosn_mm;
                    PosnData.wanted_posn_mm[1] = HallData[1].HallPosn_mm;
                #endif

                // clear speeds to zero
                SpeedData.wanted_speed_mm_per_sec[0] = 0;
                SpeedData.wanted_speed_mm_per_sec[1] = 0;
                speedsx.speedl = 0;
                speedsx.speedr = 0;
                PWMData.pwm[0] = 0;
                PWMData.pwm[1] = 0;
                #ifdef FLASH_STORAGE
                        init_PID_control();
                #endif
            }
            break;
    }
}


#ifdef CONTROL_SENSOR
////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x01 sensor_data

extern SENSOR_DATA sensor_data[2];

// used to send only pertinent data, not the whole structure
SENSOR_FRAME sensor_copy[2];

void fn_SensorData ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
    switch (fn_type) {
        case FN_TYPE_PRE_READ:
            // copy just sensor input data
            sensor_copy[0] = sensor_data[0].complete;
            sensor_copy[1] = sensor_data[1].complete;
            break;
    }
}

#endif

#ifdef HALL_INTERRUPTS
////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x02 HallData

/* see hallinterrupts.h */
#endif

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x03 SpeedData

SPEED_DATA SpeedData = {
    {0, 0},

    600, // max power (PWM)
    -600,  // min power
    40 // minimum mm/s which we can ask for
};

void fn_SpeedData ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
    switch (fn_type) {
        case FN_TYPE_PRE_READ:
            speedsx.speedl = ((SPEED_DATA*) (param->ptr))->wanted_speed_mm_per_sec[0];
            speedsx.speedr = ((SPEED_DATA*) (param->ptr))->wanted_speed_mm_per_sec[1];
            break;

        case FN_TYPE_PRE_WRITE:
            control_type = CONTROL_TYPE_SPEED;
            timeout = 0;
            break;
    }
}

#ifdef HALL_INTERRUPTS
////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x04 Position

POSN Position;

void fn_Position ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
    switch (fn_type) {
        case FN_TYPE_PRE_READ:
            ((POSN*) (param->ptr))->LeftAbsolute = HallData[0].HallPosn_mm;
            ((POSN*) (param->ptr))->LeftOffset = HallData[0].HallPosn_mm - HallData[0].HallPosn_mm_lastread;
            ((POSN*) (param->ptr))->RightAbsolute = HallData[1].HallPosn_mm;
            ((POSN*) (param->ptr))->RightOffset = HallData[1].HallPosn_mm - HallData[1].HallPosn_mm_lastread;
            break;

        case FN_TYPE_POST_WRITE:
            HallData[0].HallPosn_mm_lastread = ((POSN*) (param->ptr))->LeftAbsolute;
            HallData[1].HallPosn_mm_lastread = ((POSN*) (param->ptr))->RightAbsolute;
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x05 PositionIncr

POSN_INCR PositionIncr;

void fn_PositionIncr ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
    switch (fn_type) {
        case FN_TYPE_POST_WRITE:
            // if switching to control type POSITION,
            if ((control_type != CONTROL_TYPE_POSITION) || !enable) {
                control_type = CONTROL_TYPE_POSITION;
                // then make sure we won't rush off somwehere strange
                // by setting our wanted posn to where we currently are...
                fn_enable( s, param, FN_TYPE_PRE_WRITE, content, 0); // TODO: I don't like calling this with a param entry which does not fit to the handler..
            }

            enable = 1;
            timeout = 0;

            // increment our wanted position
            PosnData.wanted_posn_mm[0] += ((POSN_INCR*) (param->ptr))->Left;
            PosnData.wanted_posn_mm[1] += ((POSN_INCR*) (param->ptr))->Right;
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x06 PosnData

POSN_DATA PosnData = {
    {0, 0},

    200, // max pwm in posn mode
    70, // min pwm in posn mode
};


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x07 RawPosition

POSN RawPosition;


void fn_RawPosition ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
    switch (fn_type) {
        case FN_TYPE_PRE_READ:
            ((POSN*) (param->ptr))->LeftAbsolute = HallData[0].HallPosn;
            ((POSN*) (param->ptr))->LeftOffset = HallData[0].HallPosn - HallData[0].HallPosn_lastread;
            ((POSN*) (param->ptr))->RightAbsolute = HallData[1].HallPosn;
            ((POSN*) (param->ptr))->RightOffset = HallData[1].HallPosn - HallData[1].HallPosn_lastread;
            break;

        case FN_TYPE_POST_WRITE:
            HallData[0].HallPosn_lastread = ((POSN*) (param->ptr))->LeftAbsolute;
            HallData[1].HallPosn_lastread = ((POSN*) (param->ptr))->RightAbsolute;
            break;
    }
}

#endif

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x0A disablepoweroff

extern uint8_t disablepoweroff;


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x0B debug_out

extern uint8_t debug_out;

#ifndef EXCLUDE_DEADRECKONER
////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x0C xytPosn

// ded reckoning posn
extern INTEGER_XYT_POSN xytPosn;
#endif

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x0D PWMData and 0x0E PWMData.pwm

PWM_DATA PWMData = {
    .pwm[0] = 0,
    .pwm[1] = 0,
    .speed_max_power =  600,
    .speed_min_power = -600,
    .speed_minimum_pwm = 40 // guard value, below this set to zero
};

extern int pwms[2];

void fn_PWMData ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
    switch (fn_type) {
        case FN_TYPE_PRE_READRESPONSE:
        case FN_TYPE_PRE_WRITE:
            control_type = CONTROL_TYPE_PWM;
            timeout = 0;
            break;

        case FN_TYPE_POST_READRESPONSE:
        case FN_TYPE_POST_WRITE:
            for (int i = 0; i < 2; i++) {
                if (((PWM_DATA*) (param->ptr))->pwm[i] > ((PWM_DATA*) (param->ptr))->speed_max_power) {
                    ((PWM_DATA*) (param->ptr))->pwm[i] = ((PWM_DATA*) (param->ptr))->speed_max_power;
                }
                if (((PWM_DATA*) (param->ptr))->pwm[i] < ((PWM_DATA*) (param->ptr))->speed_min_power) {
                    ((PWM_DATA*) (param->ptr))->pwm[i] = ((PWM_DATA*) (param->ptr))->speed_min_power;
                }
                if ((((PWM_DATA*) (param->ptr))->pwm[i] > 0) && (((PWM_DATA*) (param->ptr))->pwm[i] < ((PWM_DATA*) (param->ptr))->speed_minimum_pwm)) {
                    ((PWM_DATA*) (param->ptr))->pwm[i] = 0;
                }
                if ((((PWM_DATA*) (param->ptr))->pwm[i] < 0) && (((PWM_DATA*) (param->ptr))->pwm[i] > -((PWM_DATA*) (param->ptr))->speed_minimum_pwm)) {
                    ((PWM_DATA*) (param->ptr))->pwm[i] = 0;
                }
            }
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x21 BuzzerData

BUZZER_DATA BuzzerData = {
    .buzzerFreq = 0,
    .buzzerPattern = 0,
    .buzzerLen = 0,
};

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
extern uint16_t buzzerLen;

void fn_BuzzerData ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
    switch (fn_type) {
        case FN_TYPE_POST_WRITE:
            buzzerFreq      = ((BUZZER_DATA*) (param->ptr))->buzzerFreq;
            buzzerLen       = ((BUZZER_DATA*) (param->ptr))->buzzerLen;
            buzzerPattern   = ((BUZZER_DATA*) (param->ptr))->buzzerPattern;
            break;

        case FN_TYPE_PRE_READ:
            ((BUZZER_DATA*) (param->ptr))->buzzerFreq       = buzzerFreq;
            ((BUZZER_DATA*) (param->ptr))->buzzerLen        = buzzerLen;
            ((BUZZER_DATA*) (param->ptr))->buzzerPattern    = buzzerPattern;
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x22 SubscribeData

SUBSCRIBEDATA SubscribeData =  { .code=0, .period=0, .count=0, .som=0 };

void fn_SubscribeData ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {

    fn_preWriteClear(s, param, fn_type, content, len); // Wipes memory before write (and readresponse is just a differenct type of writing)

    switch (fn_type) {

        case FN_TYPE_POST_WRITE:
        case FN_TYPE_POST_READRESPONSE:
            ;     // empty statement, labels must not be followed by declarations..
            int len = sizeof(s->subscriptions)/sizeof(s->subscriptions[0]);
            int index = 0;

            // Check if subscription already exists for this code
            for (index = 0; index < len; index++) {
                if(s->subscriptions[index].code == ((SUBSCRIBEDATA*) (param->ptr))->code) {
                    break;
                }
            }

            // If code was not found, look for vacant subscription slot
            if(index == len) {
                for (index = 0; index < len; index++) {
                    // NOTE: if you set a count of 0, or the count runs out, then
                    // the subscription will be overwritten later -
                    // i.e. you effectively delete it....
                    if( s->subscriptions[index].code == 0 || s->subscriptions[index].count == 0 ) {
                        break;
                    }
                }
            }

            // Fill in new subscription when possible; Plausibility check for period
            if(index < len && ((SUBSCRIBEDATA*) (param->ptr))->period >= 10) {
                s->subscriptions[index] = *((SUBSCRIBEDATA*) (param->ptr));
                //char tmp[100];
                //sprintf(tmp, "subscription added at %d for 0x%x, period %d, count %d, som %d\n", index, ((SUBSCRIBEDATA*) (param->ptr))->code, ((SUBSCRIBEDATA*) (param->ptr))->period, ((SUBSCRIBEDATA*) (param->ptr))->count, ((SUBSCRIBEDATA*) (param->ptr))->som);
                //consoleLog(tmp);
            } else {
                // TODO. Inform sender??
                consoleLog("no subscriptions available\n");
            }
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x22 and 0x23 ProtocolcountData

PROTOCOLCOUNT ProtocolcountData =  { .rx = 0 };

void fn_ProtocolcountDataSum ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {

    fn_preWriteClear(s, param, fn_type, content, len); // Wipes memory before write (and readresponse is just a differenct type of writing)

    switch (fn_type) {

        case FN_TYPE_PRE_READ:
            ProtocolcountData.rx                  = s->ack.counters.rx                  + s->noack.counters.rx;
            ProtocolcountData.rxMissing           = s->ack.counters.rxMissing           + s->noack.counters.rxMissing;
            ProtocolcountData.tx                  = s->ack.counters.tx                  + s->noack.counters.tx;
            ProtocolcountData.txFailed            = s->ack.counters.txFailed            + s->noack.counters.txFailed;
            ProtocolcountData.txRetries           = s->ack.counters.txRetries           + s->noack.counters.txRetries;
            ProtocolcountData.unwantedacks        = s->ack.counters.unwantedacks        + s->noack.counters.unwantedacks;
            ProtocolcountData.unknowncommands     = s->ack.counters.unknowncommands     + s->noack.counters.unknowncommands;
            ProtocolcountData.unplausibleresponse = s->ack.counters.unplausibleresponse + s->noack.counters.unplausibleresponse;
            ProtocolcountData.unwantednacks       = s->ack.counters.unwantednacks       + s->noack.counters.unwantednacks;
            break;

        case FN_TYPE_POST_WRITE:
            s->ack.counters = ProtocolcountData;
            s->noack.counters = ProtocolcountData;
            break;
    }
}

void fn_ProtocolcountDataAck ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {

    fn_preWriteClear(s, param, fn_type, content, len); // Wipes memory before write (and readresponse is just a differenct type of writing)

    switch (fn_type) {
        case FN_TYPE_PRE_READ:
            ProtocolcountData = s->ack.counters;
            break;

        case FN_TYPE_POST_WRITE:
            s->ack.counters = ProtocolcountData;
            break;
    }
}

void fn_ProtocolcountDataNoack ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {

    fn_preWriteClear(s, param, fn_type, content, len); // Wipes memory before write (and readresponse is just a differenct type of writing)

    switch (fn_type) {
        case FN_TYPE_PRE_READ:
            ProtocolcountData = s->noack.counters;
            break;

        case FN_TYPE_POST_WRITE:
            s->noack.counters = ProtocolcountData;
            break;
    }
}

#ifndef EXCLUDE_DEADRECKONER

void fn_xytPosn ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ){
    switch (fn_type) {
        case FN_TYPE_POST_WRITE:
            if (deadreconer) {
                // reset xyt
                reset( deadreconer, 1 );
            }
            break;
    }

}
#endif


////////////////////////////////////////////////////////////
// allows read of parameter descritpions and variable length
// accepts (unsigned char first, unsigned char count) in read message!
// data returned
typedef struct tag_descriptions {
    unsigned char first;
    unsigned char count_read;
    char descriptions[251];
} DESCRIPTIONS;
DESCRIPTIONS paramstat_descriptions;
typedef struct tag_description {
    unsigned char len;    // overall length of THIS
    unsigned char code;   // code from params
    unsigned char var_len;// length of variable referenced
    unsigned char var_type;// UI_NONE or UI_SHORT for the moment
    char description[249];// nul term description
    // could be expanded here, as len at the top.
    // but 'description' above will be shorter than 249, so strucutre variabls can;t be used?
} DESCRIPTION;

void fn_paramstat_descriptions ( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ){
    switch (fn_type) {
        case FN_TYPE_PRE_READ: {
            // content[0] is the first entry to read.
            // content[1] is max count of entries to read
            // we prepare a buffer here, an MODIFY the paramstat length to tell it how much to send! (evil!?).
            int first = 0;
            int count = 100;
            if (len > 0) {
                first = content[0];
            }
            if (len > 1) {
                count = content[1];
            }

            if (first >= paramcount) {
                params[0].len = 0;
                return;
            }
            if (first + count >= paramcount){
                count = paramcount - first;
            }

            // now loop over requested entries until no more will fit in buffer,
            // informing on start and count done.
            int actual_count = 0;
            int len_out = 0;
            char *p = paramstat_descriptions.descriptions;
            for (int i = first; i < first+count; i++){
                int desc_len = 0;
                if (params[i].description) {
                    desc_len = strlen(params[i].description);
                }
                DESCRIPTION *d = (DESCRIPTION *)p;
                if (len_out+sizeof(*d)-sizeof(d->description)+desc_len+1 > sizeof(paramstat_descriptions.descriptions)){
                    break;
                }
                d->len = sizeof(*d)-sizeof(d->description)+desc_len+1;
                d->code = params[i].code;
                d->var_len = params[i].len;
                d->var_type = params[i].ui_type;
                if (desc_len) {
                    strcpy(d->description, params[i].description);
                } else {
                    d->description[0] = 0;
                }
                p += d->len;
                len_out = p - paramstat_descriptions.descriptions;
                actual_count++;
            }
            paramstat_descriptions.first = first;
            paramstat_descriptions.count_read = actual_count;
            param->len = sizeof(DESCRIPTIONS) - sizeof(paramstat_descriptions.descriptions) + len_out;
            break;
        }

        case FN_TYPE_POST_READ:
            param->len = 0; // reset to zero
            break;
    }
}



////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

// NOTE: Don't start uistr with 'a'
PARAMSTAT params[] = {
    // Protocol Relevant Parameters
    { 0xFF, "descriptions",            NULL,  UI_NONE,  &paramstat_descriptions,              0,                         PARAM_R,  fn_paramstat_descriptions },
    { 0x00, "version",                 NULL,  UI_LONG,  &version,                             sizeof(version),           PARAM_R,  NULL },
    { 0x22, "subscribe data",          NULL,  UI_NONE,  &SubscribeData,                       sizeof(SubscribeData),     PARAM_RW, fn_SubscribeData },
    { 0x23, "protocol stats ack+noack",NULL,  UI_NONE,  &ProtocolcountData,                   sizeof(PROTOCOLCOUNT),     PARAM_RW, fn_ProtocolcountDataSum },
    { 0x24, "protocol stats ack",      NULL,  UI_NONE,  &ProtocolcountData,                   sizeof(PROTOCOLCOUNT),     PARAM_RW, fn_ProtocolcountDataAck },
    { 0x25, "protocol stats noack",    NULL,  UI_NONE,  &ProtocolcountData,                   sizeof(PROTOCOLCOUNT),     PARAM_RW, fn_ProtocolcountDataNoack },

#ifdef CONTROL_SENSOR
    { 0x01, "sensor data",             NULL,  UI_NONE,  &sensor_copy,                         sizeof(sensor_copy),       PARAM_R,  fn_SensorData },
#endif
#ifdef HALL_INTERRUPTS
    { 0x02, "hall data",               NULL,  UI_NONE,  (void *)&HallData,                    sizeof(HallData),          PARAM_R,  NULL },
    { 0x03, "speed control mm/s",      NULL,  UI_NONE,  &SpeedData,                           sizeof(SpeedData),         PARAM_RW, fn_SpeedData },
    { 0x04, "hall position mm",        NULL,  UI_NONE,  &Position,                            sizeof(Position),          PARAM_RW, fn_Position },
    { 0x05, "position control increment mm",NULL,UI_NONE,&PositionIncr,                       sizeof(PositionIncr),      PARAM_RW, fn_PositionIncr },
    { 0x06, "position control mm",     NULL,  UI_NONE,  &PosnData,                            sizeof(PosnData),          PARAM_RW, fn_preWriteClear },
    { 0x07, "hall position steps",     NULL,  UI_NONE,  &RawPosition,                         sizeof(RawPosition),       PARAM_RW, fn_RawPosition },
#endif
    { 0x08, "electrical measurements", NULL,  UI_NONE,  (void *)&electrical_measurements,     sizeof(ELECTRICAL_PARAMS), PARAM_R,  NULL },
    { 0x09, "enable motors",           NULL,  UI_CHAR,  &enable,                              sizeof(enable),            PARAM_RW, fn_enable },
    { 0x0A, "disable poweroff timer",  NULL,  UI_CHAR,  &disablepoweroff,                     sizeof(disablepoweroff),   PARAM_RW, fn_preWriteClear },
    { 0x0B, "enable console logs",     NULL,  UI_CHAR,  &debug_out,                           sizeof(debug_out),         PARAM_RW, fn_preWriteClear },
#ifndef EXCLUDE_DEADRECKONER
    { 0x0C, "read/clear xyt position", NULL,  UI_3LONG, &xytPosn,                             sizeof(xytPosn),           PARAM_RW, fn_xytPosn },
#endif
    { 0x0D, "PWM control",             NULL,  UI_NONE,  &PWMData,                             sizeof(PWMData),           PARAM_RW, fn_PWMData },
    { 0x0E, "simpler PWM",             NULL,  UI_2LONG, &(PWMData.pwm),                       sizeof(PWMData.pwm),       PARAM_RW, fn_PWMData },
    { 0x21, "buzzer",                  NULL,  UI_NONE,  &BuzzerData,                          sizeof(BuzzerData),        PARAM_RW, fn_BuzzerData },

#ifdef FLASH_STORAGE
    { 0x80, "flash magic",             "m",   UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear },  // write this with CURRENT_MAGIC to commit to flash

    { 0x81, "posn kp x 100",           "pkp", UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear },
    { 0x82, "posn ki x 100",           "pki", UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // pid params for Position
    { 0x83, "posn kd x 100",           "pkd", UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear },
    { 0x84, "posn pwm lim",            "pl",  UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // e.g. 200

    { 0x85, "speed kp x 100",          "skp", UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear },
    { 0x86, "speed ki x 100",          "ski", UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // pid params for Speed
    { 0x87, "speed kd x 100",          "skd", UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear },
    { 0x88, "speed pwm incr lim",      "sl",  UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // e.g. 20
    { 0x89, "max current limit x 100", "cl",  UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // by default 1500 (=15 amps), limited by DC_CUR_LIMIT
    { 0xA0, "hoverboard enable",       "he",  UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // e.g. 20
#endif

    { 0xF0, "temp0", NULL, UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // Placeholder for application specific functions
    { 0xF1, "temp1", NULL, UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // Placeholder for application specific functions
    { 0xF2, "temp2", NULL, UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // Placeholder for application specific functions
    { 0xF3, "temp3", NULL, UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // Placeholder for application specific functions
    { 0xF4, "temp4", NULL, UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // Placeholder for application specific functions
    { 0xF5, "temp5", NULL, UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // Placeholder for application specific functions
    { 0xF6, "temp6", NULL, UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // Placeholder for application specific functions
    { 0xF7, "temp7", NULL, UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // Placeholder for application specific functions
    { 0xF8, "temp8", NULL, UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }, // Placeholder for application specific functions
    { 0xF9, "temp9", NULL, UI_NONE, NULL, 0, PARAM_R, fn_preWriteClear }  // Placeholder for application specific functions
};

int paramcount = sizeof(params)/sizeof(params[0]);


/////////////////////////////////////////////
// Change variable at runtime
int setParamVariable(unsigned char code, char ui_type, void *ptr, int len, char rw) {

    // Check if len can actually be received
    if( len > sizeof( ((PROTOCOL_BYTES_WRITEVALS *)0)->content ) ) {
        return 1;                           // Too long, Failure
    }

    for (int i = 0; i < paramcount; i++) {
        if (params[i].code == code) {
            params[i].ui_type = ui_type;
            params[i].ptr = ptr;
            params[i].len = len;
            params[i].rw = rw;
            return 0;                       // Success
        }
    }
    return 1;                               // Not found, Failure
}



/////////////////////////////////////////////
// Register new function handler at runtime
PARAMSTAT_FN setParamHandler(unsigned char code, PARAMSTAT_FN callback) {
    PARAMSTAT_FN old = NULL;

    for (int i = 0; i < paramcount; i++) {
        if (params[i].code == code) {
            old = params[i].fn;
            params[i].fn = callback;
        }
    }
    return old;
}

/////////////////////////////////////////////
// a complete machineprotocl message has been
// received without error
void protocol_process_message(PROTOCOL_STAT *s, PROTOCOL_MSG2 *msg){
    PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;

    switch (writevals->cmd){
        case PROTOCOL_CMD_READVAL:{
            int i;
            for (i = 0; i < sizeof(params)/sizeof(params[0]); i++){
                if (params[i].code == writevals->code){
                    if (params[i].fn) params[i].fn( s, &params[i], FN_TYPE_PRE_READ, writevals->content, msg->len-2 ); // NOTE: re-uses the msg object (part of stats)
                    unsigned char *src = params[i].ptr;
                    for (int j = 0; j < params[i].len; j++){
                        writevals->content[j] = *(src++);
                    }
                    msg->len = 1+1+params[i].len;  // command + code + data len only
                    writevals->cmd = PROTOCOL_CMD_READVALRESPONSE; // mark as response
                    // send back with 'read' command plus data like write.
                    protocol_post(s, msg);
                    if (params[i].fn) params[i].fn( s, &params[i], FN_TYPE_POST_READ, writevals->content, msg->len-2 );
                    break;
                }
            }
            // parameter code not found
            if (i == sizeof(params)/sizeof(params[0])){
                msg->len = 1+1; // cmd + code only
                writevals->cmd = PROTOCOL_CMD_READVALRESPONSE; // mark as response
                // send back with 'read' command plus data like write.
                protocol_post(s, msg);
            }
            break;
        }

        case PROTOCOL_CMD_READVALRESPONSE:{
            int i;
            for (i = 0; i < sizeof(params)/sizeof(params[0]); i++){
                if (params[i].code == writevals->code){
                    if (params[i].fn) params[i].fn( s, &params[i], FN_TYPE_PRE_READRESPONSE, writevals->content, msg->len-2 );

                    unsigned char *dest = params[i].ptr;
                    // ONLY copy what we have, else we're stuffing random data in.
                    // e.g. is setting posn, structure is 8 x 4 bytes,
                    // but we often only want to set the first 8
                    for (int j = 0; ((j < params[i].len) && (j < (msg->len-2))); j++){
                        *(dest++) = writevals->content[j];
                    }
                    if (params[i].fn) params[i].fn( s, &params[i], FN_TYPE_POST_READRESPONSE, writevals->content,  msg->len-2 );
                    break;
                }
            }
            // parameter code not found
            if (i == sizeof(params)/sizeof(params[0])){
                if(msg->SOM == PROTOCOL_SOM_ACK) {
                    s->ack.counters.unplausibleresponse++;
                } else {
                    s->noack.counters.unplausibleresponse++;
                }
            }
            break;
        }

        case PROTOCOL_CMD_WRITEVALRESPONSE:{
            int i;
            for (i = 0; i < sizeof(params)/sizeof(params[0]); i++){
                if (params[i].code == writevals->code){
                    break;
                }
            }
            // parameter code not found
            if (i == sizeof(params)/sizeof(params[0])){
                if(msg->SOM == PROTOCOL_SOM_ACK) {
                    s->ack.counters.unplausibleresponse++;
                } else {
                    s->noack.counters.unplausibleresponse++;
                }
            }
            break;
        }

        case PROTOCOL_CMD_WRITEVAL:{
            int i;
            for (i = 0; i < sizeof(params)/sizeof(params[0]); i++){
                if (params[i].code == writevals->code){
                    if (params[i].fn) params[i].fn( s, &params[i], FN_TYPE_PRE_WRITE, writevals->content, msg->len-2 );
                    // NOTE: re-uses the msg object (part of stats)
                    unsigned char *dest = params[i].ptr;

                    // ONLY copy what we have, else we're stuffing random data in.
                    // e.g. is setting posn, structure is 8 x 4 bytes,
                    // but we often only want to set the first 8
                    for (int j = 0; ((j < params[i].len) && (j < (msg->len-2))); j++){
                        *(dest++) = writevals->content[j];
                    }
                    msg->len = 1+1+1; // cmd+code+'1' only
                    writevals->cmd = PROTOCOL_CMD_WRITEVALRESPONSE; // mark as response
                    writevals->content[0] = 1; // say we wrote it
                    // send back with 'write' command with no data.
                    protocol_post(s, msg);
                    if (params[i].fn) params[i].fn( s, &params[i], FN_TYPE_POST_WRITE, writevals->content, msg->len-2 );
                    break;
                }
            }
            // parameter code not found
            if (i == sizeof(params)/sizeof(params[0])){
                msg->len = 1+1+1; // cmd +code +'0' only
                writevals->cmd = PROTOCOL_CMD_WRITEVALRESPONSE; // mark as response
                writevals->content[0] = 0; // say we did not write it
                // send back with 'write' command plus data like write.
                protocol_post(s, msg);
            }
            break;
        }

        case PROTOCOL_CMD_REBOOT:
            //protocol_send_ack(); // we no longer ack from here
            HAL_Delay(500);
            HAL_NVIC_SystemReset();
            break;

        case PROTOCOL_CMD_TEST:
            // just send it back!
            writevals->cmd = PROTOCOL_CMD_TESTRESPONSE;
            // note: original 'bytes' sent back, so leave len as is
            protocol_post(s, msg);
            // post second immediately to test buffering
            // protocol_post(s, msg);
            break;

        case PROTOCOL_CMD_UNKNOWN:
            // Do nothing, otherwise endless loop is entered.
            if(msg->SOM == PROTOCOL_SOM_ACK) {
                s->ack.counters.unknowncommands++;
            } else {
                s->noack.counters.unknowncommands++;
            }
            break;

        default:
            if(msg->SOM == PROTOCOL_SOM_ACK) {
                s->ack.counters.unknowncommands++;
            } else {
                s->noack.counters.unknowncommands++;
            }            writevals->cmd = PROTOCOL_CMD_UNKNOWN;
            msg->len = 1;
            protocol_post(s, msg);
        break;
    }
}



#endif
