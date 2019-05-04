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
#ifdef SOFTWARE_SERIAL
    #include "softwareserial.h"
#endif
#ifndef SKIP_ELECTRICAL_MEASUREMENTS
    #include "bldc.h"
#endif
#ifdef FLASH_STORAGE
    #include "flashcontent.h"
    #include "flashaccess.h"
#endif
#include "comms.h"
#ifndef EXCLUDE_DEADRECKONER
    #include "deadreckoner.h"
#endif

#include <string.h>
#include <stdlib.h>

#ifndef EXCLUDE_DEADRECKONER
// ded reckoning posn
extern INTEGER_XYT_POSN xytPosn;
#endif

//////////////////////////////////////////////////////////
// Function pointer which can be set for "debugging"
void noprint(const char str[]) {};
void (*debugprint)(const char str[]) = noprint;

//////////////////////////////////////////////////////////
// things needed by main.c
int control_type = 0;
#ifdef HALL_INTERRUPTS
POSN_DATA PosnData = {
    {0, 0},

    200, // max pwm in posn mode
    70, // min pwm in posn mode
};
#endif
SPEED_DATA SpeedData = {
    {0, 0},

    600, // max power (PWM)
    -600,  // min power
    40 // minimum mm/s which we can ask for
};

BUZZER_DATA BuzzerData = {
    .buzzerFreq = 0,
    .buzzerPattern = 0,
    .buzzerLen = 0,
};

//////////////////////////////////////////////////////////




#if (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL2)

//////////////////////////////////////////////////////////
//
// Machine protocol:
// a very simple protocol, starting 02 (SOM), with length and checksum
// examples:
// ack - 02 02 41 BD
// nack - 02 02 4E B0
// test - 02 06 54 54 65 73 74 06
// e.g. for test:
// 02 - SOM
// 06 - length = 6
// 54 - byte0 - 'cmd' 'T'
// 54 - byte1 - payload for text command - 'T'est
// 65 - byte2 - 'e'
// 73 - byte3 - 's'
// 74 - byte4 - 't'
// 06 - checksum = (00 - (06+54+54+65+73+74))&0xff = 06,
// or  can be stated as : (06+54+54+65+73+74+06)&0xff = 0
//
// if a message is received with invalid checksum, then nack will be sent.
// if a message is received complete, it will with be responded to with a
// return message, or with the ack message
//
// for simplicities sake, we will treat the hoverboard controller as a
// slave unit always - i.e. not ask it to send *unsolicited* messages.
// in this way, it does not need to wait for ack, etc. from the host.
// if the host gets a bad message, or no response, it can retry.
//
//////////////////////////////////////////////////////////



///////////////////////////////////////////////
// extern variables you want to read/write here
#ifdef CONTROL_SENSOR
extern SENSOR_DATA sensor_data[2];
extern int sensor_control;
extern int sensor_stabilise;
#endif

#ifdef FLASH_STORAGE
// from main.c
extern void change_PID_constants();
extern void init_PID_control();
#endif

extern uint8_t enable; // global variable for motor enable
extern volatile uint32_t timeout; // global variable for timeout
extern int dspeeds[2];
extern int pwms[2];

extern uint8_t debug_out;
extern uint8_t disablepoweroff;
extern int powerofftimer;
extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
extern uint16_t buzzerLen;
extern uint8_t enablescope; // enable scope on values

#ifndef SKIP_ELECTRICAL_MEASUREMENTS
    extern volatile ELECTRICAL_PARAMS electrical_measurements;
#endif
///////////////////////////////////////////////


/////////////////////////////////////////////////////////////
// specify where to send data out of with a function pointer.

#ifdef SOFTWARE_SERIAL

PROTOCOL_STAT sSoftwareSerial = {
    .send_serial_data=softwareserial_Send,
    .send_serial_data_wait=softwareserial_Send_Wait,
    .timeout1 = 500,
    .timeout2 = 100,
    .allow_ascii = 1
};
#endif

#if defined(SERIAL_USART2_IT) && !defined(READ_SENSOR)

extern int USART2_IT_send(unsigned char *data, int len);

PROTOCOL_STAT sUSART2 = {
    .send_serial_data=USART2_IT_send,
    .send_serial_data_wait=USART2_IT_send,
    .timeout1 = 500,
    .timeout2 = 100,
    .allow_ascii = 1
};

#endif

#if defined(SERIAL_USART3_IT) && !defined(READ_SENSOR)

extern int USART3_IT_send(unsigned char *data, int len);

PROTOCOL_STAT sUSART3 = {
    .send_serial_data=USART3_IT_send,
    .send_serial_data_wait=USART3_IT_send,
    .timeout1 = 500,
    .timeout2 = 100,
    .allow_ascii = 1
};

#endif
/////////////////////////////////////////////////////////////

extern int protocol_post(PROTOCOL_STAT *s, PROTOCOL_MSG2 *msg);


//////////////////////////////////////////////
// variables and functions in support of parameters here
//

// e.g. to gather two separate speed variables togther,
typedef struct tag_SPEEDS{
    int speedl;
    int speedr;
} SPEEDS;
static SPEEDS speedsx = {0,0};

// after write we call this...
void PostWrite_setbuzzer(void){
    buzzerFreq      = BuzzerData.buzzerFreq;
    buzzerLen       = BuzzerData.buzzerLen;
    buzzerPattern   = BuzzerData.buzzerPattern;
}

// before read we call this...
void PreRead_getbuzzer(void){
    BuzzerData.buzzerFreq       = buzzerFreq;
    BuzzerData.buzzerLen        = buzzerLen;
    BuzzerData.buzzerPattern    = buzzerPattern;
}

// before read we call this...
void PreRead_getspeeds(void){
    speedsx.speedl = SpeedData.wanted_speed_mm_per_sec[0];
    speedsx.speedr = SpeedData.wanted_speed_mm_per_sec[1];
}

//////////////////////////////////////////////
// make values safe before we change enable...
void PreWrite_enable() {
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
}


void PreWrite_setspeeds(void){
    PreWrite_enable();
    enable = 1;
    control_type = CONTROL_TYPE_SPEED;
    timeout = 0;
}


// after write we call this...
void PostWrite_setspeeds(void){
    // SpeedData.wanted_speed_mm_per_sec[0] = speedsx.speedl;
    // SpeedData.wanted_speed_mm_per_sec[1] = speedsx.speedr;
}


PWM_DATA PWMData = {
    .pwm[0] = 0,
    .pwm[1] = 0,
    .speed_max_power =  600,
    .speed_min_power = -600,
    .speed_minimum_pwm = 40 // guard value, below this set to zero
};

void PreWrite_setpwms(void){
    PreWrite_enable();
    enable = 1;
    control_type = CONTROL_TYPE_PWM;
    timeout = 0;
}


void PostWrite_setpwms() {
    for (int i = 0; i < 2; i++) {
        if (PWMData.pwm[i] > PWMData.speed_max_power) {
            PWMData.pwm[i] = PWMData.speed_max_power;
        }
        if (PWMData.pwm[i] < PWMData.speed_min_power) {
            PWMData.pwm[i] = PWMData.speed_min_power;
        }
        if ((PWMData.pwm[i] > 0) && (PWMData.pwm[i] < PWMData.speed_minimum_pwm)) {
            PWMData.pwm[i] = 0;
        }
        if ((PWMData.pwm[i] < 0) && (PWMData.pwm[i] > -PWMData.speed_minimum_pwm)) {
            PWMData.pwm[i] = 0;
        }
    }
}




#ifdef HALL_INTERRUPTS
POSN Position;
POSN RawPosition;

void PreRead_getposnupdate(){
    Position.LeftAbsolute = HallData[0].HallPosn_mm;
    Position.LeftOffset = HallData[0].HallPosn_mm - HallData[0].HallPosn_mm_lastread;
    Position.RightAbsolute = HallData[1].HallPosn_mm;
    Position.RightOffset = HallData[1].HallPosn_mm - HallData[1].HallPosn_mm_lastread;
}

void PostWrite_setposnupdate(){
    HallData[0].HallPosn_mm_lastread = Position.LeftAbsolute;
    HallData[1].HallPosn_mm_lastread = Position.RightAbsolute;
}

void PreRead_getrawposnupdate(){
    RawPosition.LeftAbsolute = HallData[0].HallPosn;
    RawPosition.LeftOffset = HallData[0].HallPosn - HallData[0].HallPosn_lastread;
    RawPosition.RightAbsolute = HallData[1].HallPosn;
    RawPosition.RightOffset = HallData[1].HallPosn - HallData[1].HallPosn_lastread;
}

void PostWrite_setrawposnupdate(){
    HallData[0].HallPosn_lastread = RawPosition.LeftAbsolute;
    HallData[1].HallPosn_lastread = RawPosition.RightAbsolute;
}


POSN_INCR PositionIncr;

void PostWrite_incrementposition(){
    // if switching to control type POSITION,
    if ((control_type != CONTROL_TYPE_POSITION) || !enable) {
        control_type = CONTROL_TYPE_POSITION;
        // then make sure we won't rush off somwehere strange
        // by setting our wanted posn to where we currently are...
        PreWrite_enable();
    }

    enable = 1;
    timeout = 0;

    // increment our wanted position
    PosnData.wanted_posn_mm[0] += PositionIncr.Left;
    PosnData.wanted_posn_mm[1] += PositionIncr.Right;
}

#endif

#ifdef FLASH_STORAGE

void PostWrite_writeflash(){
    if (FlashContent.magic != CURRENT_MAGIC){
        char temp[128];
        sprintf(temp, "incorrect magic %d, should be %d\r\nFlash not written\r\n", FlashContent.magic, CURRENT_MAGIC);
        consoleLog(temp);
        FlashContent.magic = CURRENT_MAGIC;
        return;
    }
    writeFlash( (unsigned char *)&FlashContent, sizeof(FlashContent) );
    consoleLog("wrote flash\r\n");
}

void PostWrite_PID(){
    change_PID_constants();
}

void PostWrite_Cur_Limit(){
    electrical_measurements.dcCurLim = MIN(DC_CUR_LIMIT, FlashContent.MaxCurrLim / 100);
}
#endif


static int version = 1;

// NOTE: Don't start uistr with 'a'
PARAMSTAT params[] = {
    { 0x00, NULL, NULL, UI_NONE, &version,          sizeof(version),         PARAM_R,  NULL,                     NULL, NULL,               NULL,                        NULL },
#ifdef CONTROL_SENSOR
    { 0x01, NULL, NULL, UI_NONE, &sensor_data,      sizeof(sensor_data),     PARAM_R,  NULL,                     NULL, NULL,               NULL,                        NULL },
#endif
#ifdef HALL_INTERRUPTS
    { 0x02, NULL, NULL, UI_NONE, (void *)&HallData, sizeof(HallData),        PARAM_R,  NULL,                     NULL, NULL,               NULL,                        NULL },
#endif
    { 0x03, NULL, NULL, UI_NONE, &SpeedData,        sizeof(SpeedData),       PARAM_RW, PreRead_getspeeds,        NULL, PreWrite_setspeeds, PostWrite_setspeeds,         NULL },
#ifdef HALL_INTERRUPTS
    { 0x04, NULL, NULL, UI_NONE, &Position,         sizeof(Position),        PARAM_RW, PreRead_getposnupdate,    NULL, NULL,               PostWrite_setposnupdate,     NULL },
    { 0x05, NULL, NULL, UI_NONE, &PositionIncr,     sizeof(PositionIncr),    PARAM_RW, NULL,                     NULL, NULL,               PostWrite_incrementposition, NULL },
    { 0x06, NULL, NULL, UI_NONE, &PosnData,         sizeof(PosnData),        PARAM_RW, NULL,                     NULL, NULL,               NULL,                        NULL },
    { 0x07, NULL, NULL, UI_NONE, &RawPosition,      sizeof(RawPosition),     PARAM_RW, PreRead_getrawposnupdate, NULL, NULL,               PostWrite_setrawposnupdate,  NULL },
#endif
    { 0x09, NULL, NULL, UI_NONE, &enable,           sizeof(enable),          PARAM_RW, NULL,                     NULL, PreWrite_enable,    NULL,                        NULL },
    { 0x0A, NULL, NULL, UI_NONE, &disablepoweroff,  sizeof(disablepoweroff), PARAM_RW, NULL,                     NULL, NULL,               NULL,                        NULL },
    { 0x0B, NULL, NULL, UI_NONE, &debug_out,        sizeof(debug_out),       PARAM_RW, NULL,                     NULL, NULL,               NULL,                        NULL },
#ifndef EXCLUDE_DEADRECKONER
    { 0x0C, NULL, NULL, UI_NONE, &xytPosn,          sizeof(xytPosn),         PARAM_RW, NULL,                     NULL, NULL,               NULL,                        NULL },
#endif
    { 0x0D, NULL, NULL, UI_NONE, &PWMData,          sizeof(PWMData),         PARAM_RW, NULL,                     NULL, PreWrite_setpwms,   PostWrite_setpwms,           NULL },
    { 0x0E, NULL, NULL, UI_NONE, &(PWMData.pwm),    sizeof(PWMData.pwm),     PARAM_RW, NULL,                     NULL, PreWrite_setpwms,   PostWrite_setpwms,           NULL },
    { 0x21, NULL, NULL, UI_NONE, &BuzzerData,       sizeof(BuzzerData),      PARAM_RW, PreRead_getbuzzer,        NULL, NULL,               PostWrite_setbuzzer,         NULL },

#ifdef FLASH_STORAGE
    { 0x80, "flash magic",             "m",   UI_SHORT, &FlashContent.magic,                  sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_writeflash, NULL },  // write this with CURRENT_MAGIC to commit to flash

    { 0x81, "posn kp x 100",           "pkp", UI_SHORT, &FlashContent.PositionKpx100,         sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID,       NULL },
    { 0x82, "posn ki x 100",           "pki", UI_SHORT, &FlashContent.PositionKix100,         sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID,       NULL }, // pid params for Position
    { 0x83, "posn kd x 100",           "pkd", UI_SHORT, &FlashContent.PositionKdx100,         sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID,       NULL },
    { 0x84, "posn pwm lim",            "pl",  UI_SHORT, &FlashContent.PositionPWMLimit,       sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID,       NULL }, // e.g. 200

    { 0x85, "speed kp x 100",          "skp", UI_SHORT, &FlashContent.SpeedKpx100,            sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID,       NULL },
    { 0x86, "speed ki x 100",          "ski", UI_SHORT, &FlashContent.SpeedKix100,            sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID,       NULL }, // pid params for Speed
    { 0x87, "speed kd x 100",          "skd", UI_SHORT, &FlashContent.SpeedKdx100,            sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID,       NULL },
    { 0x88, "speed pwm incr lim",      "sl",  UI_SHORT, &FlashContent.SpeedPWMIncrementLimit, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID,       NULL }, // e.g. 20
    { 0x89, "max current limit x 100", "cl",  UI_SHORT, &FlashContent.MaxCurrLim,             sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_Cur_Limit, NULL }, // by default 1500 (=15 amps), limited by DC_CUR_LIMIT
    { 0xA0, "hoverboard enable",       "he",  UI_SHORT, &FlashContent.HoverboardEnable,       sizeof(short), PARAM_RW, NULL, NULL, NULL, NULL,                NULL } // e.g. 20
#endif
};

int paramcount = sizeof(params)/sizeof(params[0]);



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
                    if (params[i].preread) params[i].preread();
                    // NOTE: re-uses the msg object (part of stats)
                    unsigned char *src = params[i].ptr;
                    for (int j = 0; j < params[i].len; j++){
                        writevals->content[j] = *(src++);
                    }
                    msg->len = 1+1+params[i].len;  // command + code + data len only
                    writevals->cmd = PROTOCOL_CMD_READVALRESPONSE; // mark as response
                    // send back with 'read' command plus data like write.
                    protocol_post(s, msg);
                    if (params[i].postread) params[i].postread();
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
                    if (params[i].receivedread) params[i].receivedread();

                    unsigned char *dest = params[i].ptr;
                    // ONLY copy what we have, else we're stuffing random data in.
                    // e.g. is setting posn, structure is 8 x 4 bytes,
                    // but we often only want to set the first 8
                    for (int j = 0; ((j < params[i].len) && (j < (msg->len-2))); j++){
                        *(dest++) = writevals->content[j];
                    }

                    break;
                }
            }
            // parameter code not found
            if (i == sizeof(params)/sizeof(params[0])){
                s->unplausibleresponse++;
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
                s->unplausibleresponse++;
            }
            break;
        }

        case PROTOCOL_CMD_WRITEVAL:{
            int i;
            for (i = 0; i < sizeof(params)/sizeof(params[0]); i++){
                if (params[i].code == writevals->code){
                    if (params[i].prewrite) params[i].prewrite();
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
                    if (params[i].postwrite) params[i].postwrite();
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
            s->unknowncommands++;
            break;

        default:
            s->unknowncommands++;
            writevals->cmd = PROTOCOL_CMD_UNKNOWN;
            msg->len = 1;
            protocol_post(s, msg);
        break;
    }
}



#endif
