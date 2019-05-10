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
#pragma once

#include "config.h"
#include <stdint.h>

//// control structures used in firmware
typedef struct tag_POSN_DATA {
    // these get set
    long wanted_posn_mm[2];

    // configurations/constants
    int posn_max_speed; // max speed in this mode
    int posn_min_speed; // minimum speed (to get wheels moving)

    // just so it can be read back
    long posn_diff_mm[2];
    long posn_speed_demand[2];
} POSN_DATA;

extern POSN_DATA PosnData;

typedef struct tag_SPEED_DATA {
    // these get set
    long wanted_speed_mm_per_sec[2];

    // configurations/constants
    int speed_max_power; // max speed in this mode
    int speed_min_power; // minimum speed (to get wheels moving)
    int speed_minimum_speed; // below this, we don't ask it to do anything

    // just so it can be read back
    long speed_diff_mm_per_sec[2];
    long speed_power_demand[2];
} SPEED_DATA;

extern SPEED_DATA SpeedData;

typedef struct tag_PWM_DATA {
    // these get set
    long pwm[2];

    // configurations/constants
    int speed_max_power; // max speed in this mode
    int speed_min_power; // minimum speed (to get wheels moving)
    int speed_minimum_pwm; // below this, we don't ask it to do anything
} PWM_DATA;

extern PWM_DATA PWMData;




typedef struct {
    uint8_t buzzerFreq;
    uint8_t buzzerPattern;
    uint16_t buzzerLen;
} BUZZER_DATA;

extern BUZZER_DATA BuzzerData;


extern int control_type;
#define CONTROL_TYPE_NONE 0
#define CONTROL_TYPE_POSITION 1
#define CONTROL_TYPE_SPEED 2
#define CONTROL_TYPE_PWM 3
#define CONTROL_TYPE_MAX 4


/////////////////////////////////////
// the rest only if we have a protocol.
#if (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL2)

/////////////////////////////////////////////////////////////////
// 'machine' protocol structures and definitions
//
// examples:
// ack - 02 02 41 BD
// nack - 02 02 4E B0
// test - 02 06 54 54 65 73 74 06
/////////////////////////////////////////////////////////////////

#pragma pack(push, 1)
typedef struct tag_PROTOCOL_MSG2 {
    unsigned char SOM; // 0x02
    unsigned char CI; // continuity counter
    unsigned char len; // len is len of bytes to follow, NOT including CS
    unsigned char bytes[255];  // variable number of data bytes, with a checksum on the end, cmd is first
    // checksum such that sum of bytes CI to CS is zero
} PROTOCOL_MSG2;

typedef struct tag_PROTOCOL_LEN_ONWARDS {
    unsigned char len; // len is len of ALL bytes to follow, including CS
    unsigned char bytes[255];  // variable number of data bytes, with a checksum on the end, cmd is first
} PROTOCOL_LEN_ONWARDS;

// content of 'bytes' above, for single byte commands
typedef struct tag_PROTOCOL_BYTES {
    unsigned char cmd; //
    unsigned char bytes[254];
} PROTOCOL_BYTES;


// content of 'bytes' above, for single byte commands
#define PROTOCOL_CMD_READVAL 'R'
#define PROTOCOL_CMD_READVALRESPONSE 'r'
typedef struct tag_PROTOCOL_BYTES_READVALS {
    unsigned char cmd; // 'R'
    unsigned char code; // code of value to read
} PROTOCOL_BYTES_READVALS;

#define PROTOCOL_CMD_WRITEVAL 'W'
#define PROTOCOL_CMD_WRITEVALRESPONSE 'w'
typedef struct tag_PROTOCOL_BYTES_WRITEVALS {
    unsigned char cmd; // 'W'
    unsigned char code; // code of value to write
    unsigned char content[252]; // value to write
} PROTOCOL_BYTES_WRITEVALS;



//////////////////////////////////////////////////////////////////
// protocol_post uses this structure to store outgoing messages
// until they can be sent.
// messages are stored only as len|data
// SOM, CI, and CS are not included.
#define MACHINE_PROTOCOL_TX_BUFFER_SIZE 1024
typedef struct tag_MACHINE_PROTOCOL_TX_BUFFER {
    volatile unsigned char buff[MACHINE_PROTOCOL_TX_BUFFER_SIZE];
    volatile int head;
    volatile int tail;

    // count of buffer overflows
    volatile unsigned int overflow;

} MACHINE_PROTOCOL_TX_BUFFER;

//////////////////////////////////////////////////////////

typedef struct tag_SUBSCRIBEDATA {
    unsigned char code;              // code in protocol to refer to this
    unsigned int period;             // how often should the information be sent?
    int count;                       // how many messages shall be sent? -1 for infinity
    char som;                        // which SOM shall be used? with or without ACK
    unsigned long next_send_time;    // last time a message requiring an ACK was sent

} SUBSCRIBEDATA;



typedef struct tag_PROTOCOL_STAT {
    char allow_ascii;                     // If set to 0, ascii protocol is not used
    unsigned long last_send_time;         // last time a message requiring an ACK was sent
    unsigned long last_tick_time;         // last time the tick function was called

    char state;                           // state used in protocol_byte to receive packages
    unsigned long last_char_time;         // last time a character was received
    unsigned char CS;                     // temporary storage to calculate Checksum
    unsigned char count;                  // index pointing to last received byte
    unsigned int nonsync;                 // not used?
    PROTOCOL_MSG2 curr_msg;               // received message storage
    unsigned char lastRXCI;               // CI of last received message

    unsigned int unwantedacks;            // count of unwated ACK messages
    unsigned int unwantednacks;           // count of unwanted NACK messges
    unsigned int unknowncommands;         // count of messages wit unknown commands
    unsigned int unplausibleresponse;     // count of unplausible replies
    unsigned int missingRXmessages;       // count of missing RX messages

    char send_state;                      // message transmission state
    PROTOCOL_MSG2 curr_send_msg_withAck;  // transmit message storage (for messages with ACK)
    PROTOCOL_MSG2 curr_send_msg_noAck;    // transmit message storage (for messages without ACK)
    char retries;                         // number of retries left to send message
    unsigned char lastTXCI;               // CI of last sent message

    int timeout1;                         // ACK has to be received in this time
    int timeout2;                         // when receiving a packet, longest time between characters

    int (*send_serial_data)( unsigned char *data, int len );       // Function Pointer to sending function
    int (*send_serial_data_wait)( unsigned char *data, int len );

    MACHINE_PROTOCOL_TX_BUFFER TxBufferACK;    // Buffer for Messages with ACK to be sent
    MACHINE_PROTOCOL_TX_BUFFER TxBufferNoACK;  // Buffer for Messages without ACK to be sent

    SUBSCRIBEDATA subscriptions[10];           // Subscriptions to periodic messages

} PROTOCOL_STAT;

#pragma pack(pop)


///////////////////////////////////////////////////
// structure used to gather variables we want to read/write.
#define PARAM_R     1
#define PARAM_RW    3
///////////////////////////////////////////////////
#define UI_NONE 0
#define UI_SHORT 1

#pragma pack(push, 1)
typedef struct tag_PARAMSTAT {
    unsigned char code;     // code in protocol to refer to this
    char *description;      // if non-null, description
    char *uistr;            // if non-null, used in ascii protocol to adjust with f<str>num<cr>
    char ui_type;           // only UI_NONE or UI_SHORT
    void *ptr;              // pointer to value
    char len;               // length of value
    char rw;                // PARAM_R or PARAM_RW

    void (*preread)(PROTOCOL_STAT *s);                // function to call before read
    void (*postread)(PROTOCOL_STAT *s);               // function to call after read
    void (*prewrite)(PROTOCOL_STAT *s);               // function to call before write
    void (*postwrite)(PROTOCOL_STAT *s);              // function to call after write
    void (*receivedread)(PROTOCOL_STAT *s);           // function to call after requested data was received
} PARAMSTAT;


#pragma pack(pop)


/////////////////////////////////////////////////////////
// command definitions
// ack - no payload
#define PROTOCOL_CMD_ACK 'A'
// nack - no payload
#define PROTOCOL_CMD_NACK 'N'

// a test command - normal payload - 'Test'
#define PROTOCOL_CMD_TEST 'T'
#define PROTOCOL_CMD_TESTRESPONSE 't'

// cause unit to restart - no payload
#define PROTOCOL_CMD_REBOOT 'B'

// response to an unkonwn command - maybe payload
#define PROTOCOL_CMD_UNKNOWN '?'

#define PROTOCOL_SOM_ACK 2
#define PROTOCOL_SOM_NOACK 4
//
/////////////////////////////////////////////////////////////////

#pragma pack(push, 1)

typedef struct tag_POSN {
    long LeftAbsolute;
    long RightAbsolute;
    long LeftOffset;
    long RightOffset;
} POSN;

typedef struct tag_POSN_INCR {
    long Left;
    long Right;
} POSN_INCR;
extern int enable_immediate;
#pragma pack(pop)


extern PROTOCOL_STAT sUSART2;
extern PROTOCOL_STAT sUSART3;
extern PROTOCOL_STAT sSoftwareSerial;

/////////////////////////////////////////////////////////////////
// call this with received bytes; normally from main loop
void protocol_byte( PROTOCOL_STAT *s, unsigned char byte );

/////////////////////////////////////////////////////////////////
// call this schedule a message. CI and Checksum are added
int protocol_post(PROTOCOL_STAT *s, PROTOCOL_MSG2 *msg);

/////////////////////////////////////////////////////////////////
// call this regularly from main.c
void protocol_tick(PROTOCOL_STAT *s);

/////////////////////////////////////////////////////////////////
// initialize protocol
void protocol_init(PROTOCOL_STAT *s);

/////////////////////////////////////////////////////////////////
// processes ASCII characters
void ascii_byte(PROTOCOL_STAT *s, unsigned char byte );

/////////////////////////////////////////////////////////////////
// processes machine protocol messages
void protocol_process_message(PROTOCOL_STAT *s, PROTOCOL_MSG2 *msg);

/////////////////////////////////////////////////////////////////
// get buffer level
int mpTxQueued(MACHINE_PROTOCOL_TX_BUFFER *buf);

/////////////////////////////////////////////////////////////////
// callback which can be used for "debugging"
extern void (*debugprint)(const char str[]);

#endif
