/*
TWC Manager for ESP32
Copyright (C) 2020 Craig Peacock
Copyright (C) 2020 Jarl Nicolson
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 3
of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef TWC_PROTOCOL_H
#define TWC_PROTOCOL_H

#define GET_SERIAL_NUMBER	    0xFB19
#define GET_MODEL_NUMBER	    0xFB1A
#define GET_FIRMWARE_VER 	    0xFB1B
#define GET_PLUG_STATE		    0xFBB4

#define PRIMARY_HEATBEAT		0xFBE0
#define PRIMARY_PRESENCE2	    0xFBE2

#define GET_SECONDARY_PWR_STATE 0xFBEB
#define GET_FIRMWARE_VER_EXT    0xFBEC
#define GET_VIN_FIRST		    0xFBEE
#define GET_VIN_MIDDLE		    0xFBEF
#define GET_VIN_LAST		    0xFBF1

// Commands without responses (0xFC)
#define IDLE_MESSAGE            0xFC1D
#define START_CHARGING		    0xFCB1
#define STOP_CHARGING		    0xFCB2
#define PRIMARY_PRESENCE	    0xFCE1

// Responses (0xFD)
#define RESP_SERIAL_NUMBER	    0xFD19
#define RESP_MODEL_NUMBER	    0xFD1A
#define RESP_FIRMWARE_VER	    0xFD1B
#define RESP_PLUG_STATE		    0xFDB4

#define SECONDARY_HEARTBEAT		0xFDE0

#define SECONDARY_PRESENCE	    0xFDE2		// Sent by secondary on reset
#define PWR_STATUS              0xFDEB		// Sent by primary on reset
#define SECONDARY_VIN_FIRST	    0xFDEE
#define SECONDARY_VIN_MIDDLE    0xFDEF
#define SECONDARY_VIN_LAST	    0xFDF1

// Commands are SLIP encoded on the wire - need the SLIP constants
#define SLIP_END        0xC0
#define SLIP_ESC        0xDB
#define SLIP_ESC_END    0xDC
#define SLIP_ESC_ESC    0xDD

#pragma pack(1)

typedef struct PACKET_T {
    uint16_t command;
    uint16_t twcid;
    uint16_t secondary_twcid;
    uint8_t payload[8];
    uint8_t checksum;
} PACKET_T;

typedef struct VIN_T {
    uint16_t command;
    uint16_t twcid;
    uint8_t vin[7];
    uint8_t padding[8];
    uint8_t checksum;
} VIN_T;

typedef struct PRESENCE_T {
    uint16_t    command;
    uint16_t    twcid;
    uint8_t     sign;
    uint16_t    max_charge_rate;
    uint8_t     padding[8];
    uint8_t     checksum;
} PRESENCE_T;

typedef struct FIRMWARE_T {
    uint16_t command;
    uint8_t major;
    uint8_t minor;
    uint8_t revision;
    uint8_t padding[8];
    uint8_t checksum;
} FIRMWARE_T;

typedef struct PLUGSTATE_T {
    uint16_t command;
    uint16_t twcid;
    uint8_t plug_state;
    uint8_t padding[10];
    uint8_t checksum;
} PLUGSTATE_T;

typedef struct S_HEARTBEAT_T {
    uint16_t command;
    uint16_t src_twcid;
    uint16_t dst_twcid;
    uint8_t status;
    uint16_t max_current;
    uint16_t actual_current;
    uint8_t padding[4];
    uint8_t checksum;
} S_HEARTBEAT_T; 

typedef struct P_HEARTBEAT_T {
    uint16_t command;
    uint16_t src_twcid;
    uint16_t dst_twcid;
    uint8_t payload;
    uint16_t max_current;
    uint8_t plug_inserted;
    uint8_t padding[3];
    uint8_t checksum;
} P_HEARTBEAT_T; 

typedef struct POWERSTATUS_T {
    uint16_t command;
    uint16_t twcid;
    uint32_t total_kwh;
    uint16_t phase1_voltage;
    uint16_t phase2_voltage;
    uint16_t phase3_voltage;
    uint8_t phase1_current;
    uint8_t phase2_current;
    uint8_t phase3_current;
    uint8_t padding[2];
    uint8_t checksum;
} POWERSTATUS_T;
#endif /* TWC_PROTOCOL_H */