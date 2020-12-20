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

#include "twc_controller.h"
#include "mqtt.h"
#include "twc_connector.h"

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

// The next two commands are ** DANGEROUS ** !
// DO NOT USE THESE!  They are defined so that
// they can be blocked.
#define WRITE_ID_DATE           0xFC19
#define WRITE_MODEL_NO          0xFC1A

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




class TeslaController {
    public:
        TeslaController(HardwareSerial& serial, TeslaControllerIO& io);

        void Begin();
        void GetPowerStatus();
        void GetFirmware();        
        void GetVIN();
        void GetSerial();
        void GetModelNo();
        void GetFirmwareVer();
        void GetPlugState();
        void GetVin(uint16_t secondary_twcid);
        void Handle();

        void SendCommand(uint16_t command, uint16_t send_to);
        void SendPresence(bool presence2 = false);
        void SendPresence2();
        void SendHeartbeat(uint16_t secondary_twcid);
        void SendIdle();
        void Startup();
        void Debug(bool enabled);
        void SendData(uint8_t *packet, size_t length);
        void SendDataFromString(uint8_t *dataString, size_t length);
        void DecodePowerState(POWERSTATUS_T *power_state);
        void DecodePrimaryPresence(PRESENCE_T *presence, uint8_t num);
        void DecodeSecondaryPresence(PRESENCE_T *presence);
        void DecodeSecondaryHeartbeat(S_HEARTBEAT_T *heartbeat);
        void DecodeSecondaryVin(VIN_T *vin);
        void SetCurrent(uint8_t current);
        void SetMaxCurrent(uint8_t maxCurrent);
        uint8_t ChargersConnected();
        TeslaConnector * GetConnector(uint16_t twcid);


    private:
        uint8_t CalculateChecksum(uint8_t *buffer, size_t length);
        bool VerifyChecksum(uint8_t *buffer, size_t length);
        void DecodeLinkReady();
        void DecodePrimaryHeartbeat();
        void DecodeSecondaryHeartbeat();
        void ProcessPacket(uint8_t *packet, size_t length);
        static void startupTask_(void *pvParameter);

    private:
        HardwareSerial* serial_;
        TeslaControllerIO* controller_io_;
        uint8_t num_connected_chargers_;  
        uint16_t twcid_;
        uint8_t sign_;
        uint8_t receive_buffer_[MAX_PACKET_LENGTH];
        uint8_t receive_index_;
        bool message_started_ = false;
        uint8_t available_current_;
        uint8_t max_current_;
        bool current_changed_;
        bool debug_;
        TeslaConnector* chargers[3];
};

#endif /* TWC_PROTOCOL_H */