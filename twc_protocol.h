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

#endif /* TWC_PROTOCOL_H */