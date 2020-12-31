/*
TWC Manager for ESP32
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
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef TWC_H
#define TWC_H

#define HOSTNAME "TeslaCharger"
#define RO_PIN 19
#define DI_PIN 5
#define RE_PIN 18

#define RST_BUTTON 0

#define TWCID 0xAB32

// This is an absolute maximum - if a user enters something more than
// this then it will be set to 32.  This is to stop accidental typing
// of larger values
#define MAX_CURRENT 32
#define MIN_CURRENT 10 // Minimum current to either start charging or stop charging
#define DEFAULT_STOP_DELAY 300 // Time required under the minimum current to stop charging

#define DEFAULT_MQTT_PORT 1883

#define MAX_PACKET_LENGTH 32

#define htons(x) ( (((x)<<8)&0xFF00) | (((x)>>8)&0xFF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

#endif /* TWC_H */