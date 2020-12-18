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
#define RST_BUTTON 0
// This is an absolute maximum - if a user enters something more than
// this then it will be set to 32.  This is to stop accidental typing
// of larger values
#define MAX_CURRENT 32
#define DEFAULT_CURRENT 6
#define DEFAULT_MQTT_PORT 1883

#endif /* TWC_H */