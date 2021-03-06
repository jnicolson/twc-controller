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

#ifndef WIFI_H
#define WIFI_H
#include <WiFi.h>

#include "config.h"

class Wifi {
    public:
        Wifi();
        void Begin(TWCConfig &twc_config);
        void BeginConfig();
        String GetDefaultAPName();
        void Reset();
        void Scan();
        void SetCountry();
        IPAddress LocalIP();
    private:
        bool shouldSaveConfig_;
        


};

#endif /* WIFI_H */