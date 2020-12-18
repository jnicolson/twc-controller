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

#ifndef CONFIG_H
#define CONFIG_H

#include <FS.h>

struct WifiConfig {
    char ssid[32];
    char password[32];
};

struct MqttConfig {
    char server[32];
    uint16_t port;
    char username[32];
    char password[32];
};

struct TeslaConfig {
    uint8_t max_current;
};

class TWCConfig {
    public:
        TWCConfig(char* config_file);
        bool Begin();
        
        uint8_t GetUChar(char* param_name);

        TeslaConfig tesla;
        MqttConfig mqtt;
        WifiConfig wifi;
        

    private:
        bool CreateDefaultConfig();
        char* config_file_;
};



#endif /* CONFIG_H */