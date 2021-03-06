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

#ifndef OTA_H
#define OTA_H

#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>

class OTA {
public:
    OTA(AsyncWebServer *server);

    void Begin();
    void BeginWeb();
    void Handle();

private:
    String id_;
    String username_;
    String password_;
    AsyncWebServer *server_;
    bool restartRequired_ = false;
    bool authRequired_ = false;

};

#endif /* OTA_H */