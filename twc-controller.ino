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

#include <EasyButton.h>
#include <SPIFFS.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>

#include "twc_controller.h"

#include "wifi.h"
#include "ota.h"
#include "mqtt.h"
#include "config.h"
#include "twc_protocol.h"
#include "web.h"

TWCConfig twc_config("/config.json");
EasyButton button(RST_BUTTON);
PangolinMQTT mqttClient;
TeslaMqttIO mqtt(mqttClient);
TeslaController controller(Serial1, mqtt);
Wifi wifi;
DNSServer dnsServer;
AsyncWebServer server(80);
OTA ota(&server);

Web web;

bool config_mode;

void IRAM_ATTR buttonIsr() {
    button.read();
}

void buttonHeld() {
    wifi.Reset();
}

// This is blocking and will restart the ESP32
// at the end
void StartConfigMode() {
    wifi.BeginConfig();
    dnsServer.start(53, "*", wifi.LocalIP());
    ota.BeginWeb();
    web.BeginPortal(server);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Booting...");
    button.begin();

    if(!SPIFFS.begin()) {
        Serial.println(F("Cannot start SPIFFS!  Starting in configure mode... "));
        StartConfigMode();
    };

    if (!twc_config.Begin()) {
        Serial.println(F("Config initialisation failed.  Starting in configure mode.. "));
        StartConfigMode();
    };

    wifi.Begin(twc_config);
    web.Begin(server);
    ota.BeginWeb();
    ota.Begin();
    mqtt.Begin(twc_config);
    controller.Begin();
    MDNS.addService("http", "tcp", 80);

    controller.SetMaxCurrent(twc_config.tesla.max_current);

    button.onPressedFor(3000, buttonHeld);
    if (button.supportsInterrupt()) {
        button.enableInterrupt(buttonIsr);
    };

    controller.Startup();

    Serial.println(F("Starting loop..."));
}



void loop() {
    web.Handle();
    ota.Handle();
    button.update();
    controller.Handle();
}
