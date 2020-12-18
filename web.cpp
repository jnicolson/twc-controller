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

#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>


#include "web.h"


Web::Web() {

};

void Web::Handle() {

};

void Web::Begin(AsyncWebServer& server) {
    Serial.print("Starting Web Server... ");

    server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", String(ESP.getFreeHeap()));
    });

    server.serveStatic("/", SPIFFS, "/www/").setDefaultFile("index.html");

    server.on("/404", HTTP_GET, [](AsyncWebServerRequest *request) {
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/404.html", "text/html", false);
      response->setCode(404);
      request->send(response);
    });

    server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request) {

        WiFi.scanNetworks();

        int n = WiFi.scanComplete();
        if (n == -2) {
            WiFi.scanNetworks(true);
        } else if(n) {
            DynamicJsonDocument doc(n * (JSON_OBJECT_SIZE(5) + 250));
            for (int i = 0; i < n; ++i) {
                JsonObject obj = doc.createNestedObject();
                obj["rssi"] = String(WiFi.RSSI(i));
                obj["ssid"] = WiFi.SSID(i);
                obj["bssid"] = WiFi.BSSIDstr(i);
                obj["channel"] = String(WiFi.channel(i));
                obj["secure"] = String(WiFi.encryptionType(i));
            };
            WiFi.scanDelete();

            AsyncResponseStream *response = request->beginResponseStream("application/json");
            response->addHeader("Access-Control-Allow-Origin","*");
            serializeJson(doc, *response);
            request->send(response);
        };
    });

    server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest* request) {
        Serial.println(F("GET /api/config"));
        request->send(SPIFFS, "/config.json", "application/json");
    });

    AsyncCallbackJsonWebHandler* configPostHandler = new AsyncCallbackJsonWebHandler("/api/config", [](AsyncWebServerRequest *request, JsonVariant &json) {
            Serial.println(F("POST /api/config"));
            JsonObject jsonObj = json.as<JsonObject>();
    });

    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/www/index.html", "text/html");
    });

    server.begin();

    Serial.println("Done!");
};

void Web::BeginPortal(AsyncWebServer& server) {
    server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest* request) {
        Serial.println(F("GET /api/config"));
        request->send(SPIFFS, "/config.json", "application/json");
    });

    AsyncCallbackJsonWebHandler* configPostHandler = new AsyncCallbackJsonWebHandler("/api/config", [](AsyncWebServerRequest *request, JsonVariant &json) {
            Serial.println(F("POST /api/config"));
            JsonObject jsonObj = json.as<JsonObject>();

            
    });

    
};