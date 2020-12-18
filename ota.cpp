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

#include <Arduino.h>
#include <Update.h>

#include "twc_controller.h"
#include "ota.h"
#include "wifi.h"

OTA::OTA(AsyncWebServer *server) {
  id_ = String((uint32_t)ESP.getEfuseMac(), HEX);
  server_ = server;
};

void OTA::BeginWeb() {
  Serial.print("Starting OTA Web handlers... ");

  server_->on("/update", HTTP_GET, [&](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", "temp");
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server_->on("/update/identity", HTTP_GET, [&](AsyncWebServerRequest *request){
    if (!request->authenticate(username_.c_str(), password_.c_str())) {
			return request->requestAuthentication();
		}

      request->send(200, "application/json", "{\"id\": " + id_ + ", \"hardware\": \"ESP32\"}");
		});

  server_->on("/update", HTTP_POST, [&](AsyncWebServerRequest *request) {
      // the request handler is triggered after the upload has finished... 
      // create the response, add header, and send response
      AsyncWebServerResponse *response = request->beginResponse((Update.hasError())?500:200, "text/plain", (Update.hasError())?"FAIL":"OK");
      response->addHeader("Connection", "close");
      response->addHeader("Access-Control-Allow-Origin", "*");
      request->send(response);
      restartRequired_ = true;
  }, [&](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      //Upload handler chunks in data
      if(authRequired_){
          if(!request->authenticate(username_.c_str(), password_.c_str())){
              return request->requestAuthentication();
          }
      }

      if (!index) {
          if(!request->hasParam("MD5", true)) {
              return request->send(400, "text/plain", "MD5 parameter missing");
          }

          if(!Update.setMD5(request->getParam("MD5", true)->value().c_str())) {
              return request->send(400, "text/plain", "MD5 parameter invalid");
          }

          int cmd = (filename == "filesystem") ? U_SPIFFS : U_FLASH;
          if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd)) { // Start with max available size
            Update.printError(Serial);
            return request->send(400, "text/plain", "OTA could not begin");
          }
      }

      // Write chunked data to the free sketch space
      if(len){
          if (Update.write(data, len) != len) {
              return request->send(400, "text/plain", "OTA could not begin");
          }
      }
          
      if (final) { // if the final flag is set then this is the last frame of data
          if (!Update.end(true)) { //true to set the size to the current progress
              Update.printError(Serial);
              return request->send(400, "text/plain", "Could not end OTA");
          }
      } else {
          return;
      }
  });

  Serial.println("Done!");
}

void OTA::Begin() {
  Serial.print("Setting up OTA... ");
    // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(HOSTNAME);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      //portENTER_CRITICAL_ISR(&mux);
      //disableTimer();
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  Serial.println("Done!");
}

void OTA::Handle() {
    ArduinoOTA.handle();
}