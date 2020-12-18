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

#include <SPIFFS.h>
#include <ArduinoJson.h>

#include "twc_controller.h"
#include "config.h"

TWCConfig::TWCConfig(char* config_file) :
  config_file_(config_file)
{
};

bool TWCConfig::CreateDefaultConfig() {
  DynamicJsonDocument doc(512);
  JsonObject wifi = doc.createNestedObject();
  wifi["ssid"] = "";
  wifi["password"] = "";

  JsonObject mqtt = doc.createNestedObject();
  mqtt["server"] = "";
  mqtt["port"] = 1883;
  mqtt["username"] = "";
  mqtt["password"] = "";

  JsonObject tesla = doc.createNestedObject();
  tesla["max_current"] = MAX_CURRENT;

  File file = SPIFFS.open(config_file_, "w");

  if (!file) {
    Serial.println(F("Big problems - can't write to SPIFFS"));
    return false;
  };

  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
    return false;
  };

  file.close();
  return true;
};

bool TWCConfig::Begin() {
  File configFile = SPIFFS.open(config_file_, "r");

  if (!configFile) {
    Serial.println("Failed to open config file.  Creating default..");
    CreateDefaultConfig();
    return false;
  };

  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, configFile);
  if (error) {
    Serial.println(F("failed to read file"));
    return false;
  };

  strlcpy(wifi.ssid, doc["wifi"]["ssid"] | "", sizeof(wifi.ssid));
  strlcpy(wifi.password, doc["wifi"]["password"] | "", sizeof(wifi.password));
  strlcpy(mqtt.server, doc["mqtt"]["server"] | "", sizeof(mqtt.server));
  mqtt.port = doc["mqtt"]["port"];

  tesla.max_current = doc["tesla"]["max_current"] | MAX_CURRENT;

  configFile.close();
  return true;
}