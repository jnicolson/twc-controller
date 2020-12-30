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

#include <esp_wifi.h>

#include "twc_controller.h"
#include "wifi.h"
#include "config.h"


Wifi::Wifi()
{};

void Wifi::Begin(TWCConfig &twc_config) {
  Serial.print(F("Starting Wifi... "));

  SetCountry();
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  WiFi.begin(twc_config.wifi.ssid, twc_config.wifi.password);

  uint16_t counter = 0;
  
  while (!WiFi.isConnected()) {
    Serial.print(".");
    delay(500);

    counter += 500;

    if (counter >= 10000) {
        Serial.println(F("Timeout expired - restarting..."));
        ESP.restart();
    }
  }

  Serial.println(F(" Started!"));

};

void Wifi::SetCountry() {
  const wifi_country_t WIFI_COUNTRY_AU = {"AU",1,13,CONFIG_ESP32_PHY_MAX_TX_POWER,WIFI_COUNTRY_POLICY_AUTO};
  esp_wifi_set_country(&WIFI_COUNTRY_AU); 
}

void Wifi::BeginConfig() {
  Serial.print(F("Starting Soft AP... "));
  WiFi.persistent(false);
  WiFi.disconnect(1,1);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(GetDefaultAPName().c_str());
  WiFi.softAPsetHostname(GetDefaultAPName().c_str());
};

IPAddress Wifi::LocalIP() {
  return WiFi.localIP();
}

void Wifi::Reset() {
    Serial.print(F("Resetting Wifi... "));
    WiFi.disconnect(1,1);
    Serial.print(F("Done!  Resetting config... "));
    delay(1000);
    Serial.println(F("Done! Restarting.."));
    ESP.restart();
}

String Wifi::GetDefaultAPName() {
  String hostString = String((uint32_t)ESP.getEfuseMac(), HEX);
  hostString.toUpperCase();
  return "TWC_" + hostString;
}