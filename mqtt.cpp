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

#include "mqtt.h"
#include "config.h"

TeslaMqttIO::TeslaMqttIO(PangolinMQTT &mqttClient) : 
  mqttClient_(&mqttClient)
{};

void TeslaMqttIO::Begin(TWCConfig &twc_config) {
  Serial.print("Connecting to MQTT... ");

  // TODO: Fix this
  mqttClient_->setServer(twc_config.mqtt.server, twc_config.mqtt.port);

  mqttClient_->onConnect([this](bool sessionPresent){ this->onMqttConnect(sessionPresent); });
  mqttClient_->onMessage([this](const char* topic, const uint8_t* payload, size_t len, uint8_t qos, bool retain, bool dup) { this->onMqttMessage(topic, payload, len, qos, retain, dup); });
  mqttClient_->onDisconnect([this](int8_t reason){ this->onMqttDisconnect(reason); });
  mqttClient_->connect();
  Serial.println("Done!");
}

void TeslaMqttIO::onMqttMessage(const char* topic, const uint8_t* payload, size_t len, uint8_t qos, bool retain, bool dup) {
  if (std::string(topic) == "twcController/debug/packetSend") {
    if (onRawMessageCallback_) onRawMessageCallback_(payload, len);
  } else if (std::string(topic) == "twcController/availableCurrent") {
      uint8_t returnCurrent;
      char *endPtr;
      errno = 0;

      long result = strtol((const char*)payload, &endPtr, 10);

      if ((uint8_t*)endPtr == payload) {
        Serial.printf("Error converting MQTT current to number!\r\n");
        returnCurrent = 0;
      } 

      else if (errno != 0 && result == 0) {
        Serial.printf("An unspecified error occured converting MQTT current\r\n");
      }
      
      else if (errno == 0) {
        if (result > UCHAR_MAX || result < CHAR_MIN) {
          Serial.printf("Number %d out of range (range is 0-255)\r\n", result);
          returnCurrent = 0;
        } else {
          returnCurrent = (uint8_t)result;
        }
      } else {
        Serial.println("An error occured converting the MQTT current");
      }
      if(onCurrentMessageCallback_) onCurrentMessageCallback_(returnCurrent);

  } else if (std::string(topic) == "twcController/mode") {
    if (std::string((const char*)payload) == "charge") {
      // Just charge at the 
    } else if (std::string((const char*)payload) == "stop") {
      // stop charging
    } else if (std::string((const char*)payload) == "follow") {
      // follow the avaialble current
    }
  }
  else if (std::string(topic) == "twcController/debugEnabled")  {
    if ((char)payload[0] == '1') {
      if(onDebugMessageCallback_) onDebugMessageCallback_(true);
    } else {
      if(onDebugMessageCallback_) onDebugMessageCallback_(false);
    }
  }
  
  else {
    Serial.println("Unknown MQTT Message Received");
  }
}

void TeslaMqttIO::onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT");
  mqttClient_->subscribe("twcController/debugEnabled", 2);
  mqttClient_->subscribe("twcController/availableCurrent", 2);
  mqttClient_->subscribe("twcController/mode", 2);
  mqttClient_->subscribe("twcController/debug/packetSend", 2);
}

void TeslaMqttIO::onMqttDisconnect(int8_t reason) {
  Serial.println("Disconnected from MQTT.  Attempting to reconnect...");
  mqttClient_->connect();
}

void TeslaMqttIO::onRawMessage(std::function<void(const uint8_t*, size_t)> callback) {
  onRawMessageCallback_ = callback;
}

void TeslaMqttIO::onChargeChangeMessage() {

}

void TeslaMqttIO::stopCharging() {

}

void TeslaMqttIO::onCurrentMessage(std::function<void(uint8_t)> callback) {
  onCurrentMessageCallback_ = callback;
}

void TeslaMqttIO::onDebugMessage(std::function<void(bool)> callback) {
  onDebugMessageCallback_ = callback;
}

void TeslaMqttIO::writeRaw(uint8_t *data, size_t length) {
  char buffer[length * 2];
  char *target = buffer;
  for (uint8_t i = 0; i < length; i++) {
    target += sprintf(target, "%02x", data[i]);
  }
  mqttClient_->publish("twcController/debug/raw", (uint8_t *)buffer, strlen(buffer), 2, false);
}

void TeslaMqttIO::writeRawPacket(uint8_t *data, size_t length) {
  char buffer[length * 2];
  char *target = buffer;
  for (uint8_t i = 0; i < length; i++) {
    target += sprintf(target, "%02x", data[i]);
  }
  mqttClient_->publish("twcController/debug/packetReceive", (uint8_t *)buffer, strlen(buffer), 2, false);
}

void TeslaMqttIO::writeActualCurrent(float actualCurrent) {
  char buffer[10];
  snprintf(buffer, 10, "%f", actualCurrent);
  mqttClient_->publish("twcController/totalActualCurrent", (uint8_t *)buffer, strlen(buffer), 2, true);
}

void TeslaMqttIO::writeState() {
  //mqttClient.publish("topic", 2, true, payload)

  /*

  twc/total/connected_chargers
  twc/total/connected_cars
  twc/total/current_draw
  twc/total/phase_1_current
  twc/total/phase_2_current
  twc/total/phase_3_current

  twc/<twcid>/serial
  twc/<twcid>/model
  twc/<twcid>/firmware
  twc/<twcid>/connected_vin
  twc/<twcid>/plug_state
  twc/<twcid>/total_kwh_delivered
  twc/<twcid>/phase_1_voltage
  twc/<twcid>/pahse_2_voltage
  twc/<twcid>/phase_3_voltage
  twc/<twcid>/phase_1_current
  twc/<twcid>/pahse_2_current
  twc/<twcid>/phase_3_current
  twc/<twcid>/actual_current

  */
}