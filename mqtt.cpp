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
#include "twc_protocol.h"

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

void TeslaMqttIO::writeCharger(uint16_t twcid, uint8_t max_allowable_current) {
  char topic[50];
  snprintf(topic, 50, "twcController/twcs/%04x/max_allowable_current", twcid);

  char buffer[10];
  snprintf(buffer, 10, "%d", max_allowable_current);

  mqttClient_->publish(topic, (uint8_t *)buffer, strlen(buffer), 2, true);
}

void TeslaMqttIO::writeChargerTotalKwh(uint16_t twcid, uint32_t total_kwh) {
  char topic[50];
  snprintf(topic, 50, "twcController/twcs/%04x/total_kwh_delivered", twcid);

  char buffer[10];
  snprintf(buffer, 10, "%d", total_kwh);

  mqttClient_->publish(topic, (uint8_t *)buffer, strlen(buffer), 2, true);
}

void TeslaMqttIO::writeChargerSerial(uint16_t twcid, uint8_t* serial, size_t length) {
  char topic[50];
  snprintf(topic, 50, "twcController/twcs/%04x/serial", twcid);

  mqttClient_->publish(topic, serial, length, 2, true);
} 

void TeslaMqttIO::writeChargerVoltage(uint16_t twcid, uint16_t voltage, uint8_t phase) {
  if (phase > 3) {
    Serial.println("Phase Should be 3 or less!");
    return;
  }

  char topic[50];
  snprintf(topic, 50, "twcController/twcs/%04x/phase_%d_voltage", twcid, phase);

  char buffer[10];
  snprintf(buffer, 10, "%d", voltage);

  mqttClient_->publish(topic, (uint8_t *)buffer, strlen(buffer), 2, true);
}

void TeslaMqttIO::writeChargerCurrent(uint16_t twcid, uint8_t current, uint8_t phase) {
  if (phase > 3) {
    Serial.println("Phase should be 3 or less");
    return;
  }

  char topic[50];
  snprintf(topic, 50, "twcController/twcs/%04x/phase_%d_current", twcid, phase);

  char buffer[10];
  snprintf(buffer, 10, "%d", current);

  mqttClient_->publish(topic, (uint8_t *)buffer, strlen(buffer), 2, true);
};

void TeslaMqttIO::writeTotalConnectedChargers(uint8_t connected_chargers) {
  char buffer[10];
  snprintf(buffer, 10, "%d", connected_chargers);
  mqttClient_->publish("twcController/total/connected_chargers", (uint8_t *)buffer, strlen(buffer), 2, true);
};

void TeslaMqttIO::writeChargerFirmware(uint16_t twcid, EXT_FIRMWARE_PAYLOAD_T *firmware_payload) {
  char topic[50];
  snprintf(topic, 50, "twcController/twcs/%04x/firmware_version", twcid);
  
  char buffer[10];
  snprintf(buffer, 10, "%d.%d.%d.%d", 
    firmware_payload->major, 
    firmware_payload->minor, 
    firmware_payload->revision, 
    firmware_payload->extended
  );

  mqttClient_->publish(topic, (uint8_t *)buffer, strlen(buffer), 2, true);
};

void TeslaMqttIO::writeChargerActualCurrent(uint16_t twcid, uint16_t current) {
  char topic[50];
  snprintf(topic, 50, "twcController/twcs/%04x/actual_current", twcid);

  char buffer[10];
  snprintf(buffer, 10, "%d", current);

  mqttClient_->publish(topic, (uint8_t *)buffer, strlen(buffer), 2, true);
}

void TeslaMqttIO::writeState() {
  //mqttClient.publish("topic", 2, true, payload)

  /*
  twc/total/connected_cars
  twc/total/current_draw
  twc/total/phase_1_current
  twc/total/phase_2_current
  twc/total/phase_3_current

  twc/<twcid>/connected_vin
  twc/<twcid>/plug_state
  */
}