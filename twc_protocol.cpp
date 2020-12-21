/*
TWC Manager for ESP32
Copyright (C) 2020 Craig Peacock
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

#include "twc_controller.h"
#include "twc_protocol.h"
#include "functions.h"


TeslaController::TeslaController(HardwareSerial& serial, TeslaControllerIO& io) : 
    serial_(&serial),
    controller_io_(&io),
    max_current_(0),
    num_connected_chargers_(0),
    twcid_(TWCID),
    sign_(0x77),
    debug_(false)
{
}

void TeslaController::Begin() {
    Serial.print("Starting Tesla Controller... ");
    pinMode(RE_PIN, OUTPUT);
    serial_->begin(9600, SERIAL_8N1, RO_PIN, DI_PIN);
    digitalWrite(RE_PIN, LOW);

    // Register callbacks for IO
    controller_io_->onChargeChangeMessage();
    controller_io_->onCurrentMessage([this](uint8_t current){ this->SetCurrent(current); });
    controller_io_->onDebugMessage([this](bool enabled){ this->Debug(enabled); });
    controller_io_->onRawMessage([this](uint8_t* message, size_t length){ this->SendDataFromString(message, length); });

    receive_index_ = 0;

    Serial.println("Done!");
}

// This method is called in arduino setup() to start the main controller loop (a FreeRTOS task)
void TeslaController::Startup() {
    Serial.print("Starting up Tesla Controller task... ");
    xTaskCreate(this->startupTask_, "TeslaControllerTask", 2048, this, 1, NULL);
    Serial.println("Done!");
}

// This is a static method which is just used as the FreeRTOS task which sends
// the presence startup messages (5 * Presence 1, 5 * Presence 2)
void TeslaController::startupTask_(void *pvParameter) {

    TeslaController* twc = static_cast<TeslaController*>(pvParameter);

    for (uint8_t i = 0; i < 5; i++) {
        twc->SendPresence();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    };

    for (uint8_t i = 0; i < 5; i++) {
        twc->SendPresence2();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    };

    for (;;) {

        if (twc->ChargersConnected() > 0) {
            for (uint8_t i = 0; i < twc->ChargersConnected(); i++) {
                twc->SendHeartbeat(twc->chargers[i]->twcid);
            }
            if (twc->current_changed_ == true) { twc->current_changed_ = false; };
        }

        vTaskDelay(1000+random(100,200)/portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

// This is run via the main arduino loop() to actually receive data and do something with it
void TeslaController::Handle() {
    uint8_t receivedChar;

    while (serial_->available() > 0) {
        serial_->readBytes(&receivedChar, 1);
        

        if (receive_index_ > MAX_PACKET_LENGTH-1) {
            Serial.println("Packet length exceeded");
            receive_index_ = 0;
            return;
        }

        switch (receivedChar) {
            case SLIP_END:
                if (message_started_) {
                    ProcessPacket(receive_buffer_, receive_index_);

                    message_started_ = false;
                    receive_index_ = 0;
                    break;
                } else {
                    message_started_ = true;
                    receive_index_ = 0;
                }

                break;
            
            case SLIP_ESC:
                // Use readBytes rather than read so that this blocks.  Previously using
                // read, it would try to read too fast before the secondary had sent the next
                // byte and this would fall through to default.  This meant bytes were not being
                // decoded and the next character was appearing in the packet rather than the
                // decoded character.  

                // Check if readBytes returned 1 character.  If it didn't, it means
                // the timeout was hit and therefore this should be discarded (dropping)
                // the whole packet
                if (serial_->readBytes(&receivedChar, 1) != 1) {
                    Serial.println("Error while receiving packet data for a packet");
                    return;
                }
                
                switch (receivedChar) {
                    case SLIP_ESC_END:
                        receive_buffer_[receive_index_++] = SLIP_END;
                        break;

                    case SLIP_ESC_ESC:
                        receive_buffer_[receive_index_++] = SLIP_ESC;
                        break;

                    default:
                        break;
                        // TODO: This should be an error
                }
                break;

            default:
                if (message_started_) {
                    receive_buffer_[receive_index_++] = receivedChar;
                } // else disgard - probably corruption or started receiving
                // in the middle of a message

        }
    }
}

void TeslaController::Debug(bool enabled) {
    if (enabled) {
        Serial.println("Enabling Debug");
    } else {
        Serial.println("Disabling Debug");
    }
    debug_ = enabled;
}

void TeslaController::GetSerial() {
    SendCommand((uint16_t)GET_SERIAL_NUMBER, (uint16_t)0x0000);
}

void TeslaController::GetModelNo() {
    SendCommand((uint16_t)GET_MODEL_NUMBER, (uint16_t)0x0000);
}

void TeslaController::GetFirmwareVer() {
    SendCommand((uint16_t)GET_FIRMWARE_VER, (uint16_t)0x0000);
}

void TeslaController::GetVin(uint16_t secondary_twcid) {
    SendCommand(GET_VIN_FIRST, secondary_twcid);
    SendCommand(GET_VIN_MIDDLE, secondary_twcid);
    SendCommand(GET_VIN_LAST, secondary_twcid);
}

void TeslaController::SetCurrent(uint8_t current) {
    if (available_current_ != current) {
        Serial.printf("Received current change message, new current %d\r\n", current);
        current_changed_ = true;
    }

    // If the available current is higher than the maximum for our charger,
    // clamp it to the maximum
    available_current_ = current <= max_current_ ? current : max_current_;
}

void TeslaController::SendPresence(bool presence2) {
   PRESENCE_T presence;
   presence.command = presence2 ? htons(PRIMARY_PRESENCE2) : htons(PRIMARY_PRESENCE);
   presence.twcid = twcid_;
   presence.sign =  sign_;
   presence.max_charge_rate = 0x0C80; // TODO: Repalce this with something not hard coded.
   for (uint8_t i = 0; i <= 7; i++) {
       presence.padding[i] = 0x00;
   }
   presence.checksum = CalculateChecksum((uint8_t*)&presence, sizeof(presence));

   SendData((uint8_t*)&presence, sizeof(presence));
}

void TeslaController::SendPresence2() {
    SendPresence(true);
}

uint8_t TeslaController::ChargersConnected() {
    return num_connected_chargers_;
}

void TeslaController::SendHeartbeat(uint16_t secondary_twcid) {
    P_HEARTBEAT_T heartbeat;
    heartbeat.command = htons(PRIMARY_HEARTBEAT);
    heartbeat.src_twcid = twcid_;
    heartbeat.dst_twcid = secondary_twcid;

    if (current_changed_) {
        uint16_t encodedMax = available_current_ * 100;
        heartbeat.payload = 0x09; // Limit power to the value from the next two bytes

        // current * 100 (to get it to a whole number)
        heartbeat.max_current = htons(encodedMax);
    } else {
        heartbeat.payload = 0x00;
        heartbeat.max_current = 0x00;
    }

    heartbeat.plug_inserted = 0x00;

    for (uint8_t i = 0; i < 3; i++) {
       heartbeat.padding[i] = 0x00;
    }    
        
    heartbeat.checksum = CalculateChecksum((uint8_t*)&heartbeat, sizeof(heartbeat));

    SendData((uint8_t*)&heartbeat, sizeof(heartbeat));
}

void TeslaController::SendCommand(uint16_t command, uint16_t send_to) {
    PACKET_T packet;
    packet.command = htons(command);
    packet.twcid = twcid_;
    packet.secondary_twcid = send_to;
    for (uint8_t i = 0; i <= 7; i++) {
        packet.payload[i] = 0x00;
    }
    packet.checksum = CalculateChecksum((uint8_t*)&packet, sizeof(packet));
    SendData((uint8_t*)&packet, sizeof(packet));
}

uint8_t TeslaController::CalculateChecksum(uint8_t *buffer, size_t length) {
    uint8_t i;
    uint8_t endByte = 0;
    uint8_t checksum = 0;

    for (i = 1; i < length; i++) {
        checksum = checksum + buffer[i];
    }

    return checksum;
}

bool TeslaController::VerifyChecksum(uint8_t *buffer, size_t length) {
    uint8_t checksum = CalculateChecksum(buffer, length-1);
    if (buffer[length-1] == checksum) {
        return true;
    }

    return false;
}

void TeslaController::DecodePowerState(POWERSTATUS_T *power_state) {
    if (debug_) {
        Serial.printf("Decoded: Power State ID: %02x, Total kWh %d, Phase Voltages: %d, %d, %d, Phase Currents: %d, %d, %d\r\n", 
        power_state->twcid, 
        htonl(power_state->total_kwh), 
        htons(power_state->phase1_voltage), 
        htons(power_state->phase2_voltage), 
        htons(power_state->phase3_voltage),
        htons(power_state->phase1_current), 
        htons(power_state->phase2_current), 
        htons(power_state->phase3_current));
    }
}

void TeslaController::DecodePrimaryPresence(PRESENCE_T *presence, uint8_t num) {
    if (debug_) {
        Serial.printf("Decoded: Primary Presence %d - ID: %02x, Sign: %02x", 
            num,
            presence->twcid, 
            presence->sign
        );
    }
}

void TeslaController::DecodePrimaryHeartbeat(P_HEARTBEAT_T *heartbeat) {
    if (debug_) {
        Serial.printf("Decoded: Primary Heartbeat - ID: %02x, To %02x, Payload %02x, Max Current: %d, Plug Inserted: %02x\r\n", 
            heartbeat->src_twcid, 
            heartbeat->dst_twcid,
            heartbeat->payload,
            htons(heartbeat->max_current),
            heartbeat->plug_inserted
        );
    }

    /*S_HEARTBEAT_T reply;
    reply.command = SECONDARY_HEARTBEAT;
    reply.src_twcid = twcid_
    reply.dst_twcid = heartbeat->src_twcid;
    reply.status = 
    reply.max_current = 
    reply.actual_current = 
    for (uint8_t i = 0; i < 4; i++) {
        reply.padding[i] = 0x00;
    }

    reply.checksum = CalculateChecksum(reply);

    SendData((uint8_t*)&reply, sizeof(reply));*/
}

void TeslaController::DecodeSecondaryHeartbeat(S_HEARTBEAT_T *heartbeat) {
    if (debug_) {
        Serial.printf("Decoded: Secondary Heartbeat: ID: %02x, To: %02x, Status: %02x, Max Current: %d, Actual Current: %d\r\n",
            heartbeat->src_twcid,
            heartbeat->dst_twcid,
            heartbeat->status,
            htons(heartbeat->max_current),
            htons(heartbeat->actual_current)
        );
    }

    TeslaConnector *c = GetConnector(heartbeat->src_twcid);
    c->SetActualCurrent(htons(heartbeat->actual_current));

}

TeslaConnector * TeslaController::GetConnector(uint16_t twcid) {
    for (uint8_t i = 0; i < num_connected_chargers_; i++) {
        if (chargers[i]->twcid == twcid) {
            return chargers[i];
        }
    }
}

void TeslaController::DecodeSecondaryPresence(PRESENCE_T *presence) {
    bool alreadySeen = false;

    for (uint8_t i = 0; i < num_connected_chargers_; i++) {
        if (chargers[i]->twcid == presence->twcid) {
            alreadySeen = true;
        }
    }

    if (!alreadySeen) {
        Serial.printf("New charger seen - adding to controller. ID: %02x\r\n", presence->twcid);
        TeslaConnector *connector = new TeslaConnector(presence->twcid, presence->max_charge_rate);
        chargers[num_connected_chargers_++] = connector;
    }
}

void TeslaController::DecodeSecondaryVin(VIN_T *vinData) {
    TeslaConnector *t;

    for (uint8_t i = 0; i < num_connected_chargers_; i++) {
        if (chargers[i]->twcid == vinData->twcid) {
            t = chargers[i];
        }
    }

    uint8_t *vin = t->GetVin();

    switch (vinData->command) {
        case SECONDARY_VIN_FIRST:
            memcpy(&vin[0], &vinData->vin, sizeof(vinData->vin));
            break;
        case SECONDARY_VIN_MIDDLE:
            memcpy(&vin[7], &vinData->vin, sizeof(vinData->vin));
            break;
        case SECONDARY_VIN_LAST:
            memcpy(&vin[14], &vinData->vin, 3);
            break;
    }
}

// Process a fully received packet (i.e. data with C0 on each end)
void TeslaController::ProcessPacket(uint8_t *packet, size_t length) {
    if (debug_) {
        Serial.printf("Recieved Packet: ");
        for (uint8_t i = 0; i < length; i++) {
            Serial.printf("%02x", packet[i]);
        }
        Serial.println();
    }

    if (!VerifyChecksum(packet, length)) {
        Serial.println("Error processing packet - checksum verify failed");
        return;
    }
    
    uint16_t command = ((uint16_t)packet[0]<<8) | packet[1];

	switch (command) {
        case PRIMARY_PRESENCE:
            DecodePrimaryPresence((struct PRESENCE_T *)packet, 1);
            break;
        case PRIMARY_PRESENCE2:
            DecodePrimaryPresence((struct PRESENCE_T *)packet, 2);
            break;
        case PRIMARY_HEARTBEAT:
            DecodePrimaryHeartbeat((struct P_HEARTBEAT_T *)packet);
            break;
        case SECONDARY_PRESENCE:
            DecodeSecondaryPresence((struct PRESENCE_T *)packet);
            break;
        case SECONDARY_HEARTBEAT:
            DecodeSecondaryHeartbeat((struct S_HEARTBEAT_T *)packet);
            break;
        case SECONDARY_VIN_FIRST:
        case SECONDARY_VIN_MIDDLE:
        case SECONDARY_VIN_LAST:
            DecodeSecondaryVin((struct VIN_T*)packet);
            break;    
        case PWR_STATUS:
            DecodePowerState((struct POWERSTATUS_T *)packet);
			break;       
        default:
            Serial.printf("Unknown packet type received: %#02x: 0x", command);
            for (uint8_t i = 0; i < length; i++) {
                Serial.printf("%02x", packet[i]);
            }
            Serial.println();

            break;
    }
}

void TeslaController::SendDataFromString(uint8_t *dataString, size_t length) {
    uint8_t buffer[MAX_PACKET_LENGTH];
    uint8_t packetSize = hexCharacterStringToBytes(buffer, dataString, length);

    SendData(buffer, packetSize);
}

void TeslaController::SendData(uint8_t *packet, size_t length) {
    uint8_t outputBuffer[MAX_PACKET_LENGTH];
    uint8_t i;
    uint8_t j = 0;

    if (length > MAX_PACKET_LENGTH) {
        Serial.println("Error - packet larger than maximum allowable size!");
        return;
    }

    uint16_t command = ((uint16_t)packet[0]<<8) | packet[1];
    switch (command) {
        case WRITE_ID_DATE:
        case WRITE_MODEL_NO:
            Serial.println("WARNING! WRITE COMMANDS ATTEMPTED!  THESE CAN PERMANENTLY BREAK YOUR TWC.  COMMANDS BLOCKED!");
            return;
    }

    // Could probably get rid of the buffer and write directly to the serial port
    // but this way lets the value of the buffer be printed for debugging more easily
    outputBuffer[j++] = SLIP_END;
    for (i = 0; i < length; i++) {
        switch (packet[i]) {
            case SLIP_END:
                outputBuffer[j++] = SLIP_ESC;
                outputBuffer[j++] = SLIP_ESC_END;
                break;
            case SLIP_ESC:
                outputBuffer[j++] = SLIP_ESC;
                outputBuffer[j++] = SLIP_ESC_ESC;
                break;
            default:
                outputBuffer[j++] = packet[i];
        }
    }
    outputBuffer[j++] = SLIP_END;
    outputBuffer[j++] = 0xFF;

    if (debug_) {
        Serial.print("Sent packet: ");
        for (uint8_t i = 0; i < j; i++) {
            Serial.printf("%02x", outputBuffer[i]);
        }
        Serial.println();
    }

    digitalWrite(RE_PIN, HIGH);
    serial_->write(outputBuffer, j);
    serial_->flush(); // Make sure the serial data has finished sending before putting the RS485 transceiver back into receive mode
    digitalWrite(RE_PIN, LOW);
}

void TeslaController::SetMaxCurrent(uint8_t max_current) {
    Serial.printf("Setting maximum current to %d\r\n", max_current);
    // Always check to make sure we're not trying to higher than the global max
    max_current_ = max_current <= MAX_CURRENT ? max_current : MAX_CURRENT;
}


