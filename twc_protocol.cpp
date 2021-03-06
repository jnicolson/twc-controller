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
    min_current_(0),
    stopstart_delay_(0),
    num_connected_chargers_(0),
    twcid_(TWCID),
    sign_(0x77),
    debug_(false)
{
}

void TeslaController::Begin() {
    Serial.print(F("Starting Tesla Controller... "));
    pinMode(RE_PIN, OUTPUT);
    serial_->begin(9600, SERIAL_8N1, RO_PIN, DI_PIN);
    digitalWrite(RE_PIN, LOW);

    // Register callbacks for IO
    controller_io_->onChargeChangeMessage();
    controller_io_->onCurrentMessage([this](uint8_t current){ this->SetCurrent(current); });
    controller_io_->onDebugMessage([this](bool enabled){ this->Debug(enabled); });
    controller_io_->onRawMessage([this](const uint8_t* message, size_t length){ this->SendDataFromString(message, length); });

    receive_index_ = 0;

    Serial.println(F("Done!"));
}

// This method is called in arduino setup() to start the main controller loop (a FreeRTOS task)
void TeslaController::Startup() {
    Serial.print(F("Starting up Tesla Controller task as primary... "));
    xTaskCreate(this->startupTask_, "TeslaControllerTask", 2048, this, 1, NULL);
    offDelayTimer_ = xTimerCreate("OffDelayTimer", stopstart_delay_ * 1000 / portTICK_PERIOD_MS, pdFALSE, (void *)this, this->offDelayCallback_);
    onDelayTimer_ =  xTimerCreate("OnDelayTimer",  stopstart_delay_ * 1000 / portTICK_PERIOD_MS, pdFALSE, (void *)this, this->onDelayCallback_);
    
    Serial.println(F("Done!"));
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

    uint8_t commandNumber = 0;
        
    for (;;) {
        if (twc->ChargersConnected() > 0) {
            for (uint8_t i = 0; i < twc->ChargersConnected(); i++) {
                twc->SendHeartbeat(twc->chargers[i]->twcid);
                if (twc->current_changed_ == true) { twc->current_changed_ = false; };

                vTaskDelay(500+random(50,100)/portTICK_PERIOD_MS);
                
                switch (commandNumber) {
                    case 0:
                        twc->SendCommand(GET_VIN_FIRST, twc->chargers[i]->twcid);
                        break;
                    case 1:
                        twc->SendCommand(GET_VIN_MIDDLE, twc->chargers[i]->twcid);
                        break;
                    case 2:
                        twc->SendCommand(GET_VIN_LAST, twc->chargers[i]->twcid);
                        break;
                    case 3:
                        twc->SendCommand(GET_SERIAL_NUMBER, twc->chargers[i]->twcid);
                        break;
                    case 4:
                        twc->SendCommand(GET_PWR_STATE, twc->chargers[i]->twcid);
                        break;
                    case 5:
                        twc->SendCommand(GET_FIRMWARE_VER_EXT, twc->chargers[i]->twcid);
                        break;
                }
                vTaskDelay(1000+random(100,200)/portTICK_PERIOD_MS);
            }

            if (commandNumber >= 5) { 
                commandNumber = 0; 
            } else {
                commandNumber++;
            };
            
        } else {
            vTaskDelay(1000+random(100,200)/portTICK_PERIOD_MS);
        }
    }

    // This should never be reached, but just in case, make sure the task is cleaned up
    vTaskDelete(NULL);
}

// This is a static method which is the callback fired when the off delay timer hits
// the set time.  The timer contains a void pointer to the main class, which we cast
// back into the class so we can call class methods from inside the static method.
void TeslaController::offDelayCallback_(TimerHandle_t xTimer) {
    TeslaController* twc = static_cast<TeslaController*>(pvTimerGetTimerID(xTimer));
    Serial.println(F("Current below minimum for > allowable time - sending stop charging message... "));

    if (twc->ChargersConnected() > 0) {
        for (uint8_t i = 0; i < twc->ChargersConnected(); i++) {
            twc->StopCharging(twc->chargers[i]->twcid);
        }
    }
}

void TeslaController::onDelayCallback_(TimerHandle_t xTimer) {
    TeslaController* twc = static_cast<TeslaController*>(pvTimerGetTimerID(xTimer));
    Serial.println(F("Current above minimum for > allowable time - sending start charging message... "));

    if (twc->ChargersConnected() > 0) {
        for (uint8_t i = 0; i < twc->ChargersConnected(); i++) {
            twc->StartCharging(twc->chargers[i]->twcid);
        }
    }

}

// This is run via the main arduino loop() to actually receive data and do something with it
void TeslaController::Handle() {
    uint8_t receivedChar;

    while (serial_->available() > 0) {
        serial_->readBytes(&receivedChar, 1);

        if (receive_index_ > MAX_PACKET_LENGTH-1) {
            Serial.println(F("Packet length exceeded"));
            receive_index_ = 0;
            return;
        }

        switch (receivedChar) {
            case SLIP_END:
                if (message_started_) {
                    if (receive_index_ <= 2) {
                        // TODO: can this be improved?  It seems a bit arbitary to just try and
                        // detect small packets as errors.  The byte after the end frame always seems
                        // to be a fairly high number (>= 0xFC).  I'm guessing it's meant to be 0xFF but
                        // the last 1-2 bits are being dropped.  Maybe a better way would be to see if there is a
                        // byte > 0xF0 but this could be caught out by data corruption too.

                        // It's likely there was a corrupt start frame and so we're flipped
                        // and thinking that the end is the start instead.  Reset this to be the start
                        receive_index_ = 0;
                        break;
                    }
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
                    Serial.println(F("Error while receiving packet data for a packet"));
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
        Serial.println(F("Enabling Debug"));
    } else {
        Serial.println(F("Disabling Debug"));
    }
    debug_ = enabled;
}

void TeslaController::GetSerial(uint16_t secondary_twcid) {
    SendCommand(GET_SERIAL_NUMBER, secondary_twcid);
}

void TeslaController::GetFirmwareVer(uint16_t secondary_twcid) {
    SendCommand(GET_FIRMWARE_VER, secondary_twcid);
}

void TeslaController::GetPowerStatus(uint16_t secondary_twcid) {
    SendCommand(GET_FIRMWARE_VER, secondary_twcid);
};

void TeslaController::GetVin(uint16_t secondary_twcid) {
    SendCommand(GET_VIN_FIRST, secondary_twcid);
    SendCommand(GET_VIN_MIDDLE, secondary_twcid);
    SendCommand(GET_VIN_LAST, secondary_twcid);
}

void TeslaController::SetCurrent(uint8_t current) {
    if (available_current_ != current) {
        Serial.printf_P(PSTR("Received current change message, new current %d\r\n"), current);
        current_changed_ = true;
    }

    // If the available current is higher than the maximum for our charger,
    // clamp it to the maximum
    if (current <= MAX_CURRENT & current >= MIN_CURRENT) {
        available_current_ = current;
    } else if (current > MAX_CURRENT) {
        available_current_ = MAX_CURRENT;
    } else if (current < MIN_CURRENT) {
        available_current_ = MIN_CURRENT;
    }

    // Off delay timer is active, but we aren't charging.  Turn off timer
    if (!IsCharging() & xTimerIsTimerActive(offDelayTimer_) == pdTRUE) {
        xTimerStop(offDelayTimer_, 0);
    }

    // We're below our configured minimum, we are charging and the timer is not active.
    // Start the timer.
    if (current < min_current_ & IsCharging() & xTimerIsTimerActive(offDelayTimer_) != pdTRUE) {
        xTimerStart(offDelayTimer_, 0);
    } 
    
    // We're above our configured minimum current, we are charging and the timer is active.
    // Stop the timer
    if (current >= min_current_ & IsCharging() & xTimerIsTimerActive(offDelayTimer_) == pdTRUE) {
        xTimerStop(offDelayTimer_, 0);
    }

    // Current is above our configured minimum but we aren't charging and the timer isn't running.
    // Start the timer.
    if (current >= min_current_ & !IsCharging() & xTimerIsTimerActive(onDelayTimer_) != pdTRUE) {
        xTimerStart(onDelayTimer_, 0);
    }

    // Current is below our configured minimum but the timer is running.  Stop the timer.
    if (current < min_current_ & !IsCharging() & xTimerIsTimerActive(onDelayTimer_) == pdTRUE) {
        xTimerStop(onDelayTimer_, 0);
    }

    // Current is above our configured minimum, we are charging but the timer is running.  Stop the timer.
    if (current >= min_current_ & IsCharging() & xTimerIsTimerActive(onDelayTimer_) == pdTRUE) {
        xTimerStop(onDelayTimer_, 0);
    }

}

void TeslaController::SendPresence(bool presence2) {
   RESP_PACKET_T presence;
   PRESENCE_PAYLOAD_T *presence_payload = (PRESENCE_PAYLOAD_T *)&presence.payload;

   presence.command = presence2 ? htons(PRIMARY_PRESENCE2) : htons(PRIMARY_PRESENCE);
   presence.twcid = twcid_;
   presence_payload->sign =  sign_;
   presence_payload->max_allowable_current = 0x0C80; // TODO: Repalce this with something not hard coded.
   for (uint8_t i = 0; i <= 7; i++) {
       presence_payload->padding[i] = 0x00;
   }
   presence.checksum = CalculateChecksum((uint8_t*)&presence, sizeof(presence));

   SendData((uint8_t*)&presence, sizeof(presence));
}

bool TeslaController::IsCharging() {
    if (total_current_ > 0) {
        return true;
    } else {
        return false;
    }
}

void TeslaController::SendPresence2() {
    SendPresence(true);
}

// Stops the car connected to a TWC from charging. Ideally, this would be by
// using the 0xFCB2 command, but this cuts the power off seemingly without
// notifying the car, resulting in an error condition.  Instead this
// sends an IO message which is picked up by a component which uses the Tesla
// API to stop charging
void TeslaController::StopCharging(uint16_t twcid) {
    controller_io_->writeStopCharging(twcid);
}

void TeslaController::StartCharging(uint16_t twcid) {
    controller_io_->writeStartCharging(twcid);
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
        heartbeat.state = 0x09; // Limit power to the value from the next two bytes

        // current * 100 (to get it to a whole number)
        heartbeat.max_current = htons(encodedMax);
    } else {
        heartbeat.state = 0x00;
        heartbeat.max_current = 0x00;
    }

    heartbeat.plug_inserted = 0x00;

    for (uint8_t i = 0; i < 5; i++) {
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
    for (uint8_t i = 0; i < 6; i++) {
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

void TeslaController::DecodeExtFirmwareVerison(RESP_PACKET_T *firmware_ver) {
    EXT_FIRMWARE_PAYLOAD_T *firmware_payload = (EXT_FIRMWARE_PAYLOAD_T *)firmware_ver->payload;
    
    TeslaConnector *c = GetConnector(firmware_ver->twcid);

    if (memcmp(&c->firmware_version, &firmware_payload, 4) != 0) {
        memcpy(&c->firmware_version, &firmware_payload, 4);
        controller_io_->writeChargerFirmware(firmware_ver->twcid, firmware_payload);
    }

    if (debug_) {
        Serial.printf_P(PSTR("Decoded: ID: %04x, Firmware Ver: %d.%d.%d.%d\r\n"), 
            firmware_ver->twcid, 
            firmware_payload->major, 
            firmware_payload->minor, 
            firmware_payload->revision,
            firmware_payload->extended
        );
    }
}

void TeslaController::DecodeSerialNumber(EXTENDED_RESP_PACKET_T *serial) {
    SERIAL_PAYLOAD_T *serial_payload = (SERIAL_PAYLOAD_T *)serial->payload;

    TeslaConnector *c = GetConnector(serial->twcid);

    if (strcmp((const char*)c->serial_number, (const char*)serial_payload->serial) != 0) {
        strcpy((char *)&c->serial_number, (const char*)&serial_payload->serial);
        controller_io_->writeChargerSerial(serial->twcid, c->serial_number, strlen((const char*)serial_payload->serial));
    }
    

    if (debug_) {
        Serial.printf_P(PSTR("Decoded: ID: %04x, Serial Number: "), serial->twcid);
        for (uint8_t i = 0; i < strlen((const char*)serial_payload->serial); i++) {
            Serial.printf("%c", serial_payload->serial[i]);
        }
        Serial.println();
    }
}

void TeslaController::DecodePowerState(EXTENDED_RESP_PACKET_T *power_state) {
    POWERSTATUS_PAYLOAD_T *power_state_payload = (POWERSTATUS_PAYLOAD_T *)power_state->payload;

    TeslaConnector *c = GetConnector(power_state->twcid);

    uint32_t total_kwh = ntohl(power_state_payload->total_kwh);
    if (total_kwh != c->total_kwh) {
        c->total_kwh = total_kwh;
        controller_io_->writeChargerTotalKwh(power_state->twcid, total_kwh);
    };

    uint16_t voltage = ntohs(power_state_payload->phase1_voltage);
    if (voltage != c->phase1_voltage) {
        c->phase1_voltage = voltage;
        controller_io_->writeChargerVoltage(power_state->twcid, voltage, 1);
    };

    voltage = ntohs(power_state_payload->phase2_voltage);
    if (voltage != c->phase2_voltage) {
        c->phase2_voltage = voltage;
        controller_io_->writeChargerVoltage(power_state->twcid, voltage, 2);
    };

    voltage = ntohs(power_state_payload->phase3_voltage);
    if (voltage != c->phase3_voltage) {
        c->phase3_voltage = voltage;
        controller_io_->writeChargerVoltage(power_state->twcid, voltage, 3);
    };

    uint8_t current = power_state_payload->phase1_current/2;
    if (current != c->phase1_current) {
        c->phase1_current = current;
        controller_io_->writeChargerCurrent(power_state->twcid, current, 1);
        UpdateTotalPhaseCurrent(1);
    };

    current = power_state_payload->phase2_current/2;
    if (current != c->phase2_current) {
        c->phase2_current = current;
        controller_io_->writeChargerCurrent(power_state->twcid, current, 2);
        UpdateTotalPhaseCurrent(2);
    };

    current = power_state_payload->phase3_current/2;
    if (current != c->phase3_current) {
        c->phase3_current = current;
        controller_io_->writeChargerCurrent(power_state->twcid, current, 3);
        UpdateTotalPhaseCurrent(3);
    };

    if (debug_) {
        Serial.printf_P(PSTR("Decoded: ID: %04x, Power State Total kWh %d, Phase Voltages: %d, %d, %d, Phase Currents: %d, %d, %d\r\n"), 
        power_state->twcid, 
        ntohl(power_state_payload->total_kwh), 
        ntohs(power_state_payload->phase1_voltage), 
        ntohs(power_state_payload->phase2_voltage), 
        ntohs(power_state_payload->phase3_voltage),
        power_state_payload->phase1_current, 
        power_state_payload->phase2_current, 
        power_state_payload->phase3_current);
    }
}

void TeslaController::DecodePrimaryPresence(RESP_PACKET_T *presence, uint8_t num) {
    PRESENCE_PAYLOAD_T *presence_payload = (PRESENCE_PAYLOAD_T *)presence->payload;

    if (debug_) {
        Serial.printf_P(PSTR("Decoded: Primary Presence %d - ID: %02x, Sign: %02x\r\n"), 
            num,
            presence->twcid, 
            presence_payload->sign
        );
    }
}

void TeslaController::DecodePrimaryHeartbeat(P_HEARTBEAT_T *heartbeat) {
    if (debug_) {
        Serial.printf_P(PSTR("Decoded: Primary Heartbeat - ID: %02x, To %02x, State %02x, Max Current: %d, Plug Inserted: %02x\r\n"), 
            heartbeat->src_twcid, 
            heartbeat->dst_twcid,
            heartbeat->state,
            ntohs(heartbeat->max_current),
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
        Serial.printf_P(PSTR("Decoded: Secondary Heartbeat: ID: %02x, To: %02x, Status: %02x, Max Current: %d, Actual Current: %d\r\n"),
            heartbeat->src_twcid,
            heartbeat->dst_twcid,
            heartbeat->state,
            ntohs(heartbeat->max_current),
            ntohs(heartbeat->actual_current)
        );
    }

    TeslaConnector *c = GetConnector(heartbeat->src_twcid);

    // If the secondary changes it's state to 4, it's most likely because
    // it's about to start charging.  Set the current changed flag
    // so that we send the max current to the secondary again.
    if (c->state != heartbeat->state) {
        if (heartbeat->state == 4) {
            current_changed_ = true;
        }

        c->state = heartbeat->state;
        UpdateTotalConnectedCars();
        controller_io_->writeChargerState(heartbeat->src_twcid, c->state);
    }
    

    // Check whether the current the secondary is charging at has changed.  If it has
    // force an udpate of the total current being used and update the internal state
    uint8_t newCurrent = ntohs(heartbeat->actual_current)/100;
    if (newCurrent != c->GetActualCurrent()) {
        c->SetActualCurrent(newCurrent);
        UpdateTotalActualCurrent();
        controller_io_->writeChargerActualCurrent(heartbeat->src_twcid, newCurrent);
    }
}

void TeslaController::UpdateTotalConnectedCars() {
    uint8_t connected_cars = 0;

    for (uint8_t i = 0; i < num_connected_chargers_; i++) {
        if (chargers[i]->state != 0) {
            connected_cars++;
        }
    }

    controller_io_->writeTotalConnectedCars(connected_cars);
}

TeslaConnector * TeslaController::GetConnector(uint16_t twcid) {
    for (uint8_t i = 0; i < num_connected_chargers_; i++) {
        if (chargers[i]->twcid == twcid) {
            return chargers[i];
        }
    }
}

void TeslaController::DecodeSecondaryPresence(RESP_PACKET_T *presence) {
    PRESENCE_PAYLOAD_T *presence_payload = (PRESENCE_PAYLOAD_T *)presence->payload;
    bool alreadySeen = false;

    for (uint8_t i = 0; i < num_connected_chargers_; i++) {
        if (chargers[i]->twcid == presence->twcid) {
            alreadySeen = true;
        }
    }

    if (!alreadySeen) {
        Serial.printf_P(PSTR("New charger seen - adding to controller. ID: %04x, Sign: %02x, Max Allowable Current: %d\r\n"), 
            presence->twcid, 
            presence_payload->sign,
            ntohs(presence_payload->max_allowable_current)
        );

        uint8_t max_allowable_current = (uint8_t)(ntohs(presence_payload->max_allowable_current)/100);

        TeslaConnector *connector = new TeslaConnector(presence->twcid, max_allowable_current);
        chargers[num_connected_chargers_++] = connector;

        controller_io_->writeCharger(connector->twcid, connector->max_allowable_current);
        controller_io_->writeTotalConnectedChargers(num_connected_chargers_);
        
        // Write 0's to MQTT for each topic which has 0 as a valid value.  This is because
        // we compare the old and new values and by default everything is 0 so it never writes
        // anything.  This way we start at 0 and immediately update to the real value if there is
        // one, or stay at 0 (which is correct) if there isn't.
        controller_io_->writeChargerVoltage(presence->twcid, 0, 1);
        controller_io_->writeChargerVoltage(presence->twcid, 0, 2);
        controller_io_->writeChargerVoltage(presence->twcid, 0, 3);

        controller_io_->writeChargerCurrent(presence->twcid, 0, 1);
        controller_io_->writeChargerCurrent(presence->twcid, 0, 2);
        controller_io_->writeChargerCurrent(presence->twcid, 0, 3);

        controller_io_->writeChargerActualCurrent(presence->twcid, 0);

        controller_io_->writeChargerConnectedVin(presence->twcid, (uint8_t *)"0");

        controller_io_->writeChargerState(presence->twcid, 0);

        controller_io_->writeTotalConnectedCars(0);
    }
}

void TeslaController::UpdateTotalActualCurrent() {
    total_current_ = 0;
    for (uint8_t i = 0; i < num_connected_chargers_; i++) {
        total_current_ += chargers[i]->GetActualCurrent();
    }

    if (debug_) {
        Serial.printf_P(PSTR("Updating actual current to %f\r\n"), total_current_);
    }
    controller_io_->writeActualCurrent(total_current_);
}

void TeslaController::UpdateTotalPhaseCurrent(uint8_t phase) {
    uint8_t phase_current_ = 0;

    for (uint8_t i = 0; i < num_connected_chargers_; i++) {
        phase_current_ += chargers[i]->GetPhaseCurrent(phase);
    }

    controller_io_->writeChargerTotalPhaseCurrent(phase_current_, phase);
};

void TeslaController::DecodeVin(EXTENDED_RESP_PACKET_T *vin_data) {
    VIN_PAYLOAD_T *vin_payload = (VIN_PAYLOAD_T *)vin_data->payload;

    TeslaConnector *c = GetConnector(vin_data->twcid);
    uint8_t *vin = c->GetVin();

    bool changed = false;

    switch (ntohs(vin_data->command)) {
        case RESP_VIN_FIRST:
            if (debug_) { Serial.printf_P(PSTR("Decoded: ID: %04x, VIN First: "), vin_data->twcid); }
            if (memcmp(&vin[0], &vin_payload->vin, sizeof(vin_payload->vin)) != 0) {
                changed = true;
                memcpy(&vin[0], &vin_payload->vin, sizeof(vin_payload->vin));
            }
            break;
        case RESP_VIN_MIDDLE:
            if (debug_) { Serial.printf(PSTR("Decoded: ID: %04x, VIN Middle: "), vin_data->twcid); }
            if (memcmp(&vin[7], &vin_payload->vin, sizeof(vin_payload->vin)) != 0) {
                changed = true;
                memcpy(&vin[7], &vin_payload->vin, sizeof(vin_payload->vin));
            }
            break;
        case RESP_VIN_LAST:
            if (debug_) { Serial.printf(PSTR("Decoded: ID: %04x, VIN Last: "), vin_data->twcid); }
            if (memcmp(&vin[14], &vin_payload->vin, 3) != 0) {
                changed = true;   
                memcpy(&vin[14], &vin_payload->vin, 3);
            }
            break;
    }

    if (changed & (strlen((const char*)vin) == 17)) {
        controller_io_->writeChargerConnectedVin(vin_data->twcid, vin);
    } else if (changed & strlen((const char*)vin) == 0) {
        controller_io_->writeChargerConnectedVin(vin_data->twcid, (uint8_t *)"0");
    }

    if (debug_) {
        if (strlen((const char*)vin_payload->vin) == 0) {
            Serial.print(F("No Car Connected"));
        }
        for (uint8_t i = 0; i < strlen((const char*)vin_payload->vin); i++) { 
            Serial.printf("%c", vin_payload->vin[i]); 
        };
        Serial.println();
    }
    
}

// Process a fully received packet (i.e. data with C0 on each end)
void TeslaController::ProcessPacket(uint8_t *packet, size_t length) {
    if (debug_) {
        Serial.print(F("Recieved Packet: "));
        for (uint8_t i = 0; i < length; i++) {
            Serial.printf("%02x", packet[i]);
        }
        Serial.println();
        
        controller_io_->writeRawPacket(packet, length);
    }

    if (!VerifyChecksum(packet, length)) {
        Serial.print(F("Error processing packet - checksum verify failed. Full packet: "));
        for (uint8_t i = 0; i < length; i++) {
            Serial.printf("%02x", packet[i]);
        }
        Serial.println();
        return;
    }
    
    uint16_t command = ((uint16_t)packet[0]<<8) | packet[1];

	switch (command) {
        case PRIMARY_PRESENCE:
            DecodePrimaryPresence((RESP_PACKET_T *)packet, 1);
            break;
        case PRIMARY_PRESENCE2:
            DecodePrimaryPresence((RESP_PACKET_T *)packet, 2);
            break;
        case SECONDARY_PRESENCE:
            DecodeSecondaryPresence((RESP_PACKET_T *)packet);
            break;
        case SECONDARY_HEARTBEAT:
            DecodeSecondaryHeartbeat((S_HEARTBEAT_T *)packet);
            break;
        case RESP_VIN_FIRST:
        case RESP_VIN_MIDDLE:
        case RESP_VIN_LAST:
            DecodeVin((EXTENDED_RESP_PACKET_T *)packet);
            break;    
        case RESP_PWR_STATUS:
            DecodePowerState((EXTENDED_RESP_PACKET_T *)packet);
			break;
        case RESP_FIRMWARE_VER_EXT:
            DecodeExtFirmwareVerison((RESP_PACKET_T *)packet);
            break;
        case RESP_SERIAL_NUMBER:
            DecodeSerialNumber((EXTENDED_RESP_PACKET_T *)packet);
            break;
        // The next commands would normally be sent by a primary so we won't receive them
        // unless we're pretending to be a secondary (i.e. for debugging)
        case PRIMARY_HEARTBEAT:
            DecodePrimaryHeartbeat((P_HEARTBEAT_T *)packet);
            break;
        case GET_VIN_FIRST:
        case GET_VIN_MIDDLE:
        case GET_VIN_LAST:
            //DecodeGetVin((PACKET_T*)packet)
            break;   
        default:
            Serial.printf_P(PSTR("Unknown packet type received: %#02x: 0x"), command);
            for (uint8_t i = 0; i < length; i++) {
                Serial.printf("%02x", packet[i]);
            }
            Serial.println();
            break;
    }
}

void TeslaController::SendDataFromString(const uint8_t *dataString, size_t length) {
    uint8_t buffer[MAX_PACKET_LENGTH];
    uint8_t packetSize = hexCharacterStringToBytes(buffer, dataString, length);

    SendData(buffer, packetSize);
}

void TeslaController::SendData(uint8_t *packet, size_t length) {
    uint8_t outputBuffer[MAX_PACKET_LENGTH];
    uint8_t i;
    uint8_t j = 0;

    if (length > MAX_PACKET_LENGTH) {
        Serial.println(F("Error - packet larger than maximum allowable size!"));
        return;
    }

    uint16_t command = ((uint16_t)packet[0]<<8) | packet[1];
    switch (command) {
        case WRITE_ID_DATE:
        case WRITE_MODEL_NO:
            Serial.println(F("WARNING! WRITE COMMANDS ATTEMPTED!  THESE CAN PERMANENTLY BREAK YOUR TWC.  COMMANDS BLOCKED!"));
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
        Serial.print(F("Sent packet: "));
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
    Serial.printf_P(PSTR("Setting maximum current to %d\r\n"), max_current);
    // Always check to make sure we're not trying to higher than the global max
    max_current_ = max_current <= MAX_CURRENT ? max_current : MAX_CURRENT;
}

void TeslaController::SetMinCurrent(uint8_t min_current) {
    Serial.printf_P(PSTR("Setting minimum current to %d\r\n"), min_current);
    min_current_ = min_current >= MIN_CURRENT ? min_current : MIN_CURRENT;
};

void TeslaController::SetStopStartDelay(uint16_t stopstart_delay) {
    Serial.printf_P(PSTR("Setting stop/start delay to %d\r\n"), stopstart_delay);
    stopstart_delay_ = stopstart_delay;
};
