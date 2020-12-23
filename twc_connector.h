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

#ifndef TWC_CONNECTOR_H
#define TWC_CONNECTOR_H

#include <Arduino.h>

// Encapsulate some information specific to a single wall connector.  This is to allow multiple connectors to be added
class TeslaConnector {
    public:
        TeslaConnector(uint16_t twcid, uint8_t max_charge_rate);
        void SetVin(uint8_t* upperVin);
        uint8_t* GetVin();
        void SetActualCurrent(uint16_t current);
        uint16_t twcid;
        uint8_t state;

    private:
        uint8_t max_charge_rate_;
        uint8_t vin_[18];
        uint16_t actual_current_;
};

#endif /* TWC_CONNECTOR_H */