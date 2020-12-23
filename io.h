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

#ifndef IO_H
#define IO_H

// TODO: something about this
class TeslaControllerIO {
    public:
        virtual ~TeslaControllerIO() {};
        virtual void onChargeChangeMessage(/* a function pointer goes here */) = 0;
        virtual void onCurrentMessage(std::function<void(uint8_t)>) = 0;
        virtual void onRawMessage(std::function<void(uint8_t*, size_t)>) = 0; 
        virtual void onDebugMessage(std::function<void(bool)>) = 0;
        virtual void writeRaw(uint8_t*, size_t) = 0;
        virtual void writeRawPacket(uint8_t *data, size_t length) = 0;
        virtual void writeActualCurrent(float) = 0;
        virtual void writeState() = 0;
        virtual void stopCharging() = 0;
};

#endif /* IO_H */
