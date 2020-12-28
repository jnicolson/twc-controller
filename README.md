# TWC Controller

This repo contains a work in progress Tesla Wall Charger controller to run on an ESP32.  It is not ready for use, but is a starting point.

Credit goes to [WinterDragoness](https://teslamotorsclub.com/tmc/members/winterdragoness.40930/) from the Tesla Motor Club forums for figuring out the protocol (with many others in the forum also giving help) and [Craig 128](https://teslamotorsclub.com/tmc/members/craig-128.113283/) for the original C code I used as a reference.  Both their projects are linked in the links section below - without both of them this wouldn't have been possible.

## Building
This reqiures an ESP32 with a connected RS485 transceiver.  The pins the transceiver is connected (RX, TX and Enabled) are defined in twc_controller.h.  Building requires arduino with the following installed

* [Espressif ESP32 board support](https://github.com/espressif/arduino-esp32)
* [PangolinMQTT](https://github.com/philbowles/PangolinMQTT)
* [AsyncTCP](https://github.com/me-no-dev/AsyncTCP)
* [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
* [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
* [EasyButton](https://github.com/evert-arias/EasyButton)

To build, I find using the arduino-cli the easiest.  Build with

```
arduino-cli compile -e --fqbn espressif:esp32:esp32
```

-e makes arduino-cli put the output bin file in the 'build' directory (otherwise it will only be in the temp directory.  Replace the fully qualified board name (fqbn) with one which suits the board you're compiling for - esp32:esp32 is the generic board with no extra configuration.

## Code areas
### Config
The goal is to have a captive portal for configuration, but right now this is not complete.  Instead a json file called config.json should be written to SPIFFS before trying to start it up.  Format is:

```
{
    "wifi": {
        "ssid": "SSID",
        "password": "password"
    },
    "mqtt": {
        "server": "ip-address",
        "port": 1883
    },
    "twc": {
        "max_current": 32
    }
}
```
### Web
Web is currently started, but not operational.  A vue.js UI has been started which will have:
* Config
* Charger Status
* Charger Controls

### TWC Controller
The controller receives data from the serial port, assembled the data into a packet and then processes the packet and decides what to do with it.  It also runs an FreeRTOS task in an infinite loop which sends a heartbeat to all TWCs it has seen.

### MQTT
MQTT is the main input/output for twc-controller.  The topics it listens to are:

* twcController/availableCurrent - how much current the charger(s) can use
* twcController/debugEnabled - send a 1 to this to enable serial debug

And it writes to:
#### Totals
* twcController/total/current_draw - Actual current in use by all TWCs
* twcController/total/connected_chargers - Number of TWCs connected
* twcController/total/phase_[123]_current - Total current on each phase for all TWCs
* twcController/total/connected_cars - Number of cars connected

#### Per TWC
* max_allowable_current - Maximum current the TWC is rated for
* phase_[123]_voltage - Voltage on each phase
* phase_[123]_current - Current draw on each phase
* actual_current - Total current
* serial - Charger serial number
* total_kwh_delivered - Total kWh delivered by the TWC since manufacture
* firmware_version - Firmware running on the TWC
* connected_vin - VIN of car connected to TWC (likely only going to work for a TWC set to Tesla mode, with a Tesla plugged in)
* state - State frame from heartbeat (i.e. 0 = Ready, 1 = Charging, etc)

There are also some topics used for debug.  Topics listened to:
* twcController/debug/packetSend - send a raw packet (needs the checksum, but without the SLIP_END byte on each end)

Topics written to:
* twcController/debug/packetReceive - Full packets received by the contoller but before they are decoded by type
* twcController/debug/raw - Raw data from serial.  No processing done at all.

### OTA
OTA will be setup to work via either a web browser (uploading the firmware) or ArduinoOTA.  Right now only ArduinoOTA is working.  To flash via ArduinoOTA for the ESP32, the following command can be used (replacing \<ip address> and \<fimware bin file> with the actual values)

```
esptool.py -i "<ip address>" -p "3232" -f "<firmware bin file>"
```

## Other Projects

* [TWC](https://github.com/craigpeacock/TWC)
* [TWCManager](https://github.com/ngardiner/TWCManager)
* [TWCManager (original)](https://github.com/dracoventions/TWCManager)
* [AsyncElegantOTA](https://github.com/ayushsharma82/AsyncElegantOTA)
* [ElegantOTA](https://github.com/ayushsharma82/ElegantOTA)