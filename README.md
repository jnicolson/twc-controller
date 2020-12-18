# TWC Controller

This repo contains a work in progress Tesla Wall Charger controller to run on an ESP32.  It is not ready for use, but is a starting point.

## Config
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
## Web
Web is currently started, but not operational.  A vue.js UI has been started which will have:
* Config
* Charger Status
* Charger Controls

## TWC Controller
* As of 2020-12-19 there is an issue when first plugging a car in that it doesn't advertise the current properly.  If you wait too long the car thinks the charger can't deliver power and goes into an error state so you have to send an availableCurrent message quickly.  I'm sure it's just a matter of detecting the charge state changing (via the secondary heartbeat) and then giving it a current straight away based on the last one saved.

## MQTT
MQTT is the main input/output for twc-controller.  The topics it listens to are:

* twc/availableCurrent - how much current the charger(s) can use
* twc/packetSend - send a raw packet (needs the checksum, but without the SLIP_END byte on each end)
* twc/debugEnabled - send a 1 to this to enable serial debug

## Other Projects

* [TWC](https://github.com/craigpeacock/TWC)
* [TWCManager](https://github.com/ngardiner/TWCManager)
* [TWCManager (original)](https://github.com/dracoventions/TWCManager)
* [AsyncElegantOTA](https://github.com/ayushsharma82/AsyncElegantOTA)
* [ElegantOTA](https://github.com/ayushsharma82/ElegantOTA)