#include <WiFi.h>

#include "twc_private.h"
#include "twc_controller.h"
#include "ota.h"
#include "twc_protocol.h"

void setupWifi(char *ssid, char *key) {
    Serial.print("Setting up WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, key);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }

    Serial.println("Done!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void setup() {
    Serial.begin(115200);
    Serial.println("Booting...");

    setupWifi(SSID, PASSWORD);
    setupOTA();

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    HandleOTA();

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}
