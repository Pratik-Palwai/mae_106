#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "packets_vars_functions.hpp"

class TelemetryServer106 {
    WebServer server;
    const char* SSID = "pratik_envy";
    const char* PASSWORD = "...";

    String html =
    "<!DOCTYPE html>"
    "<html>"
    "<head><title>106 robot</title></head>"
    "<body>"
    "<p> timestamp: " + String(ahrs_packet_main.timestamp) + " [us]</p>"
    "<p> roll: " + String(ahrs_packet_main.roll) + " [deg]</p>"
    "<p> pitch: " + String(ahrs_packet_main.pitch) + " [deg]</p>"
    "<p> yaw: " + String(ahrs_packet_main.yaw) + " [deg]</p>"
    "<p> clicks: " + String(clicks) + "</p>"
    "<p> heading_state: " + String(heading_state) + " [us]</p>"
    "</body>"
    "</html>";

public:
    void initialize() {
        WiFi.begin(SSID, PASSWORD);
        Serial.print("Connecting to wifi ");

        while (WiFi.status() != WL_CONNECTED) {
            delay(250); 
            Serial.print(". ");
        }

        Serial.println(" finished");
        Serial.println("    IP address: " + String(WiFi.localIP()));
    }

    void update() { server.send(200, "text/html", html); }
};

TelemetryServer106 telemetry_server_main; // one instance of the telemetry server

void wifiTelemetry(void *param) {
    while(1) {
        telemetry_server_main.update();
        vTaskDelay(100);
    }
}


#endif