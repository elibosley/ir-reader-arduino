/*
 * ReceiveDump.cpp
 *
 * Dumps the received signal in different flavors.
 * Since the printing takes so much time, repeat signals may be skipped or interpreted as UNKNOWN.
 *
 *  This file is part of Arduino-IRremote https://github.com/Arduino-IRremote/Arduino-IRremote.
 *
 ************************************************************************************
 * MIT License
 *
 * Copyright (c) 2020-2021 Armin Joachimsmeyer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ************************************************************************************
 */
#include <Arduino.h>
/*
 * Define macros for input and output pin etc.
 */
#include <ESP8266WiFi.h> // Include the Wi-Fi library
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
// MQTT
#include <PubSubClient.h>

WiFiClient wifiClient;
PubSubClient mqttClient;

/** IR READER */
/*
 * You can change this value accordingly to the receiver module you use.
 * The required value can be derived from the timings printed here.
 * Keep in mind that the timings may change with the distance
 * between sender and receiver as well as with the ambient light intensity.
 */
#define MARK_EXCESS_MICROS 20 // recommended for the cheap VS1838 modules
const uint16_t kCaptureBufferSize = 1024;
const uint16_t kRecvPin = 5;
const uint8_t kTimeout = 15;

IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
decode_results results; // Somewhere to store the results

bool hasFired = false;
unsigned int runCounter = 0;

// This function connects to the MQTT broker
void reconnect()
{
    // Set our MQTT broker address and port
    mqttClient.setServer("192.168.137.1", 1883);
    mqttClient.setClient(wifiClient);

    // Loop until we're reconnected
    while (!mqttClient.connected())
    {
        // Attempt to connect
        Serial.println("Attempt to connect to MQTT broker");
        mqttClient.connect("IR_DETECTOR_OA123");

        // Wait some time to space out connection requests
        delay(3000);
    }

    Serial.println("MQTT connected");
}

void connectWifi()
{

    /* NETWORK SETUP */
    WiFi.mode(WIFI_STA);
    WiFi.begin("toothfairy_2G", "Kiwiisagoodcat1"); // Connect to the network
    Serial.print("Connecting to ");
    Serial.println(" ...");

    int i = 0;
    while (WiFi.status() != WL_CONNECTED)
    { // Wait for the Wi-Fi to connect
        delay(1000);
        Serial.print(++i);
        Serial.print(' ');
    }

    Serial.println('\n');
    Serial.println("Connection established!");
    Serial.print("IP address:\t");
    Serial.println(WiFi.localIP()); // Send the IP address of the ESP8266 to the computer
}

void setup()
{
    pinMode(1, OUTPUT);
    digitalWrite(1, HIGH);
    Serial.begin(9600); // Status message will be sent to PC at 9600 baud
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    Serial.println("Enabling Wifi");
    // Use turn on the save buffer feature for more complete capture coverage.
    connectWifi();

    irrecv.enableIRIn(); // Start the receiver
}

//+=============================================================================
// The repeating section of the code
//
void loop()
{
    if (runCounter > 1000)
    {
        runCounter = 0;
        hasFired = false;
    }

    runCounter += 1;
    Serial.print("runCounter: ");
    Serial.print(runCounter);
    Serial.print(" hasFIred? ");
    Serial.println(hasFired);
    if (!mqttClient.connected())
    {
        reconnect();
    }
    if (irrecv.decode(&results))
    {
        // Probs the wand
        if (results.bits > 50 && !hasFired)
        {
            hasFired = true;
            runCounter = 0;
            mqttClient.publish("cmnd/tasmota_70B3A1/POWER", "OFF");
            mqttClient.publish("cmnd/tasmota_6F6D92/POWER", "OFF");
            delay(500);
            mqttClient.publish("cmnd/tasmota_70B3A1/POWER", "ON");
            mqttClient.publish("cmnd/tasmota_6F6D92/POWER", "ON");
            delay(750);
            mqttClient.publish("cmnd/tasmota_70B3A1/POWER", "OFF");
            mqttClient.publish("cmnd/tasmota_6F6D92/POWER", "OFF");
            mqttClient.publish("cmnd/FIREPLACE/POWER", "OFF");
            mqttClient.publish("cmnd/CRASH_EFFECT/POWER", "ON");
            delay(500);
            mqttClient.publish("cmnd/tasmota_70B3A1/POWER", "ON");
            mqttClient.publish("cmnd/tasmota_6F6D92/POWER", "ON");
            delay(1000);
            mqttClient.publish("cmnd/tasmota_70B3A1/POWER", "OFF");
            mqttClient.publish("cmnd/tasmota_6F6D92/POWER", "OFF");
        }
        Serial.println(results.bits);
        serialPrintUint64(results.value, HEX);
        Serial.println("");
        irrecv.resume(); // Receive the next value
    }

    mqttClient.loop();
}
