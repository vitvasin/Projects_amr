#include <Arduino.h>

#define LED_PIN 38

void setup() {
    pinMode(LED_PIN, OUTPUT); // Set LED_PIN as output
    digitalWrite(LED_PIN, LOW); // Ensure LED is off initially
    Serial.begin(115200); // Start serial communication
}

void loop() {
    if (Serial.available() > 0) { // Check if data is available on the serial port
        char input = Serial.read(); // Read the incoming byte
        if (input == '1') {
            digitalWrite(LED_PIN, HIGH); // Turn LED on
            Serial.println("LED ON");
        } else if (input == '0') {
            digitalWrite(LED_PIN, LOW); // Turn LED off
            Serial.println("LED OFF");
        } else {
            Serial.println("Invalid input. Send '1' to turn ON or '0' to turn OFF.");
        }
    }
}