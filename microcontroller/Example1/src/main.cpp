#include <Arduino.h>

#define LED_PIN 38

void setup() {
    pinMode(LED_PIN, OUTPUT); // Set LED_PIN as output
    digitalWrite(LED_PIN, HIGH); // Ensure LED is off initially
    Serial0.begin(115200); // Start Serial0 communication
}

void loop() {
    if (Serial0.available() > 0) { // Check if data is available on the Serial0 port
        char input = Serial0.read(); // Read the incoming byte
        if (input == '1') {
            digitalWrite(LED_PIN, LOW); // Turn LED on
          //  Serial0.println("LED ON");
        } else if (input == '0') {
            digitalWrite(LED_PIN, HIGH); // Turn LED off
           // Serial0.println("LED OFF");
        } else {
          //  Serial0.println("Invalid input. Send '1' to turn ON or '0' to turn OFF.");
        }
    }
}