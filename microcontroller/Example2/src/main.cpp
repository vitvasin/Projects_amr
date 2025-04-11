#include <Arduino.h>
#include <Ultrasonic.h>

#define LED_PIN 38


void setup() {
   //pinMode(LED_PIN, OUTPUT); // Set LED_PIN as output
    //digitalWrite(LED_PIN, HIGH); // Ensure LED is off initially
    Serial0.begin(115200); // Start Serial0 communication
}

void loop() {

  float range = Ultrasonic(45).MeasureInCentimeters();            
  if (range > 50) {range = 50;}
  range= range /100.0;
  Serial0.println(range); // range in meter

    // Add a small delay to avoid flooding the Serial output
  delay(100);


    
}