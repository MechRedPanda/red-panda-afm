#include <Arduino.h>
#include "ad5761.hpp"
#include "ads868x.hpp"

void setup() {
  // Initialize digital pin D2 as an output
  pinMode(2, OUTPUT);

  // Start serial communication at 115200 baud
  Serial.begin(115200);
}

// *** Main Loop *** //
void loop() {
  // Check if data is available to read
  Serial.println('1');

  // Blink the LED on pin D2
  digitalWrite(2, HIGH);
  delay(1000);
  digitalWrite(2, LOW);
  delay(1000);
}