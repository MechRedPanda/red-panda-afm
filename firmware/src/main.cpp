#include <Arduino.h>
#include "ad5761.hpp"
#include "ads868x.hpp"


ADC_ads868x ads868x(1);

void init_adda(){
  // RESET THE ADC AND DAC
  ad5761_reset();
  ads868x.reset();
}


void setup() {
  ad5761_SPI_init();
    delay(500); 
}


// *** Main Loop *** //
void loop() {
}