#include <Arduino.h>

#include "closet_p_control.h"
#include <AS5600.h>

void setup() {
  Serial.begin(115200);
  cp_init();
  delay(1500);
  // Serial.print("open vocility initialized");


}

void loop() {
  test();

}

