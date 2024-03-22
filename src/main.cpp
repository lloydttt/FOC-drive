#include <Arduino.h>

#include "open_v_control.h"

void setup() {
  Serial.begin(115200);
  ov_init();
  delay(1500);
  Serial.print("open vocility initialized");


}

void loop() {
  open_v_run();

}

