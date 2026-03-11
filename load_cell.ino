#include "HX711.h"

#define DOUT 3
#define CLK 2

HX711 scale;

float calibration_factor = 22023;

void setup() {
  Serial.begin(9600);
  scale.begin(DOUT, CLK);

  scale.set_scale(calibration_factor);
  scale.tare();
}

void loop() {

  float weight = scale.get_units(20);

  if(weight < 0) weight = 0;

  Serial.print("Weight: ");
  Serial.print(weight,2);
  Serial.println(" kg");

  delay(500);
}