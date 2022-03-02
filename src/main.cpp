#include <Arduino.h>
#include <BlobWatchy.h>

#include "settings.h"

BlobWatchy watchy(settings);

void setup() {
  watchy.init();
}

void loop() {
  // put your main code here, to run repeatedly:
}