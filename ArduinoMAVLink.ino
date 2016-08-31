#include "Monitor.h"

Monitor m;

void setup() {
  m = Monitor();
}

void loop() {
  m.init();
  while(true) {
    m.run();
  }
}
