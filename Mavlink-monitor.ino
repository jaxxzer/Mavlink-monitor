#include "Monitor.h"
Monitor monitor;

void setup() {
  monitor = Monitor();
}

void loop() {
  monitor.init();
  while(true) {
    monitor.run();
  }
}
