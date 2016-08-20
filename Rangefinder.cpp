

#include "Rangefinder.h"
#include <Arduino.h>

void Rangefinder::range_receive() {

  static char distance[20];
  static uint8_t index;

  while(Serial3.available() > 0) {
    uint8_t c = Serial3.read();
    switch(c) {
      case '0' ... '9':
      case '.':
        if(index < 19)
          distance[index++] = c;
        break;
        
      case 'm':
        distance[index] = '\0';
        range = String(distance).toFloat();
      case '\r':
      case '\n':
        index = 0;
        break;

      default:
        range = 0;
        index = 0;
        break;
    }
  }
}
