#include <Arduino.h>
#include "Rangefinder.h"


Rangefinder::Rangefinder() :
range(0),
PINGRATE(0),
RANGE_ENABLED(0),
params(NULL)
{}

void Rangefinder::init(Parameters *_params) {
	params = _params;
	if(params != NULL) {
		params->add("PINGRATE", &PINGRATE);
		params->add("RANGE_ENABLE", &RANGE_ENABLED);
	}

}

void Rangefinder::update() {
	range_request();
	range_receive();
}

void Rangefinder::range_request() {
	if(PINGRATE == 0 || !RANGE_ENABLED)
		return;

	uint32_t tnow = millis();
	if(tnow > _last_range_request_ms + (1000.0f / PINGRATE)) {
		_last_range_request_ms = tnow;
		Serial3.write('Z');
	}
}

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

