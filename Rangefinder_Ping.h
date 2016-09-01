#ifndef RANGEFINDER_PING_H
#define RANGEFINDER_PING_H

#include "Param.h"
#include <Arduino.h>
#include "MapleMini.h"

#define PING_TIMEOUT_MS 1000



class Rangefinder_Ping
{
public:
	Rangefinder_Ping();
	void init(Parameters *_params);
	void update(void);
	uint32_t PINGRATE;
	uint32_t RANGE_ENABLED;
	uint16_t range;
	Parameters *params;

private:
	//int8_t _pin;

	uint16_t micros_to_cm(uint32_t microseconds);

	void trigger(int16_t pin);
	uint32_t last_ping_ms;
	uint32_t last_response_ms;
};
#endif
