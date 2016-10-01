#ifndef RANGEFINDER_PING_H
#define RANGEFINDER_PING_H

#include "Param.h"
#include <Arduino.h>
#include "MapleMini.h"
#include "LowPassFilter.h"

#define PING_TIMEOUT_MS 1000



class Rangefinder_Ping
{
public:
	Rangefinder_Ping();
	void init_params(Parameters *_params);
	void init(void);
	void update(void);
	LowPassFilterFloat range_filt;
	uint32_t PINGRATE;
	uint32_t RANGE_ENABLE;
	uint32_t LPF_ENABLE;
	float LPF_CUTOFF;
	uint16_t range;
	Parameters *params;

private:
	//int8_t _pin;

	uint16_t micros_to_cm(uint32_t microseconds);
	uint16_t last_valid_range;
	bool out_of_range_flag;
	void trigger(int16_t pin);
	uint32_t last_ping_ms;
	uint32_t last_response_ms;
};
#endif
