#ifndef TEMPSENSOR_H
#define TEMPSENSOR_H
#include <Arduino.h>
#include "Param.h"

class TempSensor {
public:
	TempSensor();

	// Add parameters
	void init_params(Parameters *_params);

	// Initialize, called once, after init_params
	void init(void);
	void update(void);

	Parameters *params;
	float T_SCALE; // Scalar, mV per degrees C
	float T_OFFSET; // Voltage offset to be applied to temperature readings
	uint8_t T_ENABLE;
	uint16_t T_LIMIT;

	// Temperature in degrees C
	uint16_t temperature;

private:
	uint32_t _last_update_ms;
	uint32_t _update_interval_ms;
	void read();
};

#endif
