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

	// Sanity check parameter values
	void constrain_params(void);

	Parameters *params;
	uint32_t T_SCALE; // Scalar, mV per degrees C
	float T_OFFSET; // Voltage offset to be applied to temperature readings

	// Temperature in degrees C
	uint16_t temperature;

private:
	uint32_t _last_update_ms;
	uint32_t _update_interval_ms;
	void read();
};

#endif
