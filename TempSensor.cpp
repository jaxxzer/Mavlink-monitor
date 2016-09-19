#include "TempSensor.h"
#include "MapleMini.h"
TempSensor::TempSensor() :
temperature(0),
_last_update_ms(0),
_update_interval_ms(1000)
{
	pinMode(PIN_TEMPSENSOR, INPUT);
}

void TempSensor::init_params(Parameters *_params) {
	params = _params;
	if(params != NULL) {
		params->add("T_SCALE", &T_SCALE, 0, 100, 10);
		params->add("T_OFFSET", &T_OFFSET, 0, 100, 0);
	}
}

// Called once on program startup, after parameters have been loaded
void TempSensor::init() {
	//constrain_params();
}

void TempSensor::update() {
	uint32_t tnow = millis();

	if(tnow < _last_update_ms + _update_interval_ms) {
		return;
	}

	_last_update_ms = tnow;

	// temperature reading in degrees C
	float voltage = (analogRead(PIN_ADC_0) * ADC_VOLTAGE_SCALAR) + T_OFFSET;

	// 10mV per degree C
	temperature = (voltage * 1000) / T_SCALE;

#if DEBUG_OUTPUT
	Serial.println(temperature);
#endif
}

// Sanity check on parameter values, initialize params to sensible values for
// those that have never been set, or have been lost due to firmware flash
void TempSensor::constrain_params() {
	T_SCALE = constrain(T_SCALE, 1, 10000);
	T_OFFSET = constrain(T_OFFSET, 0, 10000.0f);
}
