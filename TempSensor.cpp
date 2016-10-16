#include "TempSensor.h"
#include "MapleMini.h"
#define TEMPSENSOR_DEBUG 0

TempSensor::TempSensor() :
temperature(0),
_last_update_ms(0),
_update_interval_ms(100)
{
	pinMode(PIN_TEMPSENSOR, INPUT);
}

void TempSensor::init_params(Parameters *_params) {
	params = _params;
	if(params != NULL) {
		params->addUint8(param_t_enable, "T_ENABLE", &T_ENABLE, 0, 1, 1);
		params->addFloat(param_t_scale, "T_SCALE", &T_SCALE, 0, 100, 10);
		params->addFloat(param_t_offset, "T_OFFSET", &T_OFFSET, -10000, 10000, -5000); // Temperature offset in centidegrees C
		params->addUint16(param_t_limit, "T_LIMIT", &T_LIMIT, 0, 10000, 5000);
		params->addFloat(param_t_cutoff, "T_CUTOFF", &T_CUTOFF, 0.001, 100, 1);
		params->addUint8(param_t_lpf_enable, "T_LPF_ENABLE", &T_LPF_ENABLE, 0, 1, 1);
	}
}

// Called once on program startup, after parameters have been loaded
void TempSensor::init() {

}

void TempSensor::update() {
	if(!T_ENABLE) {
		temperature = 0;
		return;
	}

	uint32_t tnow = millis();

	if(tnow < _last_update_ms + _update_interval_ms) {
		return;
	}

	float voltage = (analogRead(PIN_TEMPSENSOR) * ADC_VOLTAGE_SCALAR);

	// 10mV/degree C
	//	The TMP36 is specified from
	//	−40°C to +125°C, provides a 750 mV output at 25°C, and
	//	operates to 125°C from a single 2.7 V supply.
	// temperature reading in centidegrees C
	temperature = (voltage * 1000) * T_SCALE + T_OFFSET;
	if(T_LPF_ENABLE) {
		if(temperature_filt.get_cutoff_freq() != T_CUTOFF) {
			temperature_filt.set_cutoff_frequency(T_CUTOFF);
		}
		temperature_filt.apply(temperature, (tnow - _last_update_ms)/1000.0f);
	}

	_last_update_ms = tnow;

#if TEMPSENSOR_DEBUG
	Serial.println(temperature);
#endif
}
