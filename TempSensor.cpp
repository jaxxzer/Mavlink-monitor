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
		params->addUint8("T_ENABLE", &T_ENABLE, 0, 1, 1);
		params->addFloat("T_SCALE", &T_SCALE, 0, 100, 10);
		params->addFloat("T_OFFSET", &T_OFFSET, -10000, 10000, -5000); // Temperature offset in centidegrees C
		params->addUint16("T_LIMIT", &T_LIMIT, 0, 10000, 5000);
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

	_last_update_ms = tnow;

	float voltage = (analogRead(PIN_TEMPSENSOR) * ADC_VOLTAGE_SCALAR);

	// 10mV/degree C
	//	The TMP36 is specified from
	//	−40°C to +125°C, provides a 750 mV output at 25°C, and
	//	operates to 125°C from a single 2.7 V supply.
	// temperature reading in centidegrees C
	temperature = (voltage * 1000) * T_SCALE + T_OFFSET;

#if TEMPSENSOR_DEBUG
	Serial.println(temperature);
#endif
}
