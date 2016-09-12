#include "TempSensor.h"
#include "MapleMini.h"
TempSensor::TempSensor() :
temperature(0),
last_update_ms(0)
{
	pinMode(PIN_ADC_0, INPUT);
}

void TempSensor::init() {
	temperature = analogRead(PIN_ADC_0);
}

void TempSensor::update() {
	uint32_t tnow = millis();

	if(tnow > last_update_ms + 1000) {
		last_update_ms = tnow;
		temperature = analogRead(PIN_ADC_0);
    #if DEBUG_OUTPUT
		Serial.println(temperature);
    #endif
	}
}
