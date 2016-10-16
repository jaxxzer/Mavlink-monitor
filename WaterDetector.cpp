#include "WaterDetector.h"
#include "MapleMini.h"

#define WATERDETECTOR_DEBUG 0

WaterDetector::WaterDetector() :
detected(false),
last_detect_ms(0),
_last_update_ms(0),
_update_interval_ms(50) // 20Hz
{
	memset(detectors, 0, sizeof(detectors));
	for(int i = 0; i < NUM_WATERDETECTORS; i++) {
		detectors[i].pin = -1;
	}

	detectors[0].pin = PIN_WATERDETECTOR; // configure pin for waterdetector

	for(int i = 0; i < NUM_WATERDETECTORS; i++) {
		if(detectors[i].pin != -1) {
			pinMode(detectors[i].pin, INPUT);
		}
	}
}

void WaterDetector::init_params(Parameters *_params) {
	params = _params;
	if(params != NULL) {
		params->addUint8(param_w_enable, "W_ENABLE", &W_ENABLE, 0, 1, 1);
	}
}

void WaterDetector::init() {

}

void WaterDetector::update() {
	if(!W_ENABLE) {
		return;
	}

	uint32_t tnow = millis();

	if(tnow < _last_update_ms + _update_interval_ms) {
		return;
	}

	_last_update_ms = tnow;

	for(int i = 0; i < NUM_WATERDETECTORS; i++) {
		if(detectors[i].pin != -1) {
			detectors[i].detected = !digitalRead(detectors[i].pin);
			if(detectors[i].detected)
				last_detect_ms = tnow;
#if WATERDETECTOR_DEBUG
		Serial.print("Waterdetector #");
		Serial.print(i);
		Serial.print(" state = ");
		Serial.println(detectors[i].detected);
#endif
		}
	}

	detected = (tnow < last_detect_ms + WATERDETECTOR_COOLDOWN_MS) && (last_detect_ms != 0);
#if WATERDETECTOR_DEBUG
		Serial.print("Detected = ");
		Serial.println(detected);
#endif
}
