#include "WaterDetector.h"
#include "MapleMini.h"

WaterDetector::WaterDetector() :
detected(false),
last_detect_ms(0),
_last_update_ms(0),
_update_interval_ms(100) // 10Hz
{
	memset(detectors, 0, sizeof(detectors));
	for(int i = 0; i < NUM_WATERDETECTORS; i++) {
		detectors[i].pin = -1;
	}
}

void WaterDetector::init_params(Parameters *_params) {
	params = _params;
	if(params != NULL) {

	}
}

void WaterDetector::init() {

}

void WaterDetector::update() {
	uint32_t tnow = millis();

	if(tnow < _last_update_ms + _update_interval_ms) {
		return;
	}

	_last_update_ms = tnow;

	for(int i = 0; i < NUM_WATERDETECTORS; i++) {
		if(detectors[i].pin != -1) {
			detectors[i].detected = digitalRead(detectors[i].pin);
			if(detectors[i].detected)
				last_detect_ms = tnow;
		}
	}

	detected = (tnow < last_detect_ms + WATERDETECTOR_COOLDOWN_MS) && (last_detect_ms != 0);
}
