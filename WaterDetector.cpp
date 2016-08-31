#include "WaterDetector.h"

WaterDetector::WaterDetector() :
detected(false),
last_detect_ms(0)
{
	memset(detectors, 0, sizeof(detectors));
}


void WaterDetector::init(Parameters *_params) {
	params = _params;
}

void WaterDetector::update() {
	uint32_t tnow = millis();

	for(int i = 0; i < NUM_WATERDETECTORS; i++) {
		if(detectors[i].pin != -1) {
			detectors[i].detected = digitalRead(detectors[i].pin);
			if(detectors[i].detected)
				last_detect_ms = tnow;
		}
	}

	detected = (tnow < last_detect_ms + WATERDETECTOR_COOLDOWN_MS) && (last_detect_ms != 0);
}
