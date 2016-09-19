#ifndef WATERDETECTOR_H
#define WATERDETECTOR_H

#include "Param.h"

#define NUM_WATERDETECTORS 1
#define WATERDETECTOR_COOLDOWN_MS 1000

class WaterDetector {
public:
	WaterDetector(void);

	void init_params(Parameters* _params);
	void init(void);
	void update(void);

	Parameters *params;

	typedef struct {
		uint8_t id;
		bool detected;
		int8_t pin;
	} water_detector_t;

	bool detected;

private:
	water_detector_t detectors[NUM_WATERDETECTORS];
	uint32_t last_detect_ms;
	uint32_t _last_update_ms;
	uint16_t _update_interval_ms;
};

#endif
