#ifndef WATERDETECTOR_H
#define WATERDETECTOR_H

#include "Param.h"

#define NUM_WATERDETECTORS 3
#define WATERDETECTOR_COOLDOWN_MS 1000

class WaterDetector {
public:
	WaterDetector(void);

	void init(Parameters* _params);
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
};

#endif
