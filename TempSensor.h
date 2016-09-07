#ifndef TEMPSENSOR_H
#define TEMPSENSOR_H
#include <Arduino.h>

class TempSensor {
public:
	TempSensor();
	void init(void);
	void update(void);

	uint16_t temperature;
	uint16_t offset;
	float scalar;
	uint32_t last_update_ms;

private:
	void read();
};

#endif
