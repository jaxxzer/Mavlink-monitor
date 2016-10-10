#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include "Notify.h"
#include "Param.h"
#include "LowPassFilter.h"

#define RANGEFINDER_TIMEOUT_MS 2000


class Rangefinder {
public:

	Rangefinder();

	LowPassFilterFloat range_filt;

	void init_params(Parameters *_params);
	void init(void);
	void update(void);
	void constrain_params(void);
	Parameters *params;

	uint8_t PINGRATE;
	uint8_t RANGE_ENABLE;
	uint8_t LPF_ENABLE;
	float LPF_CUTOFF;

	//TODO parse and store as uint16_t
	float range;

	conn_status_t status;

private:
	void range_receive(void);
	void range_request(void);
	uint32_t last_request_ms;
	bool response_received;
	uint32_t last_response_ms;
	float last_valid_range; // to intelligently detect out of range high/low
	bool out_of_range_flag; // to reset range_filt when valid range is recovered
};

#endif
