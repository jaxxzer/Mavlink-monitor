#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include "Notify.h"
#include "Param.h"
#include "LowPassFilter.h"

#define RANGEFINDER_TIMEOUT_MS 10000


class Rangefinder {
public:

	Rangefinder();

	LowPassFilterFloat range_filt;

	void init(Parameters *_params);
	void update(void);
	uint32_t PINGRATE;
	uint32_t RANGE_ENABLE;
	  uint32_t LPF_ENABLE;
	  float LPF_CUTOFF;

	//TODO parse and store as uint16_t
	float range;

	Parameters *params;

  conn_status_t status;


private:
	void range_receive(void);
	void range_request(void);
	uint32_t last_request_ms;
  bool response_received;
  uint32_t last_response_ms;
};

#endif
