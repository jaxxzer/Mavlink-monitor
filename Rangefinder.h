#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include "Notify.h"
#include "Param.h"


class Rangefinder {
public:

	Rangefinder();

	void init(Parameters *_params);
	void update(void);
	uint32_t PINGRATE;
	uint32_t RANGE_ENABLED;

	float range;

	Parameters *params;

  conn_status_t status;


private:
	void range_receive(void);
	void range_request(void);
	uint32_t _last_range_request_ms;
};

#endif
