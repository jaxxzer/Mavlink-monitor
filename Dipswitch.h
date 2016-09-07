#ifndef DIPSWITCH_H
#define DIPSWITCH_H

#include <Arduino.h> // type definitions

#define DIPSWITCH_NUM_POLES 4

class Dipswitch {
public:
	Dipswitch();
	void init(void); // Run once at startup
	void update(void); // Run repeatedly whenever we have time or need to

	typedef struct {
		int8_t pin;
		bool state;
	} dipswitch_pole_t;


	bool get_state(uint8_t pole) { return pole<(DIPSWITCH_NUM_POLES)?state[pole].state:false; } ;
	uint8_t get_state(void); // bitmask representing state off all poles

private:
	dipswitch_pole_t state[DIPSWITCH_NUM_POLES];
	uint32_t last_update_ms;
	bool read_pole(uint8_t pole);

};
#endif
