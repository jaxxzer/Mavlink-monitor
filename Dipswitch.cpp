#include "Dipswitch.h"
#include "MapleMini.h" // board pin assignments

Dipswitch::Dipswitch() :
last_update_ms(0)
{
	memset(state, 0, sizeof(state));

	state[0].pin = PIN_DIP_0;
	state[1].pin = PIN_DIP_1;
	state[2].pin = PIN_DIP_2;
	state[3].pin = PIN_DIP_3;
}

void Dipswitch::init() {
	for(int i = 0; i < DIPSWITCH_NUM_POLES; i++) {
		pinMode(state[i].pin, INPUT_PULLUP);
		read_pole(i);
	}
}

// read current pole states
void Dipswitch::update() {
	uint32_t tnow = millis();

	// limit update rate
	if(tnow < last_update_ms + 1000/5) {
		return; // not time to update
	}

	last_update_ms = tnow;

	for(int i = 0; i < DIPSWITCH_NUM_POLES; i++) {
		read_pole(i);
	}
}

bool Dipswitch::read_pole(uint8_t pole) {
	if(pole > DIPSWITCH_NUM_POLES-1) { // invalid pole
		return false;
	}

	if(state[pole].pin < 0) { // invalid pin assignment
		return false;
	}

	// pins are pulled HIGH, when dipswitch is in ON position, pin will read LOW
	state[pole].state = !digitalRead(state[pole].pin);

	return state[pole].state;
}

uint8_t Dipswitch::get_state() {
	uint8_t mask = 0;
	for(int i = 0; i < DIPSWITCH_NUM_POLES && i < 8; i++) {
		mask |= state[i].state << i;
	}
	return mask;
}
