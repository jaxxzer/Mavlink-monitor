#include "Notify.h"
#include "MapleMini.h"

Notify::Notify() :
params(NULL),
_last_tick_ms(0)
//flags()
{
	memset(leds, 0, sizeof(leds));

	leds[LED_MAPLE].pin = PIN_LED_MAPLE;
	leds[LED_1].pin = PIN_LED_1;
	leds[LED_2].pin = PIN_LED_2;
	leds[LED_3].pin = PIN_LED_3;

	leds[LED_MAPLE].default_on = true;
	leds[LED_1].default_on = true;
	leds[LED_2].default_on = true;
	leds[LED_3].default_on = true;
}

void Notify::init(Parameters *_params) {
	params = _params;

	// set gpio pinmode to output
	for(int i = 0; i < NOTIFY_NUM_LEDS; i++) {
		if(leds[i].pin >= 0) {
			pinMode(leds[i].pin, OUTPUT);
		}
	}

	// test leds
	for(int i = 0; i < NOTIFY_NUM_LEDS; i++) {
		if(leds[i].pin >= 0) {
			set(i, true); // turn led on
			delay(250);
		}
	}

	// initialize all leds off
	for(int i = 0; i < NOTIFY_NUM_LEDS; i++) {
		if(leds[i].pin >= 0) {
			set(i, false); // init all LEDS off
		}
	}
 delay(500);


}

void Notify::update() {
	uint32_t tnow = millis();


	if(tnow < _last_tick_ms + NOTIFY_UPDATE_MS) {
		return;
	}

	_last_tick_ms = tnow;

	for(int i = 0; i < NOTIFY_NUM_LEDS; i++) {
		if(leds[i].pin == -1) {
			continue;
		}

		if(leds[i].playing) {

			if(leds[i].delay > 0) {
				leds[i].delay--;
			} else {

				if(leds[i].c >= leds[i].pattern[leds[i].p]) {

					// toggle the led
					leds[i].state = !leds[i].state;
					digitalWrite(leds[i].pin, leds[i].state);

					leds[i].c = 0; // reset tick counter to 0
					leds[i].p++; // increment pattern pointer

					// check if we have reached the end of the pattern
					if(leds[i].p > NOTIFY_PATTERN_MAX - 1 || leds[i].pattern[leds[i].p] == 0) {
						leds[i].state = true;
						digitalWrite(leds[i].pin, leds[i].default_on);
						leds[i].p = 0;
					}
				} else {
					leds[i].c++;
				}
			}
		} else if(leds[i].blink) { // turn led on for a single tick
			digitalWrite(leds[i].pin, leds[i].default_on); // turn led on
			leds[i].state = true;
			leds[i].blink = false;
		} else {
			digitalWrite(leds[i].pin, !leds[i].default_on); // turn led off
			leds[i].state = false;
		}
	}


}

void Notify::set(uint8_t id, bool state) {
	leds[id].playing = false;
	leds[id].state = state;
	digitalWrite(leds[id].pin, leds[id].default_on?state:!state);
}

void Notify::set_status(uint8_t id, conn_status_t status) {
	if(leds[id].status == status && leds[id].playing == true)
		return;

	leds[id].status = status;
	switch (status) {
	case STATUS_CONNECTED :
		play(id, pattern_connected);
		break;
	case STATUS_NOT_CONNECTED :
		play(id, pattern_not_connected);
		break;
	case STATUS_CONNECTION_LOST :
		play(id, pattern_connection_lost);
		break;
	}
}

void Notify::play(uint8_t id) {
	leds[id].playing = true;
	leds[id].p = 0;
	leds[id].c = 0;
	leds[id].state = true;
	digitalWrite(leds[id].pin, leds[id].default_on);
}

void Notify::play(uint8_t id, uint8_t *_pattern) {
	leds[id].pattern = _pattern;
	play(id);
}

void Notify::stop(uint8_t id) {
	leds[id].playing = false;
	digitalWrite(leds[id].pin, !leds[id].default_on);
}

void Notify::set_delay(uint8_t id, uint8_t delay) {
	leds[id].delay = delay;
}

void Notify::blink(uint8_t id) {
	leds[id].playing = false; // stop pattern
	leds[id].blink = true;
}

