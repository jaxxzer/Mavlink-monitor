#include "Notify.h"
#include "MapleMini.h"

Notify::Notify() :
params(NULL),
_last_tick_ms(0)
//flags()
{
	memset(leds, 0, sizeof(leds));
	leds[LED_MAPLE].default_on = HIGH;
	leds[LED_MAPLE].pin = PIN_LED_MAPLE;

	leds[LED_1].pin = -1;
	leds[LED_2].pin = -1;
	leds[LED_3].pin = -1;
}

void Notify::init(Parameters *_params) {
	params = _params;

	leds[LED_MAPLE].default_on = true;
	leds[LED_MAPLE].pattern[0] = 5;
	leds[LED_MAPLE].pattern[1] = 10;
	leds[LED_MAPLE].pattern[2] = 5;
	leds[LED_MAPLE].pattern[3] = 25;
//	  leds[LED_MAPLE].pattern[4] = 5;
//	  leds[LED_MAPLE].pattern[5] = 20;
//	  leds[LED_MAPLE].pattern[6] = 5;
//	  leds[LED_MAPLE].pattern[7] = 50;
	leds[LED_MAPLE].playing = false;

	for(int i = 0; i < NOTIFY_NUM_LEDS; i++) {
		if(leds[i].pin >= 0) {
			pinMode(leds[i].pin, OUTPUT);
			digitalWrite(leds[i].pin, !leds[i].default_on); // init all LEDS off
		}
	}
}

void Notify::update() {
	uint32_t tnow = millis();
	if(tnow < _last_tick_ms + NOTIFY_UPDATE_MS) {
		return;
	}

	_last_tick_ms = tnow;

	for(int i = 0; i < NOTIFY_NUM_LEDS; i++) {
		if(leds[i].pin != -1 && leds[i].playing) {

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
	}
}

void Notify::set(uint8_t index, bool state) {
	leds[index].playing = false;
  leds[index].state = state;
	digitalWrite(leds[index].pin, leds[index].default_on?state:!state);
}

void Notify::play(uint8_t index) {
	leds[index].playing = true;
	leds[index].p = 0;
	leds[index].c = 0;
  leds[index].state = true;
	digitalWrite(leds[index].pin, leds[index].default_on);
}

void Notify::stop(uint8_t index) {
	leds[index].playing = false;
	digitalWrite(leds[index].pin, !leds[index].default_on);
}

