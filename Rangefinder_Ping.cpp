#include "Rangefinder_Ping.h"
#define RANGE_MAX 60
#define RANGE_MIN 15
#define RANGEFINDER_TIMEOUT_MS 500

volatile bool echo_received = false;
volatile bool echo_receiving = false;
volatile uint32_t echo_start = 0;
volatile uint32_t echo_end = 0;
void echo_receive() {
	echo_receiving = !echo_receiving;
	if(echo_receiving)
		echo_start = micros();
	else {
		echo_end = micros();
		echo_received = true;
	}
}

Rangefinder_Ping::Rangefinder_Ping() :
		last_ping_ms(0),
		last_response_ms(0),
		range_filt(0.25),
		status(STATUS_NOT_CONNECTED),
		last_valid_range(0),
		out_of_range_flag(true)
{
	pinMode(PIN_TRIGGER, OUTPUT);
	pinMode(PIN_ECHO, INPUT_PULLDOWN);
	digitalWrite(PIN_TRIGGER, LOW);
	attachInterrupt(PIN_ECHO, echo_receive, CHANGE);
}

void Rangefinder_Ping::init_params(Parameters *_params) {
	params = _params;
	if(params != NULL) {
		params->addUint32("PINGRATE", &PINGRATE, 0, 50, 10);
		params->addUint32("RANGE_ENABLE", &RANGE_ENABLE, 0, 1, 0);
		params->addUint32("LPF_ENABLE", &LPF_ENABLE, 0, 1, 0);
		params->addFloat("LPF_CUTOFF", &LPF_CUTOFF, 0.001, 100, 1);
	}

}

void Rangefinder_Ping::init() {

}

void Rangefinder_Ping::update() {

	// exit if rangefinder is not enabled
	if(PINGRATE == 0 || !RANGE_ENABLE) {
		status = STATUS_NOT_CONNECTED;
		range = 0;
		range_filt.reset(0);
		return;
	}

	if(LPF_ENABLE && LPF_CUTOFF != range_filt.get_cutoff_freq()) {
		range_filt.set_cutoff_frequency(LPF_CUTOFF);
	}

	uint32_t tnow = millis();
	if(echo_received) {
		status = STATUS_CONNECTED;
		echo_received = false;
		range = micros_to_cm(echo_end - echo_start);

		if(LPF_ENABLE && range != 0) {
			if(out_of_range_flag) {
				out_of_range_flag = false;
				range_filt.reset(range); // this is the first valid range reading since we went out of range
			} else {
				range_filt.apply(range, (tnow-last_response_ms) / 1000.0f);
			}
		}

		last_response_ms = tnow;

		// Micron reads 0 when out of range
		if(range > 0) {
			last_valid_range = range;

		} else {
			if(last_valid_range < RANGE_MIN + 5) {
				range = 1; // out of range low
			} else {
				range = 9999; // out of range high
			}

			range_filt.reset(range); // reset filter in case it's enabled
			out_of_range_flag = true;
		}
#if DEBUG_OUTPUT
		Serial.print("R: ");
		Serial.println(range);
#endif
	} else if(status == STATUS_CONNECTED && tnow > last_response_ms + RANGEFINDER_TIMEOUT_MS) {
		status = STATUS_CONNECTION_LOST;
	}

	if(tnow > last_ping_ms + 1000/PINGRATE && (last_response_ms >= last_ping_ms || tnow > last_ping_ms + PING_TIMEOUT_MS)) { // Time to ping again
		trigger(PIN_TRIGGER);
		last_ping_ms = tnow;
	}

}

void Rangefinder_Ping::trigger(int16_t pin) {
	if(pin >= 0) {
		digitalWrite(pin, HIGH);
		delayMicroseconds(10); //TODO hardware timer instead
		digitalWrite(pin, LOW);
	}
}

uint16_t Rangefinder_Ping::micros_to_cm(uint32_t microseconds) {
	uint16_t cm = microseconds/2/29;
	if(cm > RANGE_MAX || cm < RANGE_MIN) {
		return 0;
	} else return cm;
//	return microseconds/2/29; //round trip, 29us per cm in air
}
