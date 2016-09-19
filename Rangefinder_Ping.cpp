#include "Rangefinder_Ping.h"

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
		range_filt(0.25)
{
	pinMode(PIN_TRIGGER, OUTPUT);
	pinMode(PIN_ECHO, INPUT);
	digitalWrite(PIN_TRIGGER, LOW);
	attachInterrupt(PIN_ECHO, echo_receive, CHANGE);
}

void Rangefinder_Ping::init_params(Parameters *_params) {
	params = _params;
	if(params != NULL) {
		params->add("PINGRATE", &PINGRATE, 0, 50);
		params->add("RANGE_ENABLE", &RANGE_ENABLE, 0, 1);
		params->add("LPF_ENABLE", &LPF_ENABLE, 0, 1);
		params->add("LPF_CUTOFF", &LPF_CUTOFF, 0.001, 100);
	}

}

void Rangefinder_Ping::init() {

}

void Rangefinder_Ping::update() {

	if(LPF_ENABLE && LPF_CUTOFF != range_filt.get_cutoff_freq()) {
		range_filt.set_cutoff_frequency(LPF_CUTOFF);
	}
	uint32_t tnow = millis();
	if(echo_received) {
		echo_received = false;
		range = micros_to_cm(echo_end - echo_start);
		if(LPF_ENABLE) {
			range_filt.apply(range, (tnow-last_response_ms) / 1000.0f);
		}
		last_response_ms = tnow;

#if DEBUG_OUTPUT
		Serial.print("R: ");
		Serial.println(range);
#endif
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
	return microseconds/2/29; //round trip, 29us per cm in air
}
