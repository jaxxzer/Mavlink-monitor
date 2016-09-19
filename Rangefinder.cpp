#include <Arduino.h>
#include "Rangefinder.h"


Rangefinder::Rangefinder() :
range(0),
PINGRATE(0),
RANGE_ENABLE(0),
params(NULL),
status(STATUS_NOT_CONNECTED),
response_received(false),
last_request_ms(0),
last_response_ms(0),
range_filt(0.25)
{}

void Rangefinder::init_params(Parameters *_params) {
	params = _params;
	if(params != NULL) {
		params->add("PINGRATE", &PINGRATE, 0, 20, 7);
		params->add("RANGE_ENABLE", &RANGE_ENABLE, 0, 1, 0);
		params->add("LPF_ENABLE", &LPF_ENABLE, 0, 1, 0);
		params->add("LPF_CUTOFF", &LPF_CUTOFF, 0.001, 100, 1);
	}
}

void Rangefinder::init() {
	//constrain_params();
}

void Rangefinder::update() {

	// exit if rangefinder is not enabled
	if(PINGRATE == 0 || !RANGE_ENABLE) {
		status = STATUS_NOT_CONNECTED;
		return;
	}

	uint32_t tnow = millis();

	// when connected, dont request faster than rangefinder can respond
	// when disconnected, request ping at 1Hz until rangefinder responds
	if((status == STATUS_CONNECTED && last_response_ms > last_request_ms) || tnow > last_request_ms + 1000)
		range_request();

	if(last_request_ms >= last_response_ms)
		range_receive();

	// rangefinder is communicating
	if(range > 0 && tnow < last_response_ms + RANGEFINDER_TIMEOUT_MS) {
		status = STATUS_CONNECTED;
		return;
	}

	// rangefinder has stopped communicating
	if(status == STATUS_CONNECTED) {
		status = STATUS_CONNECTION_LOST;
		range = 0;
	}

}

void Rangefinder::range_request() {
	uint32_t tnow = millis();
	if(tnow > last_request_ms + (1000/PINGRATE)) {
		last_request_ms = tnow;
		Serial3.write('Z'); // Micron echosounder 'Interrogate' command
	}
}

// Parse micron range response
// Response is formatted as: 'XX.xxxm\n\r'
void Rangefinder::range_receive() {
	uint32_t tnow = millis();

	static char distance[20];
	static uint8_t index;

	while(Serial3.available() > 0) {
		uint8_t c = Serial3.read();
		switch(c) {
		case '0' ... '9':
		case '.':
			if(index < 19) // avoid overflow
				distance[index++] = c;
			break;

		case 'm':
			distance[index] = '\0';
			range = String(distance).toFloat() * 100;
			if(LPF_ENABLE) {
				range_filt.apply(range, (tnow-last_response_ms) / 1000.0f);
			}
			last_response_ms = tnow;
		case '\r':
		case '\n':
			index = 0;
			break;

		default:
			range = 0;
			index = 0;
			break;
		}
	}
}

void Rangefinder::constrain_params() {
	PINGRATE = constrain(PINGRATE, 0, 20);
	RANGE_ENABLE = constrain(RANGE_ENABLE, 0, 1);
	LPF_ENABLE = constrain(LPF_ENABLE, 0, 1);
	LPF_CUTOFF = constrain(LPF_CUTOFF, 0.01f, 20.0f);
}

