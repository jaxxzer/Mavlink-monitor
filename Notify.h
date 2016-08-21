#ifndef NOTIFY_H
#define NOTIFY_H

#include "Param.h"

#define NOTIFY_UPDATE_HZ 50
#define NOTIFY_UPDATE_MS 20

#define NOTIFY_NUM_LEDS 4
#define NOTIFY_PATTERN_MAX 8

  typedef enum {
    LED_MAPLE,
    LED_1,
    LED_2,
    LED_3
  } notify_id_t;

class Notify {
public:

	//	struct flags_t {
	//		uint16_t initializing		:1;
	//		uint16_t esp_ok			:1;
	//		uint16_t rangefinder_status :1;
	//		uint16_t pixhawk_status		:1;
	//		uint16_t monitor_status		:1;
	//	};



//	static const uint8_t[NOTIFY_PATTERN_MAX] pattern1 =
//	{30 , 5, 30, 5, 0, 0, 0, 0};
//
//	static const uint_8t[NOTIFY_PATTERN_MAX] pattern2 =
//	{5 , 30, 5, 30, 0, 0, 0, 0};

	typedef struct {
		notify_id_t id; //id of led
		uint8_t pattern[NOTIFY_PATTERN_MAX]; //pattern array, the number at each index indicates number of ticks to count before next toggle
		uint8_t p; //pattern pointer
		uint8_t c; //tick counter
		bool state; //current state of led (on or off)
		int8_t pin; //pin that led is attached to
		bool default_on; //default state of pin to turn LED on
		bool playing; //true if pattern is playing
	} led_t;

	Notify();

	void init(Parameters *_params);
	void update();

	void set(uint8_t index, bool state); // set state
	void play(uint8_t index); // start playing pattern
	void stop(uint8_t index); // stop playing pattern

	Parameters *params;

//	struct flags_t flags;

private:
	led_t leds[NOTIFY_NUM_LEDS];
	uint32_t _last_tick_ms;
};

#endif
