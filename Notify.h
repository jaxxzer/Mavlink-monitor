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

  typedef enum {
    STATUS_CONNECTED,
    STATUS_NOT_CONNECTED,
    STATUS_CONNECTION_LOST
  } conn_status_t;

static uint8_t pattern_connected[NOTIFY_PATTERN_MAX] = {3, 7, 3, 38, 0, 0, 0, 0};
static uint8_t pattern_not_connected[NOTIFY_PATTERN_MAX] = {10 , 10, 0, 0, 0, 0, 0, 0};
static uint8_t pattern_connection_lost[NOTIFY_PATTERN_MAX] = {3, 3, 3, 3, 3, 15, 0, 0};

class Notify {
public:
	typedef struct {
		notify_id_t id; //id of led
    conn_status_t status;
		uint8_t *pattern; //pattern array, the number at each index indicates number of ticks to count before next toggle
		uint8_t p; //pattern pointer
		uint8_t c; //tick counter
		bool state; //current state of led (on or off)
		int8_t pin; //pin that led is attached to
		bool default_on; //default state of pin to turn LED on
		bool playing; //true if pattern is playing
    uint8_t delay; //number of ticks to delay before playing pattern
	} led_t;

	Notify();

	void init(Parameters *_params);
	void update();

	void set(uint8_t id, bool state); // set state
	void play(uint8_t id); // start playing pattern
  void play(uint8_t id, uint8_t *_pattern);
	void stop(uint8_t id); // stop playing pattern

  void set_status(uint8_t id, conn_status_t status);

  void set_delay(uint8_t id, uint8_t delay);

  
	Parameters *params;

//	struct flags_t flags;

private:
	led_t leds[NOTIFY_NUM_LEDS];
	uint32_t _last_tick_ms;
};

#endif
