#ifndef MONITOR_H
#define MONITOR_H

#define RANGEFINDER_MICRON 0
#define RANGEFINDER_PING 1

#define RANGEFINDER_TYPE RANGEFINDER_MICRON

#include <Arduino.h>
#include "MapleMini.h"
#include "Mavlink.h"
#include "Param.h"
#include "Rangefinder.h"
#include "Rangefinder_Ping.h"
#include "BatteryMonitor.h"
#include "Notify.h"
#include "WaterDetector.h"
#include "Dipswitch.h"
#include "TempSensor.h"
#include "Button.h"
#include "BME280.h"


class Monitor {
public:

	Monitor();
	Parameters params;
	Mavlink pixhawk;
	Mavlink esp;
	BatteryMonitor battery;
#if RANGEFINDER_TYPE == RANGEFINDER_MICRON
	Rangefinder rangefinder;
#elif RANGEFINDER_TYPE == RANGEFINDER_PING
	Rangefinder_Ping rangefinder;
#endif
	Notify notify;
	WaterDetector waterdetector;
	Dipswitch dipswitch;
	TempSensor tempsensor;
	Button button;
	BME280 bme;

	void run(void);
	void init(void);

	///////////////////////////
	///////PARAMETERS//////////
	///////////////////////////

	uint32_t SRATE1;
	uint32_t SRATE2;
	uint32_t BAUD_PIX;
	uint32_t BAUD_ESP;
	uint32_t BAUD_232;
	uint32_t PIC_INTERVAL;
	uint8_t TEST;

private:
	///////////////////////////
	/////Scheduling////////////
	///////////////////////////

	uint16_t looptime; // performance
	uint32_t last_us; // performance
	uint32_t totaltime;
	uint32_t loopcounter;

	uint32_t last30s;
	uint32_t last10s;
	uint32_t last5s;
	uint32_t last1Hz;
	uint32_t last5Hz;
	uint32_t last10Hz;
	uint32_t last50Hz;
	uint32_t lastS1;
	uint32_t lastS2;

};

#endif
