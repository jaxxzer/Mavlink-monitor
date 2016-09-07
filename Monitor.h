#ifndef MONITOR_H
#define MONITOR_H

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


class Monitor {
public:

	Monitor();
	Parameters params;
	Mavlink pixhawk;
  Mavlink pixhawk1;
  Mavlink pixhawk2;
	BatteryMonitor battery;
	Rangefinder_Ping rangefinder;
  Notify notify;
  WaterDetector waterdetector;
  Dipswitch dipswitch;

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

private:
  ///////////////////////////
  /////Scheduling////////////
  ///////////////////////////

  uint16_t looptime; // performance
  uint32_t last_us; // performance
  uint32_t totaltime;
  uint32_t loopcounter;
  
  uint32_t last30s;
  uint32_t last1Hz;
  uint32_t last5Hz;
  uint32_t last10Hz;
  uint32_t last50Hz;
  uint32_t lastS1;
  uint32_t lastS2;

};

#endif
