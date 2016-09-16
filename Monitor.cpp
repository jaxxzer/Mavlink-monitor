#include "Monitor.h"

Monitor::Monitor() : 
looptime(0),
last_us(0),
last30s(100),
last1Hz(200),
last5Hz(300),
last10Hz(400),
last50Hz(500),
lastS1(600),
lastS2(700),
SRATE1(0),
SRATE2(0),
BAUD_PIX(0),
BAUD_ESP(0),
BAUD_232(0),
loopcounter(0),
totaltime(0),

params(),
battery(),
pixhawk(8, 1, &Serial, MAVLINK_COMM_0),
esp(9, 1, &Serial2, MAVLINK_COMM_1),
rangefinder(),
notify(),
waterdetector(),
dipswitch(),
tempsensor(),
button()

{}

void Monitor::init() {

	Serial.begin(115200);  //USB debugging
	Serial1.begin(115200); //pixhawk
	Serial2.begin(115200); //esp
	Serial3.begin(115200); //rs232

	params.add("SRATE1", &SRATE1);
	params.add("SRATE2", &SRATE2);
	params.add("BAUD_PIX", &BAUD_PIX);
	params.add("BAUD_ESP", &BAUD_ESP);
	params.add("BAUD_232", &BAUD_232);
  params.add("PIC_INTERVAL", &PIC_INTERVAL);
  //params.add("DEBUG_LEVEL", &DEBUG_LEVEL);

	battery.init(&params);
	pixhawk.init(&params);
	esp.init(&params);
	rangefinder.init(&params);
	notify.init(&params);
	waterdetector.init(&params);
	dipswitch.init();
	tempsensor.init();
  button.init();

	params.load_all(); // must not be called until all parameters have been added

  // Set mavlink sysid according to dipswitch state
	// id 0 is broadcast, id 1 is pixhawk itself, we can be (0~7) + 2 = 2~9
  pixhawk._sysid = (dipswitch.get_state() & 0b00000111) + 2;
  esp._sysid = (dipswitch.get_state() & 0b00000111) + 2;
  
  // dipswitch pole #3 (zero indexed) determines output of main mavlink
  if(dipswitch.get_state() & 0b00001000) {
	  pixhawk.set_port(&Serial); // mavlink through USBSerial
  } else {
	  pixhawk.set_port(&Serial1); // mavlink through PA9 and PA10 HardwareSerial
  }

	///////////////////////
	// Echo dipswitch states across leds
	for(int i = 0; i < DIPSWITCH_NUM_POLES; i++) {
		notify.set(i, dipswitch.get_state(i));
	}

	delay(1000);

	// All leds off
	for(int i = 0; i < DIPSWITCH_NUM_POLES; i++) {
		notify.set(i, false);
	}
	////////////////////////

	Serial.println("ONLINE");
}

void Monitor::run() {
	uint32_t tnow = millis();
	uint32_t tnow_us = micros();
	uint32_t looptime = tnow_us - last_us;
	last_us = tnow_us;

	loopcounter++;
	totaltime+=looptime;

	battery.update();
	pixhawk.update();
	esp.update();
	rangefinder.update();
	notify.update();
	waterdetector.update();
	dipswitch.update();
	tempsensor.update();
  button.update();
  
	notify.set_status(LED_MAPLE, pixhawk.status);
	notify.set_status(LED_1, esp.status);
	notify.set_status(LED_2, rangefinder.status);

	//30second loop
	if(tnow - last30s > 1000 * 10) {
    esp.send_nav_cmd_do_trigger_control(PIC_INTERVAL);
		last30s = tnow;
	}

	// 1Hz loop
	if(tnow - last1Hz > 1000/1) {
		last1Hz = tnow;
		pixhawk.send_heartbeat();
		esp.send_heartbeat();
    
	}

	// 5Hz loop
	if(tnow - last5Hz > 1000/5) {
		last5Hz = tnow;
		

	}

	// 10Hz loop
	if(tnow - last10Hz > 1000/10) {

		last10Hz = tnow;
    if(rangefinder.RANGE_ENABLE) {
      if(rangefinder.LPF_ENABLE) {
        pixhawk.send_distance_sensor(rangefinder.range_filt.get(), rangefinder.range);
      } else {
          pixhawk.send_distance_sensor(rangefinder.range, rangefinder.range);
      }
    }
	}

	if(tnow - last50Hz > 1000/50) {
		last50Hz = tnow;
	}

	// Custom rate 1
	if(SRATE1 > 20) SRATE1 = 20;
	if(SRATE1 < 1) SRATE1 = 1;
	if(tnow - lastS1 > 1000/SRATE1) {
		lastS1 = tnow;

		uint32_t load = totaltime/loopcounter; // Average looptime in us since last calculated
		totaltime = 0;
		loopcounter = 0;

		pixhawk.send_system_status(load, waterdetector.detected);
	}

	// Custom rate 2
	if(SRATE2 > 20) SRATE2 = 20;
	if(SRATE2 < 1) SRATE2 = 1;
	if(tnow - lastS2 > 1000/SRATE2) {
		lastS2 = tnow;

		//if(rangefinder.status == STATUS_CONNECTED)
	}
}


