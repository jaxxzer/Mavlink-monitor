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
pixhawk(8, 1, &Serial1, MAVLINK_COMM_0),
pixhawk1(9, 1, &Serial2, MAVLINK_COMM_1),
pixhawk2(10, 1, &Serial3, MAVLINK_COMM_2),
rangefinder(),
notify(),
waterdetector()

{}

void Monitor::init() {
  
  Serial.begin(921600);  //USB debugging
  Serial1.begin(921600); //pixhawk
  Serial2.begin(921600); //esp
  Serial3.begin(921600); //rs232

  delay(2000);
  
  params.add("SRATE1", &SRATE1);
  params.add("SRATE2", &SRATE2);
  params.add("BAUD_PIX", &BAUD_PIX);
  params.add("BAUD_ESP", &BAUD_ESP);
  params.add("BAUD_232", &BAUD_232);

  battery.init(&params);
  pixhawk.init(&params);
  pixhawk1.init(&params);
  pixhawk2.init(&params);
  //rangefinder.init(&params);
  notify.init(&params);
  waterdetector.init(&params);
  
  params.load_all(); // must not be called until all parameters have been added

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
    pixhawk1.update();
    pixhawk2.update();
    //rangefinder.update();
    notify.update();
    waterdetector.update();

    notify.set_status(LED_MAPLE, pixhawk1.status);
    //notify.set_status(LED_MAPLE, rangefinder.status);
    
    //30second loop
    if(tnow - last30s > 1000 * 10) {
      last30s = tnow;
    }

    // 1Hz loop
    if(tnow - last1Hz > 1000/1) {
      last1Hz = tnow;
      pixhawk.send_heartbeat();
      pixhawk1.send_heartbeat();
      pixhawk2.send_heartbeat();
    }

    // 5Hz loop
    if(tnow - last5Hz > 1000/5) {
      last5Hz = tnow;
    }
    
    // 10Hz loop
    if(tnow - last10Hz > 1000/10) {
      last10Hz = tnow;
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
      pixhawk1.send_system_status(load, waterdetector.detected);
      pixhawk2.send_system_status(load, waterdetector.detected);
    }

    // Custom rate 2
    if(SRATE2 > 20) SRATE2 = 20;
    if(SRATE2 < 1) SRATE2 = 1;
    if(tnow - lastS2 > 1000/SRATE2) {
      lastS2 = tnow;

      if(rangefinder.status == STATUS_CONNECTED)
        pixhawk.send_distance_sensor(rangefinder.range);
    } 
}


