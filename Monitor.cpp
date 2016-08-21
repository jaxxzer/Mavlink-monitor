#include "Monitor.h"

Monitor::Monitor() : 
looptime(0),
lastus(0),
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
test(0),

params(),
battery(),
pixhawk(),
rangefinder(),
notify()

{

}

void Monitor::init() {
    //params.init();
  battery.init(&params);
  pixhawk.init(&params);
  rangefinder.init(&params);
  notify.init(&params);
  
  params.add("SRATE1", &SRATE1);
  params.add("SRATE2", &SRATE2);
  params.add("BAUD_PIX", &BAUD_PIX);
  params.add("BAUD_ESP", &BAUD_ESP);
  params.add("BAUD_232", &BAUD_232);
  params.add("TEST", &test);

  params.load_all();




  Serial.println("Testing notify");
  Serial.println("Turning led on");
  notify.set(LED_MAPLE, true);
  delay(5000);
  Serial.println("Turning led off");
  notify.set(LED_MAPLE, false);
  delay(5000);
  Serial.println("Playing pattern");
  notify.play(LED_MAPLE);
}

void Monitor::run() {
  while(true) {
    uint32_t tnow = millis();

//    looptime = tnowus - lastus;
//    lastus = tnowus;
//    
//    //30second loop
//    if(tnow - last30s > 1000 * 30) {
//      last30s = tnow;
//      pixhawk.send_request_data_stream();
//    }
//    

    battery.update();
    pixhawk.update();
    rangefinder.update();
    notify.update();

    // 1Hz loop
    if(tnow - last1Hz > 1000/1) {
    
      last1Hz = tnow;
      Serial.print("test= ");
      Serial.print(test);
      Serial.print(" fart= ");
      Serial.println(fart);
      
      pixhawk.send_heartbeat();
    
    }

    
    
    // 5Hz loop
    if(tnow - last5Hz > 1000/5) {
      last5Hz = tnow;
      pixhawk.send_system_status();
//      pixhawk.send_distance_sensor(rangefinder.range);
//      
//      Serial3.write('Z');
//      
//      water = digitalRead(PB6);
    }
//    
//    
//    
//    // 10Hz loop
//    if(tnow - last10Hz > 1000/10) {
//      //send_heartbeat();
//      last10Hz = tnow;
//    
//    
//    }
//    
//    if(tnow - last50Hz > 1000/50) {
//      last50Hz = tnow;
//    
//    }
//    
//    if(SRATE1 > 20) SRATE1 = 20;
//    if(SRATE1 < 1) SRATE1 = 1;
//    if(tnow - lastS1 > 1000/SRATE1) {
//      lastS1 = tnow;
//      pixhawk.send_heartbeat();
//      
//    }
//    
//    //range_receive();
//    pixhawk.comm_receive();

  }
}


