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
VSCALE(0),
CSCALE(0),
SRATE1(0),
SRATE2(0),
BAUD_PIX(0),
BAUD_ESP(0),
BAUD_232(0),
test(0),
fart(0),

params()

//pixhawk(&params)

{

}

void Monitor::init() {
    //params.init();
  
  Serial.println("Loading");
  delay(1000);
  params.add("VSCALE", &VSCALE);
  params.add("CSCALE", &CSCALE);
  params.add("SRATE1", &SRATE1);
  params.add("SRATE2", &SRATE2);
  params.add("BAUD_PIX", &BAUD_PIX);
  params.add("BAUD_ESP", &BAUD_ESP);
  params.add("BAUD_232", &BAUD_232);
  params.add("TEST", &test);
////
  params.load_all();
}

void Monitor::run() {
  while(true) {
    delay(1000);
    Serial.println("here");

//    uint32_t tnowus = micros();
//    uint32_t tnow = millis();
//    
//    looptime = tnowus - lastus;
//    lastus = tnowus;
//    
//    //30second loop
//    if(tnow - last30s > 1000 * 30) {
//      last30s = tnow;
//      pixhawk.send_request_data_stream();
//    }
//    
//    // 1Hz loop
//    if(tnow - last1Hz > 1000/1) {
//    
//      last1Hz = tnow;
//      Serial.print("test= ");
//      Serial.print(test);
//      Serial.print(" fart= ");
//      Serial.println(fart);
//      
//      pixhawk.send_heartbeat();
//    
//    }
//    
//    
//    // 5Hz loop
//    if(tnow - last5Hz > 1000/5) {
//      last5Hz = tnow;
//      pixhawk.send_system_status();
//      pixhawk.send_distance_sensor(range*100);
//      
//      Serial3.write('Z');
//      
//      water = digitalRead(PB6);
//    }
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

uint16_t Monitor::measureVoltage() {
  float v = analogRead(ADC_VOLTAGE);
  v = analogRead(ADC_VOLTAGE);
  v = v * VSCALE * 10 / 360;
  return (uint16_t)v;
}
uint16_t Monitor::measureCurrent() {
  float c = analogRead(ADC_CURRENT);
  c = analogRead(ADC_CURRENT);
  c = c * CSCALE * 10 / 360;
  return (int16_t)c;
}
