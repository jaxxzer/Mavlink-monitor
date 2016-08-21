#include "Monitor.h"


Monitor m;

void setup() {
  m = Monitor();

  Serial.begin(115200);  //USB debugging




  //SerialManager.init();

  Serial1.begin(115200); //pixhawk
  Serial2.begin(115200); //esp
  Serial3.begin(115200); //rs232
  
  
//  uint16_t voltage = measureVoltage();
//  if(voltage < 9000) {
//    cells = 2;
//  } else if(voltage < 13000) {
//    cells = 3;
//  } else {
//    cells = 4;
//  }



  //send_text("Online");
  Serial.println("ONLINE");

}

void loop() {
  m.init();
  m.run();
  
  
}

