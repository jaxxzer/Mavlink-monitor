#include <EEPROM.h>
#include <Arduino.h>  // for type definitions (ie uint8_t)
#include "MapleMini.h" // board pinout definition
#include "Param.h"
#include "Monitor.h"
#include "Mavlink.h"

#include "SUB/ardupilotmega/mavlink.h"

//#include "SUB/protocol.h"
//#include "SUB/mavlink_helpers.h"


Monitor m;

void setup() {
  m = Monitor();

  Serial.begin(115200);  //USB debugging

  delay(10000);





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


  pinMode(PIN_LED, OUTPUT); // Onboard led
  pinMode(PB6, INPUT); // MapleMini pin 16

  //send_text("Online");
  Serial.println("ONLINE");
  digitalWrite(PIN_LED, HIGH);
}

void loop() {
  m.init();
  m.run();
  
  
}

