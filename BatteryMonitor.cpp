#include "BatteryMonitor.h"

BatteryMonitor::BatteryMonitor() :
_voltage_pin(-1),
_current_pin(-1),
VSCALE(0),
CSCALE(0)
{}

void BatteryMonitor::init(Parameters *_params) {
  params = _params;
  params->add("VSCALE", &VSCALE);
  params->add("CSCALE", &CSCALE);
}

void BatteryMonitor::update() {
  return;
}

uint16_t BatteryMonitor::measureVoltage() {
  if(_voltage_pin == -1) {
    return 0;
  }
  float v = analogRead(_voltage_pin);
  v = analogRead(_voltage_pin);
  v = v * VSCALE * 10 / 360;
  return (uint16_t)v;
}
uint16_t BatteryMonitor::measureCurrent() {
  if(_current_pin == -1) {
    return 0;
  }
  float c = analogRead(_current_pin);
  c = analogRead(_current_pin);
  c = c * CSCALE * 10 / 360;
  return (int16_t)c;
}

//  uint16_t voltage = measureVoltage();
//  if(voltage < 9000) {
//    cells = 2;
//  } else if(voltage < 13000) {
//    cells = 3;
//  } else {
//    cells = 4;
//  }
