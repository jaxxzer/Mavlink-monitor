#include "Button.h"
#include "MapleMini.h"
#include "Monitor.h"
extern Monitor monitor;

Button::Button() :
last_update_ms(0)
{}

void Button::init() {
  pinMode(PIN_BUTTON, INPUT);
}

void Button::update() {
  uint32_t tnow = millis();
  if(tnow > last_update_ms + 500) {
    last_update_ms = tnow;
    if(digitalRead(PIN_BUTTON)) {
      monitor.esp.send_nav_cmd_preflight_reboot_shutdown();
    }
  }
}

