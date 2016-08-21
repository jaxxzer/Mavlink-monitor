#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>
#include "Param.h"

#define CELL_VMAX 4200.0
#define CELL_VMIN 3500.0

class BatteryMonitor {
  public:
    BatteryMonitor();

    void init(Parameters *_params);
    void update(void);

    uint16_t measureVoltage(void);
    uint16_t measureCurrent(void);

    Parameters *params;
    
    uint8_t cells;

    float VSCALE;
    float CSCALE;

    

  private:
    int8_t _voltage_pin;
    int8_t _current_pin;
};

#endif
