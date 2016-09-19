#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>
#include "Param.h"
#include "LowPassFilter.h"

#define CELL_VMAX 4200.0f
#define CELL_VMIN 3500.0f

class BatteryMonitor {
  public:
    BatteryMonitor();

    // Add parameters
    void init_params(Parameters *_params);

    void init(void);
    void update(void);

    uint16_t measure_voltage(void);
    uint16_t measure_current(void);
    uint8_t count_cells(void); // Determine the number of cells in lipo battery
    uint8_t remaining(void); // Percent remaining * 100

    Parameters *params;
    
    float V_SCALE; // Scale factor for ADC measurements
    float C_SCALE;
    float V_OFFSET; // Offset to be added to scaled measurements
    float C_OFFSET;
    float V_LPFCUT; // Cuttoff frequencies for low pass filters
    float C_LPFCUT;

    // Last reading
    uint16_t voltage;
    uint16_t current;
    
    // Filtered readings
    LowPassFilterFloat voltage_filt;
    LowPassFilterFloat current_filt;

    // Number of lipo cells (used to determine percent remaining)
    uint8_t cells;


  private:
    int8_t _voltage_pin;
    int8_t _current_pin;
    uint32_t _last_update_ms;
    uint16_t _update_interval_ms;
};

#endif
