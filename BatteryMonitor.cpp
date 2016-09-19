#include "BatteryMonitor.h"
#include "MapleMini.h"

BatteryMonitor::BatteryMonitor() :
_voltage_pin(PIN_ADC_VOLTAGE),
_current_pin(PIN_ADC_CURRENT),
_last_update_ms(0),
_update_interval_ms(100),
cells(0)
{
	  pinMode(_voltage_pin, INPUT);
	  pinMode(_current_pin, INPUT);
}

void BatteryMonitor::init_params(Parameters *_params) {
  params = _params;
  if(params != NULL) {
	  params->add("V_SCALE", &V_SCALE);
	  params->add("C_SCALE", &C_SCALE);
	  params->add("V_OFFSET", &V_OFFSET);
	  params->add("C_OFFSET", &C_OFFSET);
	  params->add("V_LPFCUT", &V_LPFCUT);
	  params->add("C_LPFCUT", &C_LPFCUT);
  }
}

void BatteryMonitor::init() {
	cells = count_cells();
}

void BatteryMonitor::update() {
	uint32_t tnow = millis();

	if(tnow < _last_update_ms + _update_interval_ms) {
		return;
	}

	if(V_LPFCUT != voltage_filt.get_cutoff_freq()) {
		V_LPFCUT = constrain(V_LPFCUT, 0.01, 20);
		voltage_filt.set_cutoff_frequency(V_LPFCUT);
	}

	if(C_LPFCUT != current_filt.get_cutoff_freq()) {
		C_LPFCUT = constrain(C_LPFCUT, 0.01, 20);
		current_filt.set_cutoff_frequency(C_LPFCUT);
	}

	voltage = measure_voltage();
	current = measure_current();

	voltage_filt.apply(voltage, (tnow - _last_update_ms) / 1000.0f);
	current_filt.apply(current, (tnow - _last_update_ms) / 1000.0f);

	_last_update_ms = tnow;
}

uint16_t BatteryMonitor::measure_voltage() {
  if(_voltage_pin == -1) {
    return 0;
  }

  float v = analogRead(_voltage_pin);
  v = (v * V_SCALE) + V_OFFSET;
  return (uint16_t)v;
}

uint16_t BatteryMonitor::measure_current() {
  if(_current_pin == -1) {
    return 0;
  }

  float c = analogRead(_current_pin);
  c = (c * C_SCALE) + C_OFFSET;
  return (uint16_t)c;
}

// count the number of cells, should be called once only during init as it is blocking
// This only works after current and voltage has been calculated
uint8_t BatteryMonitor::count_cells() {
	uint8_t num_samples = 8;
	float vsum = 0;

	for(int i = 0; i < num_samples; i++) {
		vsum += measure_voltage();
		delay(50);
	}

	uint16_t v = vsum / num_samples;
	uint8_t cells = v / CELL_VMIN;

	return constrain(cells, 3, 4); // Works for 3 or 4 cells
}

// Measure the percent remaining taken as the interval between max charge and min charge
// Returns percent * 100 (0 ~ 10000)
uint16_t BatteryMonitor::remaining() {
	uint16_t remaining = (CELL_VMAX - (voltage_filt.get() / cells))  / (CELL_VMAX - CELL_VMIN);
	return constrain(remaining, 0, 10000);
}
