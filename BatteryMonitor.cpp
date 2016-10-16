#include "BatteryMonitor.h"
#include "MapleMini.h"
#define BATTMONITOR_DEBUG 0
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
		params->addFloat(param_v_scale, "V_SCALE", &V_SCALE, 0, 100, 8.7);
		params->addFloat(param_c_scale, "C_SCALE", &C_SCALE, 0, 100, 16.443);
		params->addFloat(param_v_offset, "V_OFFSET", &V_OFFSET, -1000, 1000, 0);
		params->addFloat(param_c_offset, "C_OFFSET", &C_OFFSET, -1000, 1000, 0); // offset in mA
		params->addFloat(param_v_lpfcut, "V_LPFCUT", &V_LPFCUT, 0.001, 100, 0.2);
		params->addFloat(param_c_lpfcut, "C_LPFCUT", &C_LPFCUT, 0.001, 100, 0.2);
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

	// voltage in mV
	voltage = measure_voltage();
	// current in mA
	current = measure_current();

	voltage_filt.apply(voltage, (tnow - _last_update_ms) / 1000.0f);
	current_filt.apply(current, (tnow - _last_update_ms) / 1000.0f);

#if BATTMONITOR_DEBUG
	Serial.print("BATTMONITOR: cells: ");
	Serial.print(cells);
	Serial.print("\tv: ");
	Serial.print(voltage);
	Serial.print("\tvfilt: ");
	Serial.print(voltage_filt.get());
	Serial.print("\tremain: ");
	Serial.println(remaining());
#endif

	_last_update_ms = tnow;
}

uint16_t BatteryMonitor::measure_voltage() {
	if(_voltage_pin == -1) {
		return 0;
	}

	float v = analogRead(_voltage_pin);
#if BATTMONITOR_DEBUG
	Serial.print("BATTMONITOR: raw vadc: ");
	Serial.println(v);
#endif
	v = (v * V_SCALE) + V_OFFSET;
	return (uint16_t)v;
}

uint16_t BatteryMonitor::measure_current() {
	if(_current_pin == -1) {
		return 0;
	}

	float c = analogRead(_current_pin);
#if BATTMONITOR_DEBUG
	Serial.print("BATTMONITOR: raw cadc: ");
	Serial.println(c);
#endif
	c = (c * C_SCALE) + C_OFFSET;
	return (uint16_t)c;
}

// count the number of cells, should be called once only during init as it is blocking
// This only works after current and voltage has been calculated
uint8_t BatteryMonitor::count_cells() {
	uint8_t num_samples = 15;
	float vsum = 0;

	for(int i = 0; i < num_samples; i++) {
		vsum += measure_voltage();
		delay(50);
	}

	uint16_t v = vsum / num_samples;
	uint8_t cells = v / CELL_VMIN;
#if BATTMONITOR_DEBUG
	Serial.print("BATTMONITOR: Cells: ");
	Serial.println(cells);
#endif

	return constrain(cells, 3, 4); // Works for 3 or 4 cells
}

// Measure the percent remaining taken as the interval between max charge and min charge
// Returns percent (0 ~ 100)
uint8_t BatteryMonitor::remaining() {
	uint8_t remaining = 100.0f * ((CELL_VMAX - CELL_VMIN) - (CELL_VMAX - (voltage_filt.get() / cells)))  / (CELL_VMAX - CELL_VMIN);
	return constrain(remaining, 0, 100);
}
