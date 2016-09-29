#ifndef BME280_H
#define BME280_H

class BME280 {
public:
	BME280();
	void init();
	void update();

	float pressure; // Pascal
	float humidity; // Relative %
	float temperature; // Degrees C

private:
	void set_mode(uint8_t osrs_h, uint8_t osrs_t, uint8_t osrs_p, uint8_t mode);
	int32_t calculate_temp(int32_t adc_T);
	uint32_t calculate_pressure(int32_t adc_P);
	uint32_t calculate_humidity(int32_t adc_H);
	void read_calibration();
	void print_calibration();
	void print_state();
	int32_t read_temp_adc();
	int32_t read_press_adc();
	int32_t read_hum_adc();

	uint32_t last_update_ms;
	uint32_t update_interval_ms;

	int32_t t_fine;

	// Calibration
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
};

#endif
