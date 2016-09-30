#include <Wire.h>
#include "BME280.h"

#define DEBUG_BME280 0

// Datasheet p31
// i2c device address tie SDO to GND for 0x76, to VDDIO for 0x77
#define BME280_ADDRESS 0x76

// Datasheet p22
#define BME280_ADD_COMPENSATION1 0x88
#define BME280_COMP1_LENGTH 26 // 0x88...0xA1
#define BME280_ADD_COMPENSATION2 0xE1
#define BME280_COMP2_LENGTH 7 // 0xE1...0xE7


// Datasheet p28,29
#define BME280_ADD_ADC_PRESS	0xF7
#define BME280_ADD_ADC_TEMP		0xFA
#define BME280_ADD_ADC_HUM		0xFD


// Datasheet p26
#define BME280_ADD_CTRL_HUM		0xF2
#define BME280_ADD_CTRL_MEAS	0xF4


// Datasheet p27
#define BME280_OVERSAMPLE_SKIP	0
#define BME280_OVERSAMPLE_1		1
#define BME280_OVERSAMPLE_2		2
#define BME280_OVERSAMPLE_4		3
#define BME280_OVERSAMPLE_8		4
#define BME280_OVERSAMPLE_16	5

#define BME280_MODE_SLEEP		0
#define BME280_MODE_FORCE		1
#define BME280_MODE_NORMAL		3

// Datasheet p28
// Standby time in ms
// Writes to the config register in normal mode may be ignored
#define BME280_T_STANDBY_0_5	0
#define BME280_T_STANDBY_62_5	1
#define BME280_T_STANDBY_125	2
#define BME280_T_STANDBY_250	3
#define BME280_T_STANDBY_500	4
#define BME280_T_STANDBY_1000	5
#define BME280_T_STANDBY_10		6
#define BME280_T_STANDBY_20		7

#define BME280_FILTER_COEFF_OFF	0
#define BME280_FILTER_COEFF_2	1
#define BME280_FILTER_COEFF_4	2
#define BME280_FILTER_COEFF_8	3
#define BME280_FILTER_COEFF_16	4

BME280::BME280() :
last_update_ms(0),
update_interval_ms(50)
{

}

void BME280::init_params(Parameters *_params) {
	params = _params;
	if(params != NULL) {
		params->add("BME_ENABLE", &BME_ENABLE, 0, 1, 0);
	}
}

void BME280::init() {
	if(!BME_ENABLE) {
		return;
	}
	Wire.scl_pin = 14;
	Wire.sda_pin = 13;
	Wire.begin(BME280_ADDRESS);
	set_mode(BME280_OVERSAMPLE_16, BME280_OVERSAMPLE_16, BME280_OVERSAMPLE_16, BME280_MODE_NORMAL);
	read_calibration();
#if DEBUG_BME280
	print_calibration();
#endif
}

void BME280::update() {
	if(!BME_ENABLE) {
		pressure = 0;
		temperature = 0;
		humidity = 0;
		return;
	}

	uint32_t tnow = millis();
	if(tnow < last_update_ms + update_interval_ms) {
		return;
	}
	last_update_ms = tnow;

	read_adcs();

	temperature = calculate_temperature(adc_T)/100.0f;
	pressure = calculate_pressure(adc_P)/256.0f;
	humidity = calculate_humidity(adc_H) / 1024.0f;
#if DEBUG_BME280
	print_state();
#endif
}

void BME280::print_state() {
	String s = "";
	s += "\nP: "; s += pressure;
	s += "\nT: "; s += temperature;
	s += "\nH: "; s += humidity;
	Serial.print(s);
}

// Datasheet p23
int32_t BME280::calculate_temperature(int32_t adc_T) {
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1))*((adc_T>>4) - ((int32_t)dig_T1))) >> 12)*((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T  = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t BME280::calculate_pressure(int32_t adc_P) {
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p<<31) - var2) * 3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}

// Datasheet p23
uint32_t BME280::calculate_humidity(int32_t adc_H) {
	int32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
			((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r *
			((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
			((int32_t)dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r>>12);
}

// Datasheet p22
void BME280::read_calibration() {
	Wire.beginTransmission(BME280_ADDRESS);
	Wire.write(BME280_ADD_COMPENSATION1);
	Wire.endTransmission();
	Wire.requestFrom(BME280_ADDRESS, BME280_COMP1_LENGTH);

	uint8_t buf[BME280_COMP1_LENGTH + BME280_COMP2_LENGTH];

	for(int i = 0; i < BME280_COMP1_LENGTH; i++) {
		buf[i] = Wire.read();
	}

	Wire.beginTransmission(BME280_ADDRESS);
	Wire.write(BME280_ADD_COMPENSATION2);
	Wire.endTransmission();
	Wire.requestFrom(BME280_ADDRESS, BME280_COMP2_LENGTH);

	for(int i = BME280_COMP1_LENGTH; i < BME280_COMP1_LENGTH + BME280_COMP2_LENGTH; i++) {
		buf[i] = Wire.read();
	}

	int i = 0;

	// 0x88
	dig_T1 = buf[i++] | (buf[i++]<<8); // unsigned short
	dig_T2 = buf[i++] | (buf[i++]<<8); // signed short
	dig_T3 = buf[i++] | (buf[i++]<<8); // signed short

	// 0x8E
	dig_P1 = buf[i++] | (buf[i++]<<8); // unsigned short
	dig_P2 = buf[i++] | (buf[i++]<<8); // signed short
	dig_P3 = buf[i++] | (buf[i++]<<8); // signed short
	dig_P4 = buf[i++] | (buf[i++]<<8); // signed short
	dig_P5 = buf[i++] | (buf[i++]<<8); // signed short
	dig_P6 = buf[i++] | (buf[i++]<<8); // signed short
	dig_P7 = buf[i++] | (buf[i++]<<8); // signed short
	dig_P8 = buf[i++] | (buf[i++]<<8); // signed short
	dig_P9 = buf[i++] | (buf[i++]<<8); // signed short

	// 0xA0, undefined, unused, skip this byte
	i++;

	// 0xA1
	dig_H1 = buf[i++]; 									// unsigned char
	dig_H2 = buf[i++] | (buf[i++]<<8); // signed short
	dig_H3 = buf[i++];									// unsigned char

	// wtf?! notice we dont increment i as we will need same byte in next step
	dig_H4 = (buf[i++]<<4) | (buf[i] & 0b00001111);  // signed short
	dig_H5 = (buf[i++] & 0b11110000) | (buf[i++]<<4);
	dig_H6 = buf[i++]; // signed char
}

void BME280::read_adcs() {
	uint8_t num_bytes = 8;

	Wire.beginTransmission(BME280_ADDRESS);
	Wire.write(BME280_ADD_ADC_PRESS);
	Wire.endTransmission();
	Wire.requestFrom(BME280_ADDRESS, num_bytes);

	// 0xF7...0xFE
	// [0:2] temp, [3:5] press, [6:7] hum
	uint8_t buf[num_bytes];
	for(int i = 0; i < num_bytes; i++) {
		buf[i] = Wire.read();
	}

	adc_P = ((buf[0] << 16) + (buf[1] << 8) + buf[2]) >> 4;
	adc_T = ((buf[3] << 16) + (buf[4] << 8) + buf[5]) >> 4;
	adc_H = (buf[6] << 8) + buf[7];
}

// Datasheet p27
void BME280::set_mode(uint8_t osrs_h, uint8_t osrs_t, uint8_t osrs_p, uint8_t mode) {
	Wire.beginTransmission(BME280_ADDRESS);
	Wire.write(BME280_ADD_CTRL_HUM); // changes to this register only become active after write to ctrl_meas register !!
	Wire.write(osrs_h);
	Wire.endTransmission();

	uint8_t ctrl_meas = (osrs_t<<5) | (osrs_p<<2) | mode;

	Wire.beginTransmission(BME280_ADDRESS);
	Wire.write(BME280_ADD_CTRL_MEAS);
	Wire.write(ctrl_meas);
	Wire.endTransmission();
}

void BME280::print_calibration() {
	String s = "Calibration:";
	s += "\nH1: "; s += dig_H1;
	s += "\nH2: "; s += dig_H2;
	s += "\nH3: "; s += dig_H3;
	s += "\nH4: "; s += dig_H4;
	s += "\nH5: "; s += dig_H5;
	s += "\nH6: "; s += dig_H6;

	s += "\nT1: "; s += dig_T1;
	s += "\nT2: "; s += dig_T2;
	s += "\nT3: "; s += dig_T3;

	s += "\nP1: "; s += dig_P1;
	s += "\nP2: "; s += dig_P2;
	s += "\nP3: "; s += dig_P3;
	s += "\nP4: "; s += dig_P4;
	s += "\nP5: "; s += dig_P5;
	s += "\nP6: "; s += dig_P6;
	s += "\nP7: "; s += dig_P7;
	s += "\nP8: "; s += dig_P8;
	s += "\nP9: "; s += dig_P9;

	Serial.println(s);
}

