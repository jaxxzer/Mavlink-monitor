#ifndef PARAM_H
#define PARAM_H

#include <Arduino.h>  // for type definitions
#include "SUB/ardupilotmega/mavlink.h"
#include <EEPROM.h>

#define MAX_PARAMS 25
#define PARAM_NAME_MAX 16


template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
	const byte* p = (const byte*)(const void*)&value;
	unsigned int i;
	for (i = 0; i < sizeof(value); i++)
		EEPROM.write(ee++, *p++);
	return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
	byte* p = (byte*)(void*)&value;
	unsigned int i;
	for (i = 0; i < sizeof(value); i++)
		*p++ = EEPROM.read(ee++);
	return i;
}


typedef struct {
	uint8_t             index;
	char                id[PARAM_NAME_MAX + 1]; // Name
	uint32_t            address; // EEPROM address
	MAV_PARAM_TYPE      type; // should we treat this as a float or uint32_t?
	float*              value;
	float				min; // minimum valid value
	float				max; // maximum valid value
	float				def;
} param_t;

enum {
	//Monitor
	param_srate1,
	param_srate2,
	param_pic_interval,
	param_reboot_now = 3,
	param_vehicleid,

	//BattMonitor
	param_v_scale,
	param_c_scale,
	param_v_offset,
	param_c_offset,
	param_v_lpfcut,
	param_c_lpfcut,

	//Tempsensor
	param_t_enable,
	param_t_scale,
	param_t_offset,
	param_t_limit,
	param_t_cutoff,
	param_t_lpf_enable,

	//Rangefinder
	param_pingrate,
	param_range_enable,
	param_lpf_enable,
	param_lpf_cutoff,

	param_w_enable,

	param_bme_enable
};

class Parameters {
public:
	Parameters();

	template <typename T, MAV_PARAM_TYPE PT>
	param_t* add(uint8_t param, const char* id, T* var, T min, T max, T def);

	param_t* addFloat(uint8_t param, const char* id, float* var, float min, float max, float def) {
		return add<float, MAV_PARAM_TYPE_REAL32>(param, id, var, min, max, def);
	};
	param_t* addInt32(uint8_t param, const char* id, int32_t* var, int32_t min, int32_t max, int32_t def) {
		return add<int32_t, MAV_PARAM_TYPE_INT32>(param, id, var, min, max, def);
	};
	param_t* addUint32(uint8_t param, const char* id, uint32_t* var, uint32_t min, uint32_t max, uint32_t def) {
		return add<uint32_t, MAV_PARAM_TYPE_UINT32>(param, id, var, min, max, def);
	};
	param_t* addInt16(uint8_t param, const char* id, int16_t* var, int16_t min, int16_t max, int16_t def) {
		return add<int16_t, MAV_PARAM_TYPE_INT16>(param, id, var, min, max, def);
	};
	param_t* addUint16(uint8_t param, const char* id, uint16_t* var, uint16_t min, uint16_t max, uint16_t def) {
		return add<uint16_t, MAV_PARAM_TYPE_UINT16>(param, id, var, min, max, def);
	};
	param_t* addInt8(uint8_t param, const char* id, int8_t* var, int8_t min, int8_t max, int8_t def) {
		return add<int8_t, MAV_PARAM_TYPE_INT8>(param, id, var, min, max, def);
	};
	param_t* addUint8(uint8_t param, const char* id, uint8_t* var, uint8_t min, uint8_t max, uint8_t def) {
		return add<uint8_t, MAV_PARAM_TYPE_UINT8>(param, id, var, min, max, def);
	};

	void load_all(void);

	void save(param_t param);
	void save(float* var);

	param_t* find(char *id);
	param_t* get(uint8_t index);
	param_t* set(char* id, float value);

	bool constrain_param(uint8_t index);

	template <typename T>
	bool constrain_t(uint8_t index);

	template <typename T>
	void set_default(uint8_t index);

	uint8_t num_params(void) const { return _n; };

private:
	param_t _params[MAX_PARAMS];
	uint8_t    _n;


};

template <typename T, MAV_PARAM_TYPE PT>
param_t* Parameters::add(uint8_t param, const char* id, T* var, T min, T max, T def)
{
	if(_n >= MAX_PARAMS) {
		return NULL;
	}

	strncpy(_params[_n].id, id, PARAM_NAME_MAX);
	_params[_n].id[PARAM_NAME_MAX] = 0; // add null terminator
	_params[_n].type = PT;
	_params[_n].address = param * sizeof(float);
	_params[_n].index = _n;
	_params[_n].min = (float)min;
	_params[_n].max = (float)max;
	_params[_n].value = (float*)var;
	_params[_n].def = def;

	return &_params[_n++];
}

template <typename T>
bool Parameters::constrain_t(uint8_t index) {
	if(isnan(*(_params[index].value)) || isinf(*(_params[index].value))) {
		return false;
	} else if(*(T*)_params[index].value < (T)_params[index].min) {
		T min = (T)_params[index].min;
		*(_params[index].value) = *(float*)&min;
		return false;
	} else if(*(T*)_params[index].value > (T)_params[index].max) {
		T max = (T)_params[index].max;
		*(_params[index].value) = *(float*)&max;
		return false;
	}
	return true;
}

template <typename T>
void Parameters::set_default(uint8_t index) {
	T def = (T)_params[index].def;
	*(_params[index].value) = *(float*)&def;
}

#endif
