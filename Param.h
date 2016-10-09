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

class Parameters {
public:
	Parameters();

	template <typename T, MAV_PARAM_TYPE PT>
	param_t* add(char* id, T* var, T min, T max, T def);
	param_t* add(char* id, float* var, float min, float max, float def);
	param_t* add(char* id, uint32_t* var, uint32_t min, uint32_t max, uint32_t def);

	param_t* addFloat(char* id, float* var, float min, float max, float def) {
		return add<float, MAV_PARAM_TYPE_REAL32>(id, var, min, max, def);
	};
	param_t* addUint32(char* id, uint32_t* var, uint32_t min, uint32_t max, uint32_t def) {
		return add<uint32_t, MAV_PARAM_TYPE_UINT32>(id, var, min, max, def);
	};
	param_t* addInt16(char* id, int16_t* var, int16_t min, int16_t max, int16_t def) {
		return add<int16_t, MAV_PARAM_TYPE_INT16>(id, var, min, max, def);
	};

	void load_all(void);

	void save(param_t param);
	void save(float* var);

	param_t* find(char *id);
	param_t* get(uint8_t index);
	param_t* set(char* id, float value);
	bool constrain_param(uint8_t index);

	uint8_t num_params(void) const { return _n; };

private:
	param_t _params[MAX_PARAMS];
	uint8_t    _n;


};

template <typename T, MAV_PARAM_TYPE PT>
param_t* Parameters::add(char* id, T* var, T min, T max, T def)
{
	if(_n >= MAX_PARAMS) {
		return NULL;
	}

	strncpy(_params[_n].id, id, PARAM_NAME_MAX);
	_params[_n].id[PARAM_NAME_MAX] = 0; // add null terminator
	_params[_n].type = PT;
	_params[_n].address = _n * sizeof(float);
	_params[_n].index = _n;
	_params[_n].min = min;
	_params[_n].max = max;
	_params[_n].value = (float*)var;
	_params[_n].def = def;

	return &_params[_n++];
}

#endif
