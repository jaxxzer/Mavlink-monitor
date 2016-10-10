#include "Param.h"
#define PARAM_DEBUG 0
Parameters::Parameters() :
_n(0)
{
	memset(_params, 0, sizeof(_params));
}

void Parameters::load_all()
{
	for(int i = 0; i < _n; i++) {
		EEPROM_readAnything(_params[i].address, *_params[i].value);
		
		if(!constrain_param(i)) {

			switch(_params[i].type) {
			case MAV_PARAM_TYPE_INT8:
				set_default<int8_t>(i);
				break;
			case MAV_PARAM_TYPE_INT16:
				set_default<int16_t>(i);
				break;
			case MAV_PARAM_TYPE_INT32:
				set_default<int32_t>(i);
				break;
			case MAV_PARAM_TYPE_UINT8:
				set_default<uint8_t>(i);
				break;
			case MAV_PARAM_TYPE_UINT16:
				set_default<uint16_t>(i);
				break;
			case MAV_PARAM_TYPE_UINT32:
				set_default<uint32_t>(i);
				break;
			case MAV_PARAM_TYPE_REAL32:
				set_default<float>(i);
				break;
			default:
				break;
			}

			constrain_param(i); // Sanity check
		}

#if PARAM_DEBUG
		Serial.print("Loaded parameter");
		Serial.print("\tID: ");
		Serial.print(_params[i].id);
		Serial.print("\tIndex: ");
		Serial.print(_params[i].index);
		Serial.print("\tValue: ");
		if(_params[i].type == MAV_PARAM_TYPE_UINT32)
			Serial.print(*(uint32_t*)_params[i].value);
		else
			Serial.print(*_params[i].value);
		Serial.print("\tmin: ");
		Serial.print(_params[i].min);
		Serial.print("\tmax: ");
		Serial.println(_params[i].max);
#endif
	}
}

void Parameters::save(param_t param)
{
	EEPROM_writeAnything(param.address, *param.value);

}

void Parameters::save(float* var)
{
	for(int i = 0; i < _n; i++) {
		if(_params[i].value == var) {
			EEPROM_writeAnything(_params[i].address, *_params[i].value);
		}
	}

}

param_t* Parameters::find(char* id) {

	char key[PARAM_NAME_MAX + 1];
	strncpy(key, id, PARAM_NAME_MAX);
	key[PARAM_NAME_MAX] = 0; //add null terminator

	for(int i = 0; i < _n; i++) {
#if PARAM_DEBUG
		Serial.print("Seaching: ");
		Serial.print(key);
		Serial.print(" against: ");
		Serial.println(_params[i].id);
#endif

		if(strncmp(_params[i].id, key, PARAM_NAME_MAX) == 0) {
#if PARAM_DEBUG
			Serial.print("MATCH index: ");
			Serial.println(i);
#endif
			return &_params[i];
		}

	}
#if PARAM_DEBUG
	Serial.println("NOMATCH");
#endif
	return NULL;
}

param_t* Parameters::set(char* id, float value) {
	param_t* param = find(id);

	if(param == NULL)
		return NULL;

	*(param->value) = value;
	constrain_param(param->index);
	save(*param);

#if PARAM_DEBUG
	Serial.print("PARAM ");
	Serial.print(id);
	if(param->type == MAV_PARAM_TYPE_UINT32)
		Serial.println(*((uint32_t*)param->value));
	else
		Serial.println(*(param->value));
#endif
	return param;
}

param_t* Parameters::get(uint8_t index) {
	if(index >= _n) {
		return NULL;
	}
	return &_params[index];

}

// Constrain parameter value to min/max range
// Return true if param value within correct range
// Return false if param value had to be constrained
bool Parameters::constrain_param(uint8_t index)	{
	if(index < 0 || index > _n-1) {
		return false; // out of range
	}
	switch(_params[index].type) {
	case MAV_PARAM_TYPE_INT8:
		return constrain_t<int8_t>(index);
		break;
	case MAV_PARAM_TYPE_INT16:
		return constrain_t<int16_t>(index);
		break;
	case MAV_PARAM_TYPE_INT32:
		return constrain_t<int32_t>(index);
		break;
	case MAV_PARAM_TYPE_UINT8:
		return constrain_t<uint8_t>(index);
		break;
	case MAV_PARAM_TYPE_UINT16:
		return constrain_t<uint16_t>(index);
		break;
	case MAV_PARAM_TYPE_UINT32:
		return constrain_t<uint32_t>(index);
		break;
	case MAV_PARAM_TYPE_REAL32:
		return constrain_t<float>(index);
		break;
	default:
		return false;
	}
}
