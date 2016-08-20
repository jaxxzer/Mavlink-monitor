#include "Param.h"

Parameters::Parameters() :
_n(0)
{
  memset(_params, 0, sizeof(_params));
}


param_t* Parameters::add(char* id, float* var)
{
  if(_n >= MAX_PARAMS) {
    return NULL;
  }
  
  strncpy(_params[_n].id, id, PARAM_NAME_MAX);
  _params[_n].id[PARAM_NAME_MAX] = 0; // add null terminator
  _params[_n].type = MAV_PARAM_TYPE_REAL32;
  _params[_n].address = _n * sizeof(float);
  _params[_n].index = _n;
  _params[_n].value = var;

  return &_params[_n++];
}

param_t* Parameters::add(char* id, uint32_t* var)
{
  if(_n >= MAX_PARAMS) {
    return NULL;
  }
  
  strncpy(_params[_n].id, id, PARAM_NAME_MAX);
  _params[_n].id[PARAM_NAME_MAX] = 0; // add null terminator
  _params[_n].type = MAV_PARAM_TYPE_UINT32;
  _params[_n].address = _n * sizeof(float);
  _params[_n].index = _n;
  _params[_n].value = (float*)var;

  return &_params[_n++];
}




void Parameters::load_all()
{
  for(int i = 0; i < _n; i++) {
    EEPROM_readAnything(_params[i].address, *_params[i].value);
    Serial.print("Loaded parameter");
    Serial.print(" ID: ");
    Serial.print(_params[i].id);
    Serial.print(" Index: ");
    Serial.print(_params[i].index);
    Serial.print(" Value: ");
    if(_params[i].type == MAV_PARAM_TYPE_UINT32)
      Serial.println(*(uint32_t*)_params[i].value);
    else
      Serial.println(*_params[i].value);
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
    Serial.print("Seaching: ");
    Serial.print(key);
    Serial.print(" against: ");
    Serial.println(_params[i].id);
    
    if(strncmp(_params[i].id, key, PARAM_NAME_MAX) == 0) {
      Serial.print("MATCH index: ");
      Serial.println(i);
      return &_params[i];
    }
    
  }
  
  Serial.println("NOMATCH");
  return NULL;
}

param_t* Parameters::set(char* id, float value) {
  param_t* param = find(id);
  
  if(param == NULL)
    return NULL;
    
  *(param->value) = value;
  save(*param);

  Serial.print("PARAM ");
  Serial.print(id);
  if(param->type == MAV_PARAM_TYPE_UINT32)
    Serial.println(*((uint32_t*)param->value));
  else
    Serial.println(*(param->value));
  return param;
}

param_t* Parameters::get(uint8_t index) {
  if(index >= _n) {
    return NULL;
  }
  return &_params[index];

}
