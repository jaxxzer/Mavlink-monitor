#ifndef PARAM_H
#define PARAM_H

#include <Arduino.h>  // for type definitions
#include "SUB/ardupilotmega/mavlink.h"
#include <EEPROM.h>

#define MAX_PARAMS 10
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
    char                id[16];
    uint32_t            address;
    MAV_PARAM_TYPE      type;
    float*              value;
} param_t;

class Parameters {
  public:
    Parameters();
    
    param_t* add(char* id, float* var);
    param_t* add(char* id, uint32_t* var);
    
    void load_all(void);
    
    void save(param_t param);
    void save(float* var);

    param_t* find(char *id);
    param_t* get(uint8_t index);
    param_t* set(char* id, float value);

    uint8_t num_params(void) const { return _n; };
    
  private:
    param_t _params[MAX_PARAMS];
    uint8_t    _n;
    
    
};

#endif
