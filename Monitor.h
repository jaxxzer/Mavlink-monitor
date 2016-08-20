#ifndef MONITOR_H
#define MONITOR_H

#include <EEPROM.h>
#include <Arduino.h>
#include "MapleMini.h"
#include "Mavlink.h"
#include "Param.h"

    
#define CELL_VMAX 4200.0
#define CELL_VMIN 3500.0

class Monitor {
  public:

    Monitor();
    Parameters params;
    //Mavlink pixhawk;
    
    void run(void);
    void init(void);


    ///////////////////////////
    ///////PARAMETERS//////////
    ///////////////////////////
    float VSCALE;
    float CSCALE;
    uint32_t SRATE1;
    uint32_t SRATE2;
    uint32_t BAUD_PIX;
    uint32_t BAUD_ESP;
    uint32_t BAUD_232;
    uint32_t test;
    uint16_t fart;
  
    ///////////////////////////
    /////Scheduling////////////
    ///////////////////////////
    
    uint16_t looptime; // performance
    uint32_t lastus; // performance
    
    uint32_t last30s;
    uint32_t last1Hz;
    uint32_t last5Hz;
    uint32_t last10Hz;
    uint32_t last50Hz;
    uint32_t lastS1;
    uint32_t lastS2;

    uint16_t measureVoltage(void);
    uint16_t measureCurrent(void);
    
    
    
    ///////////////////////////
    /////BATTERY MANAGEMENT////
    ///////////////////////////

    
    uint8_t cells;
    

    
    ///////////////////////////
    ///////////////////////////
    ///////////////////////////
    
    float range;
    bool water;
    
  private:
  

  
};

#endif
