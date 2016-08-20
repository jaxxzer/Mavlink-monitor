#ifndef MAVLINK_MONITOR_H
#define MAVLINK_MONITOR_H

#include "Param.h"


#include "SUB/ardupilotmega/mavlink.h"

#define SYSID 2
#define COMPID 1


class Mavlink {
  public:
    Mavlink(Parameters* params);
    void send_heartbeat(void);
    void send_system_status(void);
    void send_params(void);
    void send_param(uint8_t index);
    void send_text(char* text);
    void send_battery_status(void);
    void send_power_status(void);
    void send_distance_sensor(uint16_t distance_cm);
    void send_request_data_stream(void);
    void send_mission_count(uint8_t target_system, uint8_t target_component);
    void comm_receive(void);
    Parameters* params;

  private: 
    uint32_t master_time;
};

#endif
