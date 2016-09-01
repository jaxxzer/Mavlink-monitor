#ifndef MAVLINK_MONITOR_H
#define MAVLINK_MONITOR_H

#include "Param.h"
#include "Notify.h"
#include "SUB/ardupilotmega/mavlink.h"

#define LINK_TIMEOUT_MS 2000

class Mavlink {
  public:
    Mavlink(uint8_t sysid, uint8_t compid, HardwareSerial *port, uint8_t channel);

    void init(Parameters *_params);
    void update(void);
    
    void send_heartbeat(void);
    void send_system_status(uint16_t looptime, bool water_detected);
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

    conn_status_t status;

  private: 
    static uint32_t master_time;
    uint32_t last_master_recv_ms;
    uint8_t _sysid;
    uint8_t _compid;
    HardwareSerial *_port;
    uint8_t _channel;

};

#endif
