#ifndef MAVLINK_MONITOR_H
#define MAVLINK_MONITOR_H

#include "Param.h"
#include "Notify.h"
#include "SUB/ardupilotmega/mavlink.h"

#define LINK_TIMEOUT_MS 2000

class Mavlink {
  friend class Monitor;
  public:
    Mavlink(uint8_t sysid, uint8_t compid, Stream *port, uint8_t channel);
    void init_params(Parameters *_params);
    void init();
    void update(void);
    
    void send_heartbeat(void);
    void send_system_status(uint16_t looptime);
    void send_params(void);
    void send_param(uint8_t index);
    void send_text(const char* text);
    void send_battery_status(void);
    void send_power_status(void);
    void send_distance_sensor(uint16_t distance_cm, uint16_t distance_cm_filt);
    void send_request_data_stream(MAV_DATA_STREAM stream_id, uint16_t rate, uint8_t start_stop);
    void send_mission_count(uint8_t target_system, uint8_t target_component);
    void comm_receive(void);
    void send_nav_cmd_do_trigger_control(uint32_t pic_interval_ms);
    void send_nav_cmd_preflight_reboot_shutdown(void);
    void set_port(Stream *port) { _port = port; };
    void set_mav_type(MAV_TYPE type) { system_type = type; };
    MAV_TYPE system_type = MAV_TYPE_MONITOR;
    int autopilot_type = MAV_AUTOPILOT_INVALID;

    Parameters* params;

    conn_status_t status;

  private: 
    int32_t master_time_offset;
    uint32_t last_master_recv_ms;
    uint8_t _sysid;
    uint8_t _compid;
    Stream *_port;
    uint8_t _channel;
    uint32_t last_master_time_request_ms;
    uint32_t last_master_time_sync_ms;

};

#endif
