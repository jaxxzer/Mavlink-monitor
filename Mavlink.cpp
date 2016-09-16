#include "Mavlink.h"
#define DEBUG_OUTPUT 1

Mavlink::Mavlink(uint8_t sysid, uint8_t compid, Stream *port, uint8_t channel) :
status(STATUS_NOT_CONNECTED),
last_master_recv_ms(0),
_sysid(sysid),
_compid(compid),
_port(port),
_channel(channel),
master_time(0),
system_type(MAV_TYPE_MONITOR),
autopilot_type(MAV_AUTOPILOT_INVALID)
{}

void Mavlink::init(Parameters *_params) {
  params = _params;

}

void Mavlink::update(void) {
  uint32_t tnow = millis();
  
  comm_receive();

  if(last_master_recv_ms == 0)
    return;
    
  if(tnow > last_master_recv_ms + LINK_TIMEOUT_MS) {
    if(status == STATUS_CONNECTED)
      status = STATUS_CONNECTION_LOST;
    return;
  }

  if(status != STATUS_CONNECTED) {
    status = STATUS_CONNECTED;
  }
}

void Mavlink::send_heartbeat() {
  ////////////////////
  //Heartbeat
  //////////////////////
  // Define the system type (see mavlink_types.h for list of possible types) 
  
#if DEBUG_OUTPUT
  //Serial.println("Sending Heartbeat");
#endif

  // Initialize the required buffers 
  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  //mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC);
  mavlink_msg_heartbeat_pack(_sysid, _compid, &msg, system_type, autopilot_type, 0, 0, 0);

//  static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//                   uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
  
  // Copy the message to send buffer 
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes) 
  _port->write(buf, len);
}

void Mavlink::send_system_status(uint16_t looptime, bool water_detected) {
  ////////////////////
  //System Status
  //////////////////////

  //https://pixhawk.ethz.ch/mavlink/#SYS_STATUS

//  static inline uint16_t mavlink_msg_sys_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//                   uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, 
//                   uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, int16_t current_battery, 
//                   int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, 
//                   uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4)
  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint32_t sensor_health = 0;
  sensor_health |= !water_detected * 0x20000000;
  
//  uint16_t voltage = measureVoltage();
//  uint16_t current = measureCurrent();
      float percent_remaining = 0;
//  float percent_remaining = (100 * (voltage - (cells * CELL_VMIN))) / (cells * (CELL_VMAX - CELL_VMIN));
  mavlink_msg_sys_status_pack(_sysid, _compid, &msg,
                   0, 0x20000000, 
                   sensor_health, looptime, 0, 0,
                   (int8_t)percent_remaining, 0, 0, 0,
                   0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  _port->write(buf, len);
}

void Mavlink::send_params() {
  for(int i = 0; i < params->num_params(); i++) {
    send_param(i);   
  }
}

void Mavlink::send_param(uint8_t index) {
  if(index > params->num_params())
    return;

  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  
  param_t* param = params->get(index);
#if DEBUG_OUTPUT
  Serial.print("Sending param: ");
  Serial.println(param->id);
#endif

  mavlink_msg_param_value_pack(_sysid, _compid, &msg, param->id, *(param->value), param->type, params->num_params(), param->index);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  
  _port->write(buf, len);   
}

void Mavlink::send_text(char* text) {

  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_statustext_pack(_sysid, _compid, &msg, MAV_SEVERITY_INFO, text);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  _port->write(buf, len);
}

void Mavlink::send_battery_status() {
  ////////////////////
  //Battery status 
  //////////////////////
  
  //static inline uint16_t mavlink_msg_battery_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  //                 uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t *voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining)

  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t voltages[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  
  mavlink_msg_battery_status_pack(_sysid, _compid, &msg, 1, 1, 1, 1, voltages, 1, 1, 1, 1);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  _port->write(buf, len);
}

void Mavlink::send_power_status() {
  ////////////////////
  //Power Status
  //////////////////////

  //Arduino/ArduinoMAVLink/common/mavlink_msg_power_status.h

  //static inline uint16_t mavlink_msg_power_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  //                 uint16_t Vcc, uint16_t Vservo, uint16_t flags)

  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_power_status_pack(_sysid, _compid, &msg,
                   1254, 59, 4);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  _port->write(buf, len);
}

void Mavlink::send_distance_sensor(uint16_t distance_cm, uint16_t distance_cm_filt) {
  ////////////////////
  //Power Status
  //////////////////////
#if DEBUG_OUTPUT
  //Serial.println("Sending range");
#endif
  //Arduino/ArduinoMAVLink/common/mavlink_msg_distance_sensor.h

  //static inline uint16_t mavlink_msg_distance_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  //                   uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance)

  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_distance_sensor_pack(_sysid, _compid, &msg,
                   master_time, 30, distance_cm_filt, distance_cm, MAV_DISTANCE_SENSOR_ULTRASOUND, 1, MAV_SENSOR_ROTATION_PITCH_90, 0);
                   
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  _port->write(buf, len);
}

// Send pixhawk request to stop sending extraneous messages intended for a GCS
void Mavlink::send_request_data_stream() {

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

//static inline uint16_t mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//                   uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)


  mavlink_msg_request_data_stream_pack(_sysid, _compid, &msg,
                     1, 1, MAV_DATA_STREAM_ALL, 0, 0);
                     
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  _port->write(buf, len);
                       
}

void Mavlink::send_mission_count(uint8_t target_system, uint8_t target_component) {
//  static inline uint16_t mavlink_msg_mission_count_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//                   uint8_t target_system, uint8_t target_component, uint16_t count)
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_count_pack(_sysid, _compid, &msg,
              target_system, target_component, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  _port->write(buf, len);
}

void Mavlink::send_nav_cmd_do_trigger_control(uint32_t pic_interval_ms) {
//	static inline uint16_t mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//							       uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)

	  mavlink_message_t msg;
	  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	  mavlink_msg_command_long_pack(_sysid, _compid, &msg,
	              0, 0, MAV_CMD_DO_TRIGGER_CONTROL,
				  0,
				  pic_interval_ms,
				  0.0,
				  0.0,
				  0.0,
				  0.0,
				  0.0,
				  0.0);

	  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	  _port->write(buf, len);
}

void Mavlink::send_nav_cmd_preflight_reboot_shutdown() {
//  static inline uint16_t mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//                     uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(_sysid, _compid, &msg,
                0, 0, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
          0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    _port->write(buf, len);
}

void Mavlink::comm_receive() { 
  mavlink_message_t msg; 
  mavlink_status_t status;
  
  //receive data over Serial 
  while(_port->available() > 0) { 
    uint8_t c = _port->read();

    //try to get a new message 
    if(mavlink_parse_char(_channel, c, &msg, &status)) { 
      // Accept messages from pixhawk {1,1} or from esp {_sysid,2} on same system
      if(msg.sysid == 1 && msg.compid == 1 ||
          msg.sysid == _sysid)
        last_master_recv_ms = millis();
#if DEBUG_OUTPUT
      //Got a valid message
      Serial.print("Got msg ");
      Serial.print(msg.msgid);                                   
      Serial.print(" from ");
      Serial.print(msg.sysid);
      Serial.print(" , ");
      Serial.println(msg.compid);
#endif
      
      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          
        } break;

        case MAVLINK_MSG_ID_SYSTEM_TIME: {
          mavlink_system_time_t in;
          mavlink_msg_system_time_decode(&msg, &in);
 
          master_time = in.time_boot_ms;
          send_request_data_stream();
        } break;
        
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
          send_params();
        } break;
  
        //https://pixhawk.ethz.ch/mavlink/#PARAM_SET
        case MAVLINK_MSG_ID_PARAM_SET: {

  
          mavlink_param_set_t in;
          mavlink_msg_param_set_decode(&msg, &in);
                    #if DEBUG_OUTPUT
          Serial.print("Got PARAM_SET: target=");
          Serial.print(in.target_system);
          Serial.print(":");
          Serial.println(in.target_component);
          Serial.print("I am: ");
          Serial.print(_sysid);
          Serial.print(":");
          Serial.print(_compid);
          #endif
          if(in.target_system == _sysid && in.target_component == _compid) {
#if DEBUG_OUTPUT
            Serial.print("SET PARAM: ");
            Serial.print(in.param_id);
            Serial.print(" TYPE: ");
            Serial.println(in.param_type);
#endif
            param_t* param = params->set(in.param_id, in.param_value);
            send_param(param->index); // Acknowledge request by sending updated parameter value
          }
        } break;

        case MAVLINK_MSG_ID_STATUSTEXT: {
          mavlink_statustext_t in;
          mavlink_msg_statustext_decode(&msg, &in);
#if DEBUG_OUTPUT
          Serial.println(in.text);
#endif
          
        } break;

        case MAVLINK_MSG_ID_PARAM_VALUE: {
          mavlink_param_value_t in;
          mavlink_msg_param_value_decode(&msg, &in);
#if DEBUG_OUTPUT
          Serial.println(in.param_id);
#endif
        } break;
        
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
          send_mission_count(msg.sysid, msg.compid);
        } break;

        case MAVLINK_MSG_ID_COMMAND_LONG: {
          mavlink_command_long_t in;
          mavlink_msg_command_long_decode(&msg, &in);
#if DEBUG_OUTPUT
          Serial.print("command# ");
          Serial.println(in.command);
#endif
        }
      }
    } 
    // And get the next one
  }
}
