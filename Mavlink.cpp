#include "Mavlink.h"

Mavlink::Mavlink(uint8_t sysid, uint8_t compid, HardwareSerial *port) :
status(STATUS_NOT_CONNECTED),
last_master_recv_ms(0),
_sysid(sysid),
_compid(compid)
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
    send_request_data_stream();
  }


}

void Mavlink::send_heartbeat() {
  ////////////////////
  //Heartbeat
  //////////////////////
  Serial.println("Sending heartbeat");
  // Define the system type (see mavlink_types.h for list of possible types) 
  int system_type = MAV_TYPE_MONITOR;
  int autopilot_type = MAV_AUTOPILOT_INVALID;
  
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
  Serial1.write(buf, len);
}

void Mavlink::send_system_status() {
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
  
//  uint16_t voltage = measureVoltage();
//  uint16_t current = measureCurrent();
      float percent_remaining = 0;
//  float percent_remaining = (100 * (voltage - (cells * CELL_VMIN))) / (cells * (CELL_VMAX - CELL_VMIN));
//  uint32_t water_detected = water?0x20000000:0x0;
//  digitalWrite(PIN_LED, water_detected > 0);
  mavlink_msg_sys_status_pack(_sysid, _compid, &msg,
                   0, 0x20000000, 
                   0, 0, 0, 0,
                   (int8_t)percent_remaining, 0, 0, 0,
                   0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void Mavlink::send_params() {
  //  static inline uint16_t mavlink_msg_param_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  //                   const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
//  mavlink_message_t msg; 
//  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//  uint16_t len;

  for(int i = 0; i < params->num_params(); i++) {
    send_param(i);
//    param_t* param = params->get(i);
//    Serial.print("Send type: ");
//    Serial.println(param->type);
//
//    mavlink_msg_param_value_pack(_sysid, _compid, &msg, param->id, *(param->value), param->type, params->num_params(), param->index);
//    len = mavlink_msg_to_send_buffer(buf, &msg);
//  
//    Serial1.write(buf, len);   
  }
}

void Mavlink::send_param(uint8_t index) {
  if(index > params->num_params())
    return;

  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  
  param_t* param = params->get(index);
  Serial.print("Send type: ");
  Serial.println(param->type);

  mavlink_msg_param_value_pack(_sysid, _compid, &msg, param->id, *(param->value), param->type, params->num_params(), param->index);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Serial1.write(buf, len);   
}

void Mavlink::send_text(char* text) {

  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_statustext_pack(_sysid, _compid, &msg, MAV_SEVERITY_INFO, text);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial1.write(buf, len);
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
  Serial1.write(buf, len);
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
  Serial1.write(buf, len);
}

void Mavlink::send_distance_sensor(uint16_t distance_cm) {
  ////////////////////
  //Power Status
  //////////////////////

  //Arduino/ArduinoMAVLink/common/mavlink_msg_distance_sensor.h

  //static inline uint16_t mavlink_msg_distance_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  //                   uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance)

  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_distance_sensor_pack(_sysid, _compid, &msg,
                   master_time, 30, 500, distance_cm, MAV_DISTANCE_SENSOR_ULTRASOUND, 1, MAV_SENSOR_ROTATION_PITCH_90, 0);
                   
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void Mavlink::send_request_data_stream() {

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

//static inline uint16_t mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//                   uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)


  mavlink_msg_request_data_stream_pack(_sysid, _compid, &msg,
                     1, 1, MAV_DATA_STREAM_ALL, 0, 0);
                     
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
                       
}

void Mavlink::send_mission_count(uint8_t target_system, uint8_t target_component) {
//  static inline uint16_t mavlink_msg_mission_count_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//                   uint8_t target_system, uint8_t target_component, uint16_t count)
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_count_pack(_sysid, _compid, &msg,
              target_system, target_component, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void Mavlink::comm_receive() { 
  mavlink_message_t msg; 
  mavlink_status_t status;
  
  //receive data over Serial 
  while(Serial1.available() > 0) { 
    uint8_t c = Serial1.read();

    //try to get a new message 
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
      if(msg.sysid == 1 && msg.compid == 1)
        last_master_recv_ms = millis();

//      if(connected == false) {
//        connected = true;
//        send_request_data_stream();
//      }
      //Got a valid message

      Serial.print("Got msg ");
      Serial.print(msg.msgid);                                   
      Serial.print(" from ");
      Serial.print(msg.sysid);
      Serial.print(" , ");
      Serial.println(msg.compid);

      
      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {

            //digitalWrite(PIN_LED, !digitalRead(PIN_LED));

          
        } break;

        case MAVLINK_MSG_ID_SYSTEM_TIME: {
          mavlink_system_time_t in;
          mavlink_msg_system_time_decode(&msg, &in);
 
          master_time = in.time_boot_ms;
        } break;
        
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
          send_params();
        } break;
  
        //https://pixhawk.ethz.ch/mavlink/#PARAM_SET
        case MAVLINK_MSG_ID_PARAM_SET: {
  
          mavlink_param_set_t in;
          mavlink_msg_param_set_decode(&msg, &in);
          if(in.target_system == _sysid && in.target_component == _compid) {
            Serial.print("SET PARAM: ");
            Serial.print(in.param_id);
            Serial.print(" TYPE: ");
            Serial.println(in.param_type);
            param_t* param = params->set(in.param_id, in.param_value);
            send_param(param->index);
          }
        } break;

        case MAVLINK_MSG_ID_STATUSTEXT: {
          mavlink_statustext_t in;
          mavlink_msg_statustext_decode(&msg, &in);
          
          Serial.println(in.text);
          
        } break;

        case MAVLINK_MSG_ID_PARAM_VALUE: {
          mavlink_param_value_t in;
          mavlink_msg_param_value_decode(&msg, &in);

          Serial.println(in.param_id);
        } break;
        
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
          send_mission_count(msg.sysid, msg.compid);
        } break;

        case MAVLINK_MSG_ID_COMMAND_LONG: {
          mavlink_command_long_t in;
          mavlink_msg_command_long_decode(&msg, &in);
          Serial.print("command# ");
          Serial.println(in.command);
        }
      }
    } 
    // And get the next one
  }
}
