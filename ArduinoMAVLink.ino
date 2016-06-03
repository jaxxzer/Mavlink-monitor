// Arduino MAVLink test code.
#include "mavlink.h"// Mavlink interface
#include "protocol.h"
#include "mavlink_helpers.h"


#include <EEPROM.h>
#include <Arduino.h>  // for type definitions

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



#define ADC_VOLTAGE A0
#define ADC_CURRENT A1

#define SYSID 3
#define COMPID 1


#define ADD_VSCALE 0 * sizeof(float)
#define ADD_CSCALE 1 * sizeof(float)

#define ID_VSCALE 0
#define ID_CSCALE 1

#define NUM_PARAMS 2

uint16_t looptime = 0;
uint32_t last1Hz = 0;
uint32_t last5Hz = 200;
uint32_t last10Hz = 400;

uint8_t cells = 3;
float VSCALE = 1580;
float CSCALE = 1;

#define CELL_VMAX 4200.0
#define CELL_VMIN 3500.0


void setup() {
  EEPROM_readAnything(ADD_VSCALE, VSCALE);
  EEPROM_readAnything(ADD_CSCALE, CSCALE);

  Serial.begin(115200);
  
  for(int i = 0; i < 5; i++) {
    send_params();
    delay(20);
  }
  
  uint16_t voltage = measureVoltage();
  if(voltage < 9000) {
    cells = 2;
  } else if(voltage < 13000) {
    cells = 3;
  } else {
    cells = 4;
  }

  send_text("Online");

  char buf[10];
  itoa(cells, buf, 10);
  send_text(buf);

  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, HIGH);

}

void loop() {
  uint32_t tnowus = micros();
  uint32_t tnow = millis();

  // 1Hz loop
  if(tnow - last1Hz > 1000) {
    last1Hz = tnow;

    send_heartbeat();
    send_system_status(); 
  }

  // 5Hz loop
  if(tnow - last5Hz > 5000) {
    last5Hz = tnow;
    
  }

  // 10Hz loop
  if(tnow - last10Hz > 10000) {
    last10Hz = tnow;
    
  }
  
  comm_receive();

  looptime = micros() - tnowus;
}

void comm_receive() { 
	mavlink_message_t msg; 
	mavlink_status_t status;
	
	//receive data over Serial 
	while(Serial.available() > 0) { 
		uint8_t c = Serial.read();

		//try to get a new message 
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
      //Got a valid message
      
			// Handle message
 			switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          if(msg.sysid == 252) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
          }
        	
        } break;
        
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
          send_params();
        } break;
  
        //https://pixhawk.ethz.ch/mavlink/#PARAM_SET
        case MAVLINK_MSG_ID_PARAM_SET: {
  
          mavlink_param_set_t in;
          mavlink_msg_param_set_decode(&msg, &in);
          if(in.target_system == SYSID && in.target_component == COMPID) {
            
            if(strncmp("VSCALE", in.param_id, 6) == 0) {
              digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
              VSCALE = in.param_value;
              EEPROM_writeAnything(ADD_VSCALE, VSCALE);
            } else if(strncmp("CSCALE", in.param_id, 6) == 0) {
              CSCALE = in.param_value;
              EEPROM_writeAnything(ADD_CSCALE, CSCALE);
            }
          }
        } break;  
			}
		} 
		// And get the next one
	}
}

uint16_t measureVoltage() {
  float v = analogRead(ADC_VOLTAGE);
  v = analogRead(ADC_VOLTAGE);
  v = v * VSCALE * 10 / 360;
  return (uint16_t)v;
}
uint16_t measureCurrent() {
  float c = analogRead(ADC_CURRENT);
  c = analogRead(ADC_CURRENT);
  c = c * CSCALE * 10 / 360;
  return (int16_t)c;
}

void send_heartbeat() {
  ////////////////////
  //Heartbeat
  //////////////////////

  // Define the system type (see mavlink_types.h for list of possible types) 
  int system_type = MAV_TYPE_SUBMARINE;
  int autopilot_type = MAV_AUTOPILOT_GENERIC;
  
  // Initialize the required buffers 
  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  //mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC);
  mavlink_msg_heartbeat_pack(SYSID, COMPID, &msg, system_type, autopilot_type, 0, 0, 0);

//  static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//                   uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
  
  // Copy the message to send buffer 
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes) 
  Serial.write(buf, len);
}

void send_system_status() {
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
  
  uint16_t voltage = measureVoltage();
  delay(1); // Delay for ADC recovery
  uint16_t current = measureCurrent();
      
  float percent_remaining = (100 * (voltage - (cells * CELL_VMIN))) / (cells * (CELL_VMAX - CELL_VMIN));
  
  mavlink_msg_sys_status_pack(SYSID, COMPID, &msg,
                   1, 1, 
                   1, 1, voltage, current,
                   (int8_t)percent_remaining, 0, 0, 0,
                   0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void send_params() {
  //  static inline uint16_t mavlink_msg_param_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  //                   const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;

  mavlink_msg_param_value_pack(SYSID, COMPID, &msg, "VSCALE", VSCALE, MAV_PARAM_TYPE_REAL32, NUM_PARAMS, ID_VSCALE);
  len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial.write(buf, len);

  
  mavlink_msg_param_value_pack(SYSID, COMPID, &msg, "CSCALE", CSCALE, MAV_PARAM_TYPE_REAL32, NUM_PARAMS, ID_CSCALE);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Serial.write(buf, len);

  
}

void send_text(char* text) {

  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_statustext_pack(SYSID, COMPID, &msg, MAV_SEVERITY_INFO, text);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial.write(buf, len);
}

void send_battery_status() {
  ////////////////////
  //Battery status 
  //////////////////////
  
  //static inline uint16_t mavlink_msg_battery_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  //                 uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t *voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining)

  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t voltages[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  
  mavlink_msg_battery_status_pack(SYSID, COMPID, &msg, 1, 1, 1, 1, voltages, 1, 1, 1, 1);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void send_power_status() {
  ////////////////////
  //Power Status
  //////////////////////

  //Arduino/ArduinoMAVLink/common/mavlink_msg_power_status.h

  //static inline uint16_t mavlink_msg_power_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  //                 uint16_t Vcc, uint16_t Vservo, uint16_t flags)

  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_power_status_pack(2, 1, &msg,
                   1254, 59, 4);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

