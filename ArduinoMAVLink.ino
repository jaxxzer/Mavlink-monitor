// Arduino MAVLink test code.
#include "mavlink.h"// Mavlink interface
#include "protocol.h"

#include "mavlink_helpers.h"






void setup() {
	Serial.begin(115200);
}

void loop() { 
	// Define the system type (see mavlink_types.h for list of possible types) 
	int system_type = MAV_TYPE_QUADROTOR;
	int autopilot_type = MAV_AUTOPILOT_GENERIC;
	
	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	// Pack the message
	//mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC);
	mavlink_msg_heartbeat_pack(253, 111, &msg, system_type, autopilot_type, 0, 0, 0);

//  static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//                   uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
	
	// Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);

  ////////////////////
  //Battery status test
  //////////////////////



  //Arduino/ArduinoMAVLink/common


 //static inline uint16_t mavlink_msg_battery_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
 //                 uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t *voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining)

  uint16_t voltages[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  
  mavlink_msg_battery_status_pack(253, 111, &msg, 2, 1, 1, 1, voltages, 1, 1, 1, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);

  ////////////////////
  //Power Status test
  //////////////////////


  //Arduino/ArduinoMAVLink/common/mavlink_msg_power_status.h


  //static inline uint16_t mavlink_msg_power_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  //                 uint16_t Vcc, uint16_t Vservo, uint16_t flags)


  mavlink_msg_power_status_pack(253, 111, &msg,
                   12, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);


  
	//comm_receive();
  delay(1000);
}

float batt_measure() {
  return 11.2;
}

void comm_receive() { 
	mavlink_message_t msg; 
	mavlink_status_t status;
	
	//receive data over serial 
	while(Serial.available() > 0) { 
		uint8_t c = Serial.read();
		
		//try to get a new message 
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
			// Handle message
 			switch(msg.msgid) {
	        case MAVLINK_MSG_ID_HEARTBEAT: {
	        	if(msg.sysid == 253) {
              digitalWrite(13, !digitalRead(13));
	        	}
	        }
	        break;
  				default:
  					//Do nothing
  				break;
			}
		} 
		// And get the next one
	}
}


