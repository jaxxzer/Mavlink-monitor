// Arduino MAVLink test code.
#ifdef __AVR__
  #include "avr/pgmspace.h"
#endif

#include "mavlink.h"// Mavlink interface
#include "protocol.h"
#include "mavlink_helpers.h"


#include <EEPROM.h>
#include <Arduino.h>  // for type definitions


typedef struct {
  char                id[16];
  uint8_t             index;
  uint8_t             ADDRESS;
  MAV_PARAM_TYPE      param_type = MAV_PARAM_TYPE_REAL32;
  float               value;
} Param;

typedef enum PARAMS {
  VMULT,
  CMULT
} PARAMS;


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

//Pin mappings
#ifdef __AVR__
  #define ADC_VOLTAGE A0
  #define ADC_CURRENT A1
  #define PIN_LED LED_BUILTIN
#else//STM32F103 devices
  #ifdef ESP8266
    #define ADC_VOLTAGE A0
    #define ADC_CURRENT A0
    #define PIN_LED LED_BUILTIN
  #else
    #define ADC_VOLTAGE PA1
    #define ADC_CURRENT PA0
    //#define PIN_LED PC13 Generic STMF103
    #define PIN_LED 33
  #endif
#endif

#define SYSID 2
#define COMPID 1


#define ADD_VSCALE 0 * sizeof(float)
#define ADD_CSCALE 1 * sizeof(float)
#define ADD_SRATE1 2 * sizeof(float)
#define ADD_SRATE2 3 * sizeof(float)

#define ID_VSCALE 0
#define ID_CSCALE 1

#define NUM_PARAMS 2

uint16_t looptime = 0;
uint32_t lastus = 0;
uint32_t last1Hz = 0;
uint32_t last5Hz = 200;
uint32_t last10Hz = 400;
uint32_t last50Hz = 600;

uint8_t cells = 3;
float VSCALE = 1580;
float CSCALE = 1;
uint8_t SRATE1 = 1;
uint8_t SRATE2 = 1;



Param V_MULT;
Param C_MULT;

#define CELL_VMAX 4200.0
#define CELL_VMIN 3500.0



float range = 0;

void setup() {

  EEPROM_readAnything(ADD_VSCALE, VSCALE);
  EEPROM_readAnything(ADD_CSCALE, CSCALE);
  EEPROM_readAnything(ADD_SRATE1, SRATE1);
  EEPROM_readAnything(ADD_SRATE2, SRATE2);

  //Serial.begin(115200);  //usb
  Serial1.begin(115200); //pixhawk
  //Serial2.begin(115200); //esp
  Serial3.begin(115200); //rs232
  
//  for(int i = 0; i < 5; i++) {
//    send_params();
//    delay(20);
//  }
  
  uint16_t voltage = measureVoltage();
  if(voltage < 9000) {
    cells = 2;
  } else if(voltage < 13000) {
    cells = 3;
  } else {
    cells = 4;
  }

  send_text("Online");

//  char buf[10];
//  itoa(cells, buf, 10);
//  send_text(buf);

  pinMode(PIN_LED, OUTPUT);     // Initialize the PIN_LED pin as an output
  digitalWrite(PIN_LED, HIGH);
}



void loop() {
  uint32_t tnowus = micros();
  uint32_t tnow = millis();
  
  looptime = tnowus - lastus;
  lastus = tnowus;


  // 1Hz loop
  if(tnow - last1Hz > 1000/1) {
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    last1Hz = tnow;

    send_heartbeat();
     
  }

  // 5Hz loop
  if(tnow - last5Hz > 1000/5) {
    last5Hz = tnow;
    send_system_status();
    send_distance_sensor(range*100);

    Serial3.write('Z');
    
 

    
  }

  // 10Hz loop
  if(tnow - last10Hz > 1000/10) {
    last10Hz = tnow;

    
  }

  if(tnow - last50Hz > 1000/50) {
    last50Hz = tnow;
  
  }

  range_receive();
  comm_receive();
  
}



void range_receive() {

  static char distance[20];
  static uint8_t index;

  while(Serial3.available() > 0) {
    uint8_t c = Serial3.read();
    switch(c) {
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
      case '.':
        if(index < 19)
          distance[index++] = c;
        break;
        
      case 'm':
        distance[index] = '\0';
        range = String(distance).toFloat();
      case '\r':
      case '\n':
        index = 0;
        break;

      default:
        range = 0;
        index = 0;
        break;
    }
  }
}

void comm_receive() { 
	mavlink_message_t msg; 
	mavlink_status_t status;
	
	//receive data over Serial 
	while(Serial1.available() > 0) { 
		uint8_t c = Serial1.read();

		//try to get a new message 
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
      //Got a valid message
      
			// Handle message
 			switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {

            digitalWrite(PIN_LED, !digitalRead(PIN_LED));

        	
        } break;
        
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
          send_params();
        } break;
  
        //https://pixhawk.ethz.ch/mavlink/#PARAM_SET
        case MAVLINK_MSG_ID_PARAM_SET: {
  
          mavlink_param_set_t in;
          mavlink_msg_param_set_decode(&msg, &in);
          if(in.target_system == SYSID && in.target_component == COMPID) {
            
            if(strncmp("VSCALE", in.param_id, 6) == 0) {
              digitalWrite(PIN_LED, !digitalRead(PIN_LED));
              VSCALE = in.param_value;
              EEPROM_writeAnything(ADD_VSCALE, VSCALE);
            } else if(strncmp("CSCALE", in.param_id, 6) == 0) {
              CSCALE = in.param_value;
              EEPROM_writeAnything(ADD_CSCALE, CSCALE);
            } else if(strncmp("SRATE1", in.param_id, 6) == 0) {
              SRATE1 = in.param_value;
              EEPROM_writeAnything(ADD_SRATE1, SRATE1);
            } else if(strncmp("SRATE2", in.param_id, 6) == 0) {
              SRATE2 = in.param_value;
              EEPROM_writeAnything(ADD_SRATE2, SRATE2);
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
  Serial1.write(buf, len);
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
  uint16_t current = measureCurrent();
      
  float percent_remaining = (100 * (voltage - (cells * CELL_VMIN))) / (cells * (CELL_VMAX - CELL_VMIN));
  
  mavlink_msg_sys_status_pack(SYSID, COMPID, &msg,
                   looptime, 1, 
                   1, looptime, voltage, current,
                   (int8_t)percent_remaining, looptime, 0, 0,
                   0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void send_params() {
  //  static inline uint16_t mavlink_msg_param_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  //                   const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;

  mavlink_msg_param_value_pack(SYSID, COMPID, &msg, "VSCALE", VSCALE, MAV_PARAM_TYPE_REAL32, NUM_PARAMS, ID_VSCALE);
  len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial1.write(buf, len);

  
  mavlink_msg_param_value_pack(SYSID, COMPID, &msg, "CSCALE", CSCALE, MAV_PARAM_TYPE_REAL32, NUM_PARAMS, ID_CSCALE);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Serial1.write(buf, len);

  
}

void send_text(char* text) {

  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_statustext_pack(SYSID, COMPID, &msg, MAV_SEVERITY_INFO, text);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial1.write(buf, len);
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
  Serial1.write(buf, len);
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
  
  mavlink_msg_power_status_pack(SYSID, COMPID, &msg,
                   1254, 59, 4);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void send_distance_sensor(uint16_t distance_cm) {
  ////////////////////
  //Power Status
  //////////////////////

  //Arduino/ArduinoMAVLink/common/mavlink_msg_distance_sensor.h

  //static inline uint16_t mavlink_msg_distance_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  //                   uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance)

  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_distance_sensor_pack(SYSID, COMPID, &msg,
                   0, 30, 500, distance_cm, MAV_DISTANCE_SENSOR_ULTRASOUND, 1, MAV_SENSOR_ROTATION_PITCH_90, 0);
                   
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}
