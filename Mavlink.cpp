#include "Mavlink.h"
#include "Monitor.h"

#define MAVLINK_DEBUG 1

extern Monitor monitor;

Mavlink::Mavlink(uint8_t sysid, uint8_t compid, Stream *port, uint8_t channel) :
		status(STATUS_NOT_CONNECTED),
		last_master_recv_ms(0),
		_sysid(sysid),
		_compid(compid),
		_port(port),
		_channel(channel),
		master_time_offset(0),
		system_type(MAV_TYPE_MONITOR),
		autopilot_type(MAV_AUTOPILOT_INVALID),
		last_master_time_request_ms(0)
{}

void Mavlink::init_params(Parameters *_params) {
	params = _params;
	if(_params == NULL) {
		Serial.println("Null params!");
	}
}

void Mavlink::init() {

}

void Mavlink::update(void) {
	uint32_t tnow = millis();

	comm_receive();

	if(last_master_recv_ms == 0)
		return;

	// request time at 1hz until we have synchronized with master
	// only use with pixhawk at {sysid,compid} = {1,1}
	if(master_time_offset == 0 && tnow > last_master_time_request_ms + 1000) {
		last_master_time_request_ms = tnow;
		send_request_data_stream(MAV_DATA_STREAM_EXTRA3, 1, 1);
	}

	if(tnow > last_master_recv_ms + LINK_TIMEOUT_MS) {
		if(status == STATUS_CONNECTED)
			status = STATUS_CONNECTION_LOST;
			master_time_offset = 0;
		return;
	}

	if(status != STATUS_CONNECTED) {
		status = STATUS_CONNECTED;
	}
}

void Mavlink::send_heartbeat() {
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

void Mavlink::send_system_status(uint16_t looptime) {
	//  static inline uint16_t mavlink_msg_sys_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
	//                   uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled,
	//                   uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, int16_t current_battery,
	//                   int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1,
	//                   uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4)
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	uint32_t MAV_SENSOR_WATER =		0x20000000;
	uint32_t MAV_SENSOR_TEMP =		0x40000000;
	uint32_t MAV_SENSOR_HUMIDITY =	0x80000000;
	uint32_t MAV_SENSOR_PRESSURE =	0x08;

	uint32_t sensors_enabled = 0;

	if(monitor.waterdetector.W_ENABLE) {
		sensors_enabled |= MAV_SENSOR_WATER;
	}

	if(monitor.tempsensor.T_ENABLE) {
		sensors_enabled |= MAV_SENSOR_TEMP;
	}

	if(monitor.bme.BME_ENABLE) {
		sensors_enabled |= MAV_SENSOR_TEMP;
		sensors_enabled |= MAV_SENSOR_HUMIDITY;
		sensors_enabled |= MAV_SENSOR_PRESSURE;
	}

	uint32_t sensors_present = sensors_enabled;

	uint16_t tempsensor_temp = 0;


	if(monitor.tempsensor.T_LPF_ENABLE) {
		tempsensor_temp = monitor.tempsensor.temperature_filt.get();
	} else {
		tempsensor_temp = monitor.tempsensor.temperature;
	}


	uint32_t sensors_health = 0;
	sensors_health |= !monitor.waterdetector.detected * MAV_SENSOR_WATER;
	sensors_health |= ((tempsensor_temp < monitor.tempsensor.T_LIMIT) &&
		(monitor.bme.temperature * 100.0f < monitor.tempsensor.T_LIMIT)) * MAV_SENSOR_TEMP;
	sensors_health |= (monitor.bme.humidity < 80) * MAV_SENSOR_HUMIDITY;
	sensors_health |= (monitor.bme.pressure < 104000) * MAV_SENSOR_PRESSURE;

	mavlink_msg_sys_status_pack(_sysid, _compid, &msg,
			sensors_present,
			sensors_enabled,
			sensors_health,
			looptime,
			monitor.battery.voltage_filt.get(), // voltage in millivolts
			monitor.battery.current_filt.get() / 10.0f, // current in centiAmps
			monitor.battery.remaining(),
			0, 0,
			monitor.bme.humidity * 100.0f,
			monitor.bme.pressure / 100.0f,
			monitor.bme.temperature * 100.0f,
			tempsensor_temp);
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
#if MAVLINK_DEBUG
	Serial.print("Sending param: ");
	Serial.println(param->id);
#endif

	mavlink_msg_param_value_pack(_sysid, _compid, &msg, param->id, *(param->value), param->type, params->num_params(), param->index);
	len = mavlink_msg_to_send_buffer(buf, &msg);

	_port->write(buf, len);
}

void Mavlink::send_text(const char* text) {

	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_statustext_pack(_sysid, _compid, &msg, MAV_SEVERITY_INFO, text);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	_port->write(buf, len);
}

void Mavlink::send_battery_status() {
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

	//static inline uint16_t mavlink_msg_distance_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
	//                   uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance)

	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	uint32_t master_time = millis() + master_time_offset;

	mavlink_msg_distance_sensor_pack(_sysid, _compid, &msg,
			master_time, 30, distance_cm_filt, distance_cm, MAV_DISTANCE_SENSOR_ULTRASOUND, 1, MAV_SENSOR_ROTATION_PITCH_90, 0);

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	_port->write(buf, len);
}

// Send pixhawk request to stop sending extraneous messages intended for a GCS
void Mavlink::send_request_data_stream(MAV_DATA_STREAM stream_id, uint16_t rate, uint8_t start_stop) {

	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	//static inline uint16_t mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
	//                   uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)


	mavlink_msg_request_data_stream_pack(_sysid, _compid, &msg,
			1, 1, stream_id, rate, start_stop);

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
			if(msg.sysid == _sysid && msg.compid == _compid) {
				return; // handle local loopback
			}
			monitor.notify.blink(LED_3);
			// Accept messages from pixhawk {1,1} or from esp {_sysid,2} on same system
			if(msg.sysid == 1 && msg.compid == 1 ||
					msg.sysid == _sysid)
				last_master_recv_ms = millis();
#if MAVLINK_DEBUG
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

				uint32_t master_time = in.time_boot_ms;
				master_time_offset = master_time - millis();
			}
			// EXTRA1
			case MAVLINK_MSG_ID_ATTITUDE:
			case MAVLINK_MSG_ID_AHRS2:
			case MAVLINK_MSG_ID_AHRS3:
			// EXTRA2
			case MAVLINK_MSG_ID_VFR_HUD:
			// EXTRA3
			case MAVLINK_MSG_ID_VIBRATION:
			case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
			case MAVLINK_MSG_ID_RANGEFINDER:
			case MAVLINK_MSG_ID_HWSTATUS:
			case MAVLINK_MSG_ID_AHRS:
			// EXT_STAT
			case MAVLINK_MSG_ID_SYS_STATUS:
			case MAVLINK_MSG_ID_MEMINFO:
			case MAVLINK_MSG_ID_MISSION_CURRENT:
			case MAVLINK_MSG_ID_GPS_RAW_INT:
			case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
			case MAVLINK_MSG_ID_LIMITS_STATUS:
			// POSITION
			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			// RAW_SENS
			case MAVLINK_MSG_ID_SCALED_PRESSURE2:
			case MAVLINK_MSG_ID_SCALED_IMU2:
			case MAVLINK_MSG_ID_SCALED_PRESSURE:
			case MAVLINK_MSG_ID_RAW_IMU:
			// RC_CHAN:
			case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
			case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
				// Send request to stop all streams from pixhawk
				send_request_data_stream(MAV_DATA_STREAM_ALL, 0, 0);
				break;

			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
				send_params();
			} break;

			//https://pixhawk.ethz.ch/mavlink/#PARAM_SET
			case MAVLINK_MSG_ID_PARAM_SET: {
				mavlink_param_set_t in;
				mavlink_msg_param_set_decode(&msg, &in);
#if MAVLINK_DEBUG
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
#if MAVLINK_DEBUG
					Serial.print("SET PARAM: ");
					Serial.print(in.param_id);
					Serial.print(" TYPE: ");
					Serial.println(in.param_type);
#endif
					param_t* param = params->set(in.param_id, in.param_value);
					if(*param->value != in.param_value) { // Submitted value was out of range and had to be constrained
						send_text("Invalid param value!");
					}
					send_param(param->index); // Acknowledge request by sending updated parameter value
				}
			} break;

			case MAVLINK_MSG_ID_STATUSTEXT: {
				mavlink_statustext_t in;
				mavlink_msg_statustext_decode(&msg, &in);
#if MAVLINK_DEBUG
				Serial.println(in.text);
#endif

			} break;

			case MAVLINK_MSG_ID_PARAM_VALUE: {
				mavlink_param_value_t in;
				mavlink_msg_param_value_decode(&msg, &in);
#if MAVLINK_DEBUG
				Serial.println(in.param_id);
#endif
			} break;

			case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
				send_mission_count(msg.sysid, msg.compid);
			} break;

			case MAVLINK_MSG_ID_COMMAND_LONG: {
				mavlink_command_long_t in;
				mavlink_msg_command_long_decode(&msg, &in);
#if MAVLINK_DEBUG
				Serial.print("command# ");
				Serial.println(in.command);
#endif
			}
			}
		}
		// And get the next one
	}
}
