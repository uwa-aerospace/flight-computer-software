#include "FlightLogger.h"

FlightLogger::FlightLogger() {
	card_pin = DEFAULT_LOG_PIN;
	pinMode(card_pin, OUTPUT);
}


FlightLogger::FlightLogger(boolean b) {
	card_pin = DEFAULT_LOG_PIN;
	diagnostics_log = b;
}

void FlightLogger::write_to_log(long event_time, String event_data) {
	if (diagnostics_log) {
		//Serial.println("DIAGNOSTICS: \tTime: " + String(event_time) + String("\tOutput: ") + event_data);
	} else {
    const char * data = event_data.c_str();
    char send_buffer[200];
    sprintf(send_buffer,"%lu,%s", (uint32_t)event_time, data);
    Serial.println(send_buffer);
    Serial.flush();
	}
}

void FlightLogger::write_to_coms(long event_time, int msg_type, int sequence, int status, const char* event_data) {
  if (diagnostics_log) {
    
  } else {
    char send_buffer[300];
    sprintf(send_buffer, "%lu,%i,%i,%i,%s", (uint32_t)event_time, msg_type, sequence, status, event_data);
    Serial1.println(send_buffer);
    Serial1.flush();
  }
}

void FlightLogger::write(char* data, long log_time) {
	//Interface with SPI
}


