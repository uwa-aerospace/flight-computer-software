#ifndef _LOGGER_h
#define _LOGGER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Commands.h"
#include <SoftwareSerial.h>
#define DEFAULT_LOG_PIN  3;

class FlightLogger {
public:
		FlightLogger();
		FlightLogger(boolean);
		void write_to_log(long, String);
		void write_to_coms(long, int, int, int, const char*);
private:
	boolean diagnostics_log = false;
	int card_pin = DEFAULT_LOG_PIN;
	char log_file_name[10];
	void write(char*, long);
};

#endif



