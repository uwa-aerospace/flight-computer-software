#ifndef _FLIGHTHANDLER_h
#define _FLIGHTHANDLER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define BASE_HEATBEAT_RATE		1000L
#define LAUNCH_HEARTBEAT_RATE	500L
#define RECOVER_HEARTBEAT_RATE	3000L
#define BASE_DATA_RATE			99999999999L
#define LAUNCH_DATA_RATE		100L
#define PRM_TO_SEC_DELAY		2000L
#define MAIN_TO_DROG_DELAY		10000L


#include "FlightLogger.h"
#include "Calculations.h"

class FlightHandler {
public:
	Status *status;
	Calculations *calculator;
	FlightLogger *logger;
	FlightHandler(Status *, Calculations *, FlightLogger *, long);
  void send_calculations();
	void update(float, float, float, float);
	void reset();
	void set_status(int);
	void set_gnd_ready_flag();
	void handle_diagnostic_data(String*);
  float starting_height = -999.99;
	//Include velocity, acceleration, and altitude data.
private:
	Calculations get_calculations_module();
	boolean gnd_ready_flag = false;
 
	void update_parachutes();
	void update_apogee_times();
	long current_time; //Time since start-up
	long launch_time = 0;  //Time since launch
	long calc_time_to_apogee;
	long est_time_to_apogee;
	long primary_to_secondary_delay = 2000;
	long drouge_to_main_delay = 8000;

	long data_rate = LAUNCH_DATA_RATE;
	long heart_beat_rate = BASE_HEATBEAT_RATE;
};

#endif



