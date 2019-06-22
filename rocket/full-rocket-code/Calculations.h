#ifndef Calculations_h
#define Calculations_h

#include "Arduino.h"
#include "Status.h"

#define ATM_PRESSURE 1030 //Pressure in hPa

class Calculations {
	public:
		Calculations(int pin_accel, int pin_alti, int status_code);
		Calculations(int pin_accel, int status_code);
		void  update_calcs();
    void  update_calcs(float, float, float, float, int);
		void  update_time(long);
		long  time_to_apogee();
		float get_curr_acceleration();
		float get_prev_acceleration();
		float get_curr_acceleration_mag();
		float get_prev_acceleration_mag();
		float get_delta_a();
		float get_delta_a_mag();
		float get_curr_altitude();
		float get_prev_altitude();
		float get_delta_alti();
		float get_delta_alti_mag();
		float get_curr_temperature();
		float get_curr_velocity();
		float get_prev_velocity();
		float get_delta_v();
		float get_delta_v_mag();
		float get_delta_t();
		long  get_delta_t_L();
		long  get_curr_time();
		void  diagnostics_calculations(long, float);
		boolean in_diagnostics = false;
    // Min-Max Values
    float max_alti = -99999.0;
    float min_alti = 99999.0;
    float max_accel = -99999.0;
    float min_accel = 99999.0;
    float max_v = -99999.0;
    float min_v = 99999.0;
    float max_delta_alti = -99999.0;
    float min_delta_alti = 99999.0;
    float max_delta_a = -99999.0;
    float min_delta_a = 99999.0;
    float max_delta_v = -99999.0;
    float min_delta_v = 99999.0;
	private:
		// Private Functions
		float get_acceleration_mag(float x, float y, float z);

		// Raw Sensor Values
		float curr_alti = 0.0;
		float prev_alti = 0.0;
		float curr_accel = 0.0;
    float accel_x = 0.0;
    float accel_y = 0.0;
    float accel_z = 0.0;
		float prev_accel = 0.0;
		float curr_pressure = 100000.0;
		float prev_pressure = 100000.0;
    
		// Derived Values
    float velo_x = 0.0;
    float velo_y = 0.0;
    float velo_z = 0.0;
		float curr_velocity = 0.0;
		float prev_velocity = 0.0;

		// Time Based Derivatives
		float delta_alti = 0.0;	    // alt1 - alt0
		float delta_pressure = 0.0; //p2 - p1
		float delta_v = 0.0;	      // v - u
		float delta_a = 0.0;	      // a1 - a0
		long delta_t = 0.0;		      // t1 - t0

		// Values Determined by Derivatives
		boolean delta_v_is_pos = false;
		boolean delta_a_is_pos = false;
		boolean delta_alti_is_pos = false;

		// Time Spans for further derivations
		long prev_time = 0;
		long curr_time = 0;

		// Status
		int curr_status_code = RK_ON;
		// Pins
		int accel_pin = 2;
		int alti_pin = 0;
};

#endif


