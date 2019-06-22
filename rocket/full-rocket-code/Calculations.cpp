#include "Arduino.h"
#include "Calculations.h"

#include <math.h>

Calculations::Calculations(int pin_accel, int init_status_code) {
  	accel_pin = pin_accel;
  	alti_pin = 2;
  	pinMode(accel_pin, INPUT);
  	pinMode(alti_pin, INPUT);
  	curr_status_code = init_status_code;
}

Calculations::Calculations(int pin_accel, int pin_alti, int init_status_code) {
    accel_pin = pin_accel;
    alti_pin = pin_alti;
    pinMode(accel_pin, INPUT);
    pinMode(alti_pin, INPUT);
    curr_status_code = init_status_code;
}

void Calculations::update_calcs() {
	// Update Altitude
	prev_alti = curr_alti;
	curr_alti = analogRead(alti_pin);

	delta_alti = curr_alti - prev_alti;
	delta_alti_is_pos = (delta_alti >= 0);

	// Update Acceleration
	if (!in_diagnostics) {
		prev_accel = curr_accel;
		curr_accel = digitalRead(accel_pin);
	
		delta_a = curr_accel - prev_accel;
		delta_a_is_pos = (delta_a >= 0);

		//NOTE!!!!!!!!!!!!!!!!!!!!!!! JUST TEMP:
		curr_accel = delta_alti;
	}
	//

	// Update Velocity
	prev_velocity = curr_velocity;
	curr_velocity = prev_velocity + (curr_accel * get_delta_t()); // (v - u) / t = a

	delta_v = curr_velocity - prev_velocity;
	delta_v_is_pos = (delta_v >= 0);

	if (curr_status_code == RK_COAST) {
		time_to_apogee();
	}

	/*
	 Statistics
	*/
	if (curr_accel > max_accel) max_accel = curr_accel;
	if (curr_accel < min_accel) min_accel = curr_accel;
	if (delta_a > max_delta_a) max_delta_a = delta_a;
	if (delta_a < min_delta_a) min_delta_a = delta_a;

	if (curr_alti > max_alti) max_alti = curr_alti;
	if (curr_alti < min_alti) min_alti = curr_alti;
	if (delta_alti > max_delta_alti) max_delta_alti = delta_alti;
	if (delta_alti < min_delta_alti) min_delta_alti = delta_alti;

	if (curr_velocity > max_v) max_v = curr_velocity;
	if (curr_velocity < min_v) min_v = curr_velocity;
	if (delta_v > max_delta_v) max_delta_v = delta_v;
	if (delta_v < min_delta_v) min_delta_v = delta_v;
}

void Calculations::update_calcs(float new_alti, float acc_x, float acc_y, float acc_z, int status_code) {
  // Update Altitude
  prev_alti = curr_alti;
  curr_alti =  new_alti;

  delta_alti = curr_alti - prev_alti;
  delta_alti_is_pos = (delta_alti >= 0);

  
  float new_accel = get_acceleration_mag(acc_x, acc_y, acc_z);
  // Update Acceleration
  if (!in_diagnostics) {
    prev_accel = curr_accel;
    curr_accel = new_accel;
  
    delta_a = curr_accel - prev_accel;
    delta_a_is_pos = (delta_a >= 0);
  }

  if (status_code > RK_LAUNCHED) {
    if (acc_x >= 0.2) {
      velo_x += (acc_x*get_delta_t());
    }
    if (acc_y >= 0.2) {
      velo_y += (acc_y*get_delta_t());
    }
    if (acc_z >= 0.2) {
      velo_z += (acc_z*get_delta_t());
    }
    
    float new_velo = get_acceleration_mag(velo_x, velo_y, velo_z);
    
    // Update Velocity
    prev_velocity = curr_velocity;
    curr_velocity = new_velo;
  
    delta_v = curr_velocity - prev_velocity;
    delta_v_is_pos = (delta_v >= 0);
  }
  
  if (curr_status_code == RK_COAST) {
    time_to_apogee();
  }

  /*
   Statistics
  */
  if (curr_accel > max_accel) max_accel = curr_accel;
  if (curr_accel < min_accel) min_accel = curr_accel;
  if (delta_a > max_delta_a) max_delta_a = delta_a;
  if (delta_a < min_delta_a) min_delta_a = delta_a;

  if (curr_alti > max_alti) max_alti = curr_alti;
  if (curr_alti < min_alti) min_alti = curr_alti;
  if (delta_alti > max_delta_alti) max_delta_alti = delta_alti;
  if (delta_alti < min_delta_alti) min_delta_alti = delta_alti;

  if (curr_velocity > max_v) max_v = curr_velocity;
  if (curr_velocity < min_v) min_v = curr_velocity;
  if (delta_v > max_delta_v) max_delta_v = delta_v;
  if (delta_v < min_delta_v) min_delta_v = delta_v;
}

void Calculations::diagnostics_calculations(long new_time, float new_accel) {
	update_time(new_time);
	prev_accel = curr_accel;
	curr_accel = new_accel;

	delta_a = curr_accel - prev_accel;
	delta_a_is_pos = (delta_a >= 0);
	update_calcs();
}

void Calculations::update_time(long new_time) {
	// Update Timing
	prev_time = curr_time;
	curr_time = new_time;
	delta_t = curr_time - prev_time;
}

long Calculations::time_to_apogee() {
	if (get_curr_velocity() >= 0) {
		float expected_time = (0.0 - get_curr_velocity()) / (-9.8);
		return (long)(expected_time*1000.0);
	}
	return 10000;
}

/*
 Acceleration Getters
*/
float Calculations::get_curr_acceleration() { return curr_accel; }

float Calculations::get_prev_acceleration() { return prev_accel; }

float Calculations::get_curr_acceleration_mag() { return fabs(get_curr_acceleration()); }

float Calculations::get_prev_acceleration_mag() { return fabs(get_curr_acceleration()); }


float Calculations::get_acceleration_mag(float a_x, float a_y, float a_z) {
	return sqrtf(pow(a_x, 2) + pow(a_y, 2) + pow(a_z, 2));
}

float Calculations::get_delta_a() { return delta_a / get_delta_t(); }

float Calculations::get_delta_a_mag() { return fabs(get_delta_a()); }

/*
 Altitude Getters
*/
float Calculations::get_curr_altitude() { return curr_alti; }

float Calculations::get_prev_altitude() { return prev_alti; }

float Calculations::get_delta_alti() { return delta_alti; }

float Calculations::get_delta_alti_mag() { return fabs(get_delta_alti()); }


/*float Calculations::get_curr_pressure() { return curr_pressure; }

float Calculations::get_delta_pressure() { return delta_pressure; }

float Calculations::get_curr_temperature() { return altimeter_sensor->readTemperature(); }*/

/*
	Velocity Getters
*/
float Calculations::get_curr_velocity() { return curr_velocity; }

float Calculations::get_prev_velocity() { return prev_velocity; }

float Calculations::get_delta_v() { return  delta_v; }

float Calculations::get_delta_v_mag() { return fabs(get_delta_v()); }

float Calculations::get_delta_t() { return ((float)delta_t)/1000.0f; }

long Calculations::get_delta_t_L() { return delta_t; }

long Calculations::get_curr_time() { return curr_time; }


