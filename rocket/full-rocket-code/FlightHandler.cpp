#include "FlightHandler.h"

FlightHandler::FlightHandler(Status *flight_status, Calculations *flight_calculator, FlightLogger *flight_logger, long sim_time_to_apogee) {
	logger = flight_logger;
	status = flight_status;
	calculator = flight_calculator;
	current_time = millis();
	//At a minimum apogee wont occur until 8 seconds after launch
	sim_time_to_apogee >= 80000 ? est_time_to_apogee = sim_time_to_apogee : est_time_to_apogee = 15000;
}

void FlightHandler::reset() {
	delete calculator;
	delete status;
	status = new Status(RK_ON, RK_STARTUP);
	calculator = new Calculations(5, 0, status->get_current_status());
}

void FlightHandler::update(float alti, float acc_x, float acc_y, float acc_z) {
	current_time = millis();
	if (!calculator->in_diagnostics)
		calculator->update_time(current_time);

	if (status->get_current_status() == RK_LAUNCHED) {
		update_apogee_times();
	}

	if (status->get_current_status() == RK_APOGEE) {
		status->set_status(RK_DROG_DEPLOY_PRM);
	}

	if (status->get_current_sequence() >= RK_SEQ_RECOV_DROG && status->get_current_sequence() <= RK_SEQ_RECOV_MAIN) {
		update_parachutes();
	}

	switch (status->get_current_sequence()) {
	case RK_SEQ_STARTUP:
		// Require base station to update the state.
		break;
	case RK_SEQ_DIAGNOSTICS:
		if (RK_DIAG_FINISH) {
		}
		break;
	case RK_SEQ_LAUNCH:
    calculator->update_calcs(alti, acc_x, acc_y, acc_z, status->get_current_status()); // Take data in and process it.
    Serial.print("Alti: ");
    Serial.print(calculator->get_curr_altitude());
    Serial.print("\tDeltaAlti: ");
    Serial.print(calculator->get_delta_alti());
    Serial.print("\tAccel Mag: ");
    Serial.println(calculator->get_curr_acceleration());
    if (status->get_current_status() == RK_READY) {
      if (calculator->get_curr_altitude() - this->starting_height >= 0.0 && calculator->get_curr_acceleration() >= 1.3)  { status->set_status(RK_LAUNCHED); launch_time = millis(); }
    } else if (status->get_current_status() == RK_LAUNCHED) {
      if (calculator->get_curr_altitude() - this->starting_height >= 0.0 && calculator->get_curr_acceleration() < 1.3) status->set_status(RK_COAST);
      else if (calculator->get_curr_altitude() - this->starting_height > 0.0 && calculator->get_delta_alti_mag() < 0.6 && calculator->get_curr_acceleration() < 1.5)  status->set_status(RK_APOGEE);
    } else if (status->get_current_status() == RK_APOGEE) {
		  if (calculator->get_curr_altitude() - this->starting_height > 0.0 && calculator->get_delta_alti() < 0.0 && calculator->get_delta_alti_mag() >= 0.6)	status->set_status(RK_RECOVERY);
    }
		break;
	case RK_SEQ_RECOV_DROG:
    calculator->update_calcs(alti, acc_x, acc_y, acc_z, status->get_current_status()); // Take data in and process it.
		break;
	case RK_SEQ_RECOV_MAIN:
    calculator->update_calcs(alti, acc_x, acc_y, acc_z, status->get_current_status()); // Take data in and process it.
		break;
	case RK_SEQ_RECOVERY:
    Serial.print("Alti: ");
    Serial.print(calculator->get_curr_altitude());
    Serial.print("\tDeltaAlti: ");
    Serial.print(calculator->get_delta_alti());
    Serial.print("\tAccel Mag: ");
    Serial.println(calculator->get_curr_acceleration());
    calculator->update_calcs(alti, acc_x, acc_y, acc_z, status->get_current_status()); // Take data in and process it.
    if (status->get_current_status() == RK_LANDED) {
      Serial.print("Landed");
    } else {
      if (calculator->get_delta_alti_mag() <= 0.5 && calculator->get_curr_acceleration() < 1.2 && calculator->get_curr_acceleration() > 0.8 && calculator->get_delta_a_mag() <= 0.2) status->set_status(RK_LANDED);
    }
		
    break;
	}
}

void FlightHandler::update_apogee_times() {
	calc_time_to_apogee = calculator->get_delta_t_L();
	if (millis() - launch_time >= est_time_to_apogee) {
		status->set_status(RK_APOGEE);
	}
}

void FlightHandler::update_parachutes() {
	if (status->get_current_status() == RK_DROG_DEPLOY_PRM) {
		// Trigger
		primary_to_secondary_delay -= calculator->get_delta_t_L();
		if (primary_to_secondary_delay <= 0) {
			status->set_status(RK_DROG_DEPLOY_SEC);
			primary_to_secondary_delay = PRM_TO_SEC_DELAY;
		}
	} 
	if (status->get_current_status() == RK_DROG_DEPLOY_SEC) {
		//Trigger
		drouge_to_main_delay -= calculator->get_delta_t_L();
		if (drouge_to_main_delay <= 0) { //calculator->get_curr_altitude() < 500 ||
			status->set_status(RK_MAIN_DEPLOY_PRM);
		}
	} else if (status->get_current_status() == RK_MAIN_DEPLOY_PRM) {
		//Trigger 
		primary_to_secondary_delay -= calculator->get_delta_t_L();
		if (primary_to_secondary_delay <= 0) {
			status->set_status(RK_MAIN_DEPLOY_SEC);
		}
	} else if (status->get_current_status() == RK_MAIN_DEPLOY_SEC) {
		//Trigger
		status->set_status(RK_RECOVERY);
	}
}

void FlightHandler::set_status(int new_status_code) {
	if (status->determine_sequence(new_status_code) != status->get_current_sequence())
		logger->write_to_coms(millis(), MSG_SEQ_CHANGE, status->get_current_sequence(), status->get_current_status(), String(status->determine_sequence(new_status_code)).c_str());
	logger->write_to_coms(millis(), MSG_STATUS_CHANGE, status->get_current_sequence(), status->get_current_status(), String(new_status_code).c_str());
	status->set_status(new_status_code);
}

void FlightHandler::set_gnd_ready_flag() {
	gnd_ready_flag = true;
}

void FlightHandler::handle_diagnostic_data(String data[9]) {
	long time = (long)(data[1].toFloat()*1000);
	float accel_z = data[2].toFloat();
	calculator->diagnostics_calculations(time, accel_z);
	//update();
}

void FlightHandler::send_calculations() {
	// Save all data.
	if (calculator->in_diagnostics) {
		logger->write_to_log(calculator->get_curr_time(), String(calculator->get_curr_velocity()));
	} else {
		if (status->get_current_sequence() >= RK_SEQ_LAUNCH) {
			String data_string = String(calculator->get_curr_time()).concat(',') + String(calculator->get_curr_altitude()).concat(',') + String(calculator->get_curr_acceleration()).concat(',') + String(calculator->get_curr_velocity()) + ",0.0,0.0";
			logger->write_to_coms(millis(), MSG_DATA, status->get_current_sequence(), status->get_current_status(), "");
		} else {
	  }
	}
}

