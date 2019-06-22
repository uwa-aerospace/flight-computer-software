#include "Status.h"


Status::Status(int init_status_code) {
	prev_status_code = RK_OFF;
	curr_status_code = init_status_code;
	set_init_sequence(prev_status_code, curr_status_code);
}

Status::Status(int base_status, int curr_status) {
	prev_status_code = base_status;
	curr_status_code = curr_status;
	set_init_sequence(prev_status_code, curr_status_code);
}

void Status::set_init_sequence(int prev_status, int curr_status) {
	prev_sequence_code = determine_sequence(prev_status);
	curr_sequence_code = determine_sequence(curr_status);
}

int Status::determine_sequence(int status_code) {
	if (IS_SEQ_STARTUP(status_code))		return RK_SEQ_STARTUP;
	if (IS_SEQ_DIAGNOSTICS(status_code))	return RK_SEQ_DIAGNOSTICS;
	if (IS_SEQ_LAUNCH(status_code))			return RK_SEQ_LAUNCH;
	if (IS_SEQ_RECOV_DROG(status_code))		return RK_SEQ_RECOV_DROG;
	if (IS_SEQ_RECOV_MAIN(status_code))		return RK_SEQ_RECOV_MAIN;
	if (IS_SEQ_RECOVERY(status_code))		return RK_SEQ_RECOVERY;
}

int Status::get_current_sequence() { return curr_sequence_code; }

int Status::get_previous_sequence() { return prev_sequence_code; }

void Status::set_sequence(int new_sequence_code) {
	if (IS_VALID_SEQ(new_sequence_code)) {
		prev_sequence_code = curr_sequence_code;
		curr_sequence_code = new_sequence_code;
	}
}

int Status::get_current_status() { return curr_status_code; }

int Status::get_previous_status() { return prev_status_code; }

// Used to ensure that a status is within the range of expected values.
boolean Status::is_valid_status(int status_code) { 
	return (status_code >= RK_OFF && status_code <= RK_LANDED) || (status_code == RK_ERROR);
}

void Status::set_status(int new_status_code) {
	if (is_valid_status(new_status_code)) {
		if (new_status_code != curr_status_code) {
			if (determine_sequence(new_status_code) > curr_sequence_code) {
				set_sequence(determine_sequence(new_status_code));
			}
			prev_status_code = curr_status_code;
			curr_status_code = new_status_code;
		}
	}
}


