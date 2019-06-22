#ifndef _STATUS_h
#define _STATUS_h

// All the different states the rocket can possibly be in
#define RK_OFF				  0
#define RK_ON				  1
#define RK_STARTUP			  2
#define RK_DIAG_START		  3
#define RK_DIAG_FINISH		  4
#define RK_DIAG_ERROR		  5
#define RK_READY			  6

#define RK_LAUNCHED			  7
#define RK_COAST			  8
#define RK_APOGEE			  9
#define RK_DROG_DEPLOY_PRM	  10
#define RK_DROG_DEPLOY_SEC	  11
#define RK_MAIN_DEPLOY_PRM	  12
#define RK_MAIN_DEPLOY_SEC	  13
#define RK_RECOVERY			  14
#define RK_LANDED			  15
#define RK_ERROR			  99

// All the different sequences (collection of common states) the rocket may be in.
#define RK_SEQ_STARTUP		  0
#define RK_SEQ_DIAGNOSTICS	  1
#define RK_SEQ_LAUNCH		  2
#define RK_SEQ_RECOV_DROG	  3
#define RK_SEQ_RECOV_MAIN	  4
#define RK_SEQ_RECOVERY		  5

#define IS_VALID_SEQ(x)		  x >= RK_SEQ_STARTUP && x <= RK_SEQ_RECOVERY

// Simple functions used to determine which sequence a rocket is in based on status.
#define IS_SEQ_STARTUP(x)	  x <= RK_STARTUP
#define IS_SEQ_DIAGNOSTICS(x) x <= RK_DIAG_ERROR && x >= RK_DIAG_START
#define IS_SEQ_LAUNCH(x)	  x <= RK_APOGEE && x >= RK_READY
#define IS_SEQ_RECOV_DROG(x)  x <= RK_DROG_DEPLOY_SEC && x >= RK_DROG_DEPLOY_PRM
#define IS_SEQ_RECOV_MAIN(x)  x <= RK_MAIN_DEPLOY_SEC && x >= RK_MAIN_DEPLOY_PRM
#define IS_SEQ_RECOVERY(x)    x <= RK_LANDED && x >= RK_RECOVERY

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
class Status {
	public:
		Status(int init_status_code);
		Status(int base_status_code, int init_status_code);
		int  get_current_sequence();
		int  get_previous_sequence();
		void set_init_sequence(int, int);
		int  determine_sequence(int);
		void set_sequence(int);
		int  get_current_status();
		int  get_previous_status();
		void set_status(int);
	private:
		boolean is_valid_sequence(int);
		boolean is_valid_status(int);
		int prev_sequence_code = 0;
		int curr_sequence_code = 0;
		int prev_status_code = 0;
		int curr_status_code = 0;
};
#endif



