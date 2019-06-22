#ifndef _COMMANDS_h
#define _COMMANDS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//Unused
#define	MSG_AWAIT_FLAG		0
#define MSG_RECV_FLAG		1
#define MSG_READY_FLAG		2
//End

#define MSG_HEARTBEAT		0
#define MSG_DATA			1
#define MSG_SEQ_CHANGE		2
#define MSG_STATUS_CHANGE	3
#define MSG_STATS			4

#define GND_STARTUP			4
#define GND_CONFIRM_READY	5
#define GND_FORCE_READY		6
#define GND_START_DIAG		7
#define	GND_DIAG_DATA		8
#define GND_STOP_DIAG		9
#define GND_DIAG_ERROR		10
#define GND_APOGEE_OVERRIDE 11
#define GND_DROG_DEPLOY		12
#define GND_MAIN_DEPLOY		13
#define GND_STATS_REQUEST 14

#define STOP_DATA_WRITE 99

/*
	Messages sent will have the following structure:
	| int	   | int	  | int	   | char*
	| msg_type | sequence | status | payload

	Messages received just need to make receiver do something hence have following structure.
	| int	  |
	| command |
*/
#endif



