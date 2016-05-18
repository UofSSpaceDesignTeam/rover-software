#ifndef ARM2016_TYPES
#define ARM2016_TYPES

////////////////////////////////////////////////////////////////////////////////
//                             GENERAL TYPES
////////////////////////////////////////////////////////////////////////////////
typedef unsigned int uint_t;

////////////////////////////////////////////////////////////////////////////////
//                              COMM'S TYPES
////////////////////////////////////////////////////////////////////////////////
enum Ecommand_type {MANUAL, INVERSE_KIN};

struct packet {
	Ecommand_type type;
	int8_t position[3]; //x, y, z
	int8_t velocity[6]; // speed and direction of each motor
	uint16_t checksum();
};

////////////////////////////////////////////////////////////////////////////////
//                              DCM TYPES
////////////////////////////////////////////////////////////////////////////////
typedef enum {
	RAMP_UP,
	POSITION_SYNC,
	RAMP_DOWN,
	MIN_VEL,
	DONE
} ERFStage;

#endif
