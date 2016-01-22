/*
 * protocol.h
 *
 *  Created on: Dec 11, 2015
 *      Author: arthur
 *
 *      This file provides the structures and enumerators for comunication with the groundstation and the DALEK OBC.
 *      The communication is done via three different types of frames:
 *      - telemetry frame
 *      - payload frame
 *      - command frame
 *      Telemetry and payload is sent from the DALEK, the command frame only by the groundstation
 *      If you want to add values to the frames, you must add the new data_type in the enumerator and then again in the
 *      struct below (at the same position relative to the other commands!)
 *      the protocol.h must be synced on the embedded system and on the groundstation to ensure proper communication.
 *      Double and FLoat values are send in full precision as chars (not in ascii representation!) -> need to assemble the four/eight bytes
 *      when receiving!
 *
 *      The framenumber must be increased each time a frame is sent.
 *
 *      EXAMPLE FRAME STRUCTURE:
 *      $$$COMMAND101411230300###
 *      (FRAME_START FRAME_TYPE FRAMENUMBER COMMAND TIME_NANOSECONDS FRAME_END)
 *		
 *		! Everything is sent in uint8_t -> so appropriate recombination is needed! (see embedded system telemetry class for helper functions)
 *		! Casting for signed integers still has to be done
 *		! The values are not separated! The FRAME_START and FRAME_END is repeated three times each at frame start and end!
 *
 */

#include "stdint.h"
#ifndef BASIC_PROTOCOL_H_
#define BASIC_PROTOCOL_H_

#define FRAME_START '$'
#define FRAME_END   '#'
#define MSG_SEPERATOR ';'
#define VALUE_SEPERATOR ','


#define COMMAND_FRAME_SIZE		24	// size in bytes including start and end characters
#define TELEMETRY_FRAME_SIZE    107 //58  // size in bytes including start and end characters
#define PAYLOAD_FRAME_SIZE      224 // size in bytes including start and end characters
#define PAYLOAD_DATA_SIZE       200 // this is the size of only the payload data in the payload frame





/*************** FRAME Structures for the three different frames ***************************************/
enum telemetryFrame{
	TM_FRAMETYPE,		//frametypes are Command, telemetry and payload, see the enumerators below
	TM_FRAMENUMBER,
	SYSTEM_MODE,	//again, see enumerators below
	LIGHT,
	ROLL,
	PITCH,
	YAW,
	GYRO_X,
	GYRO_Y,
	GYRO_Z,
	ACCL_X,
	ACCL_Y,
	ACCL_Z,
	MAG_X,
	MAG_Y,
	MAG_Z,
	TEMP,
	SOLAR_VOLTAGE,
	BATTERY_VOLTAGE,
	CURRENT,
	MOTOR_SPEED,
	IR_DATA_ONE,
	IR_DATA_TWO,
	IR_DATA_THREE,
	GS_SUNFINDER,
	SUN_INCIDENCE_ANGLE,
	TM_LOCALTIME
};
enum payloadFrame{
	PL_FRAMETYPE,
	PL_FRAMENUMBER,
	PAYLOAD_NUMBER, 	// consecutive for one image frame
	PAYLOAD_SIZE,		// size in payload frames
	PAYLOAD,			// array of BYTEs with size of PAYLOAD_SIZE
	PL_LOCALTIME
};
enum commandFrame{
	CMD_FRAMETYPE,
	CMD_FRAMENUMBER,
	COMMAND, 			// values see in enums below
	COMMAND_VALUE,		// value for some commands, e.g. "take picture at angle xx.xx"
	CMD_LOCALTIME
};



/***************************** Enumerators for commands, system modes and frametypes ******************************/
enum frametype{
	TM,
	PL,
	CMD
};
enum commands{
	GOTO_MODE,
	RESET_DALEK,
	// Motor Stuff
	SET_ROTATION_SPEED, // dutycycle, value between 0 and 1000
	CONTROL_MOTOR, //with command value 0 -> stop and command val 1 -> start motor
	GOTO_ANGLE,
	// Mission Stuff
	TAKE_PICTURE_AT,
	EXTERMINAATE,
	// only for Debug Communication -> used before big Mode Loop (used in setNewData in mainthread)
	SET_BETA_GAIN,
	SET_ANGLE_P,
	SET_ANGLE_I,
	SET_ANGLE_D,
	SET_ROTAT_P,
	SET_ROTAT_I,
	SET_ROTAT_D,
	ENABLE_TELEMETRY
};

enum system_mode{
	STANDBY,
	SUN_FINDING,
	MOTOR_CONTROL,
	MISSION
};



/***************** Now Struct definitions for the enumerators (for use in the code 'n shit) ************/

// telemetry frame to Ground
struct TELEMETRY_FRAME{
	int frametype;
	uint32_t framenumber;
	uint8_t systemMode;
	uint16_t light; 			// in lux
	float roll;					// in degree
	float pitch;				// in degree
	float heading;				// in degree
	float gyroX;				// degree per seconds
	float gyroY;
	float gyroZ;
	float acclX;				// g
	float acclY;
	float acclZ;
	float magnX;				// gauss
	float magnY;
	float magnZ;
	float temp;					// degree celsius
	float solarVoltage;		// in Volts
	float batteryVoltage;		// in Volts
	float current;				// in Ampere
	float motorSpeed;			// in ???? dps?
	float irDataOne;			// in centimeter
	float irDataTwo;			// in centimeter
	float irDataThree;			// in centimeter
	uint8_t gs_sunfinder;		// values for Groundstation, 0: default, 1: sunfinder starts, 2: finished rotation, 3: rotates to sun finished, 4: start deployemt, 5: finished and stabilizes
	float incidenceAngle;		// calculated sun incidence angle
	int64_t localTime;			// in nanoseconds since system start!
};

// payload frame to ground
struct PAYLOAD_FRAME{
	int frametype;
	uint32_t framenumber;
	uint32_t payloadnumber;
	uint8_t	payloadsize;
	uint8_t payload[PAYLOAD_FRAME_SIZE];
	int64_t localTime;
};

// command frame from ground to DALEK
struct COMMAND_FRAME{
	uint8_t frameType;
	uint32_t frameNumber;
	uint8_t command;
	float commandValue;
	int64_t localTime;
};

#endif /* BASIC_PROTOCOL_H_ */
