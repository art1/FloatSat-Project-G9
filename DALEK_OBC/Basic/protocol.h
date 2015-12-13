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
 *      $,COMMAND,10,14,11230300,#
 *      (FRAME_START,FRAME_TYPE,FRAMENUMBER,COMMAND,TIME_NANOSECONDS,FRAME_END)
 *
 */

#ifndef BASIC_PROTOCOL_H_
#define BASIC_PROTOCOL_H_

#define FRAME_START '$'
#define FRAME_END   '#'
#define MSG_SEPERATOR ';'
#define VALUE_SEPERATOR ','






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
	SET_ROTATION_SPEED,
	GOTO_ANGLE,
	TAKE_PICTURE_AT
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
	float light; 				// in lux
	double roll;				// in degree
	double pitch;				// in degree
	double heading;				// in degree
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
	int32_t solarVoltage;		// in Volts
	int32_t batteryVoltage;		// in Volts
	float current;				// in Ampere
	float motorSpeed;			// in ???? dps?
	int32_t irDataOne;			// in meter
	int32_t irDataTwo;			// in meter
	int32_t irDataThree;		// in meter
	int64_t localTime;			// in nanoseconds since system start!
};

// payload frame to ground
struct PAYLOAD_FRAME{
	int frametype;
	uint32_t framenumber;
	uint32_t payloadnumber;
	uint8_t	payloadsize;
	uint8_t payload[];
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
