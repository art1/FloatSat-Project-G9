/*
 * tm.h
 *
 *  Created on: Dec 12, 2015
 *      Author: arthur
 */

#ifndef COMMUNICATION_TELEMETRY_H_
#define COMMUNICATION_TELEMETRY_H_

#include "../Basic/basic.h"


class Telemetry : public Thread {
public:
	Telemetry();
	virtual ~Telemetry();
	void init();
	void run();
	void setNewData(IMU_RPY_FILTERED _imu);
	void setNewData(IMU_DATA_RAW _imuRaw);
	void setNewData(LUX_DATA _lux);
	void setNewData(SOLAR_DATA _sol);
	void setNewData(IR_DATA _ir);
	void setNewData(ACTIVE_SYSTEM_MODE _mode);

	uint32_t getCurrentFrameNumber();

private:
	CommBuffer<TELEMETRY> tmBuf;
	UDPMsg msg;
	uint32_t frameNumber;
	ACTIVE_SYSTEM_MODE systemMode;
	void buildFrame();


//	Fifo<IMU_RPY_FILTERED,10> imu;
	RingBuffer<IMU_DATA_RAW,10> imuRaw;
	RingBuffer<IMU_RPY_FILTERED,10> imu;
	RingBuffer<LUX_DATA,10> lux;
	RingBuffer<SOLAR_DATA,10> sol;
	RingBuffer<IR_DATA,10> ir;





};

#endif /* COMMUNICATION_TELEMETRY_H_ */
