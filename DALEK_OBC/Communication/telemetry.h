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
	void setActive(bool _val);
	bool isActive();
	void setNewData(IMU_RPY_FILTERED _imu);
	void setNewData(IMU_DATA_RAW _imuRaw);
	void setNewData(LUX_DATA _lux);
	void setNewData(SOLAR_DATA _sol);
	void setNewData(IR_DATA _ir);
	void setNewData(CURRENT_DATA _current);
	void setNewData(ACTIVE_SYSTEM_MODE _mode);
	void setNewData(INTERCOMM _interComm);
	void sendPayload(CAM_DATA _camData);

	uint32_t getCurrentFrameNumber();

private:
	bool active;
	CommBuffer<TELEMETRY> tmBuf;
	UDPMsg msg;
	uint32_t frameNumber;
	ACTIVE_SYSTEM_MODE systemMode;
	void buildFrame();


//	Fifo<IMU_RPY_FILTERED,10> imu;
	RingBuffer<IMU_DATA_RAW,5> imuRaw;
	RingBuffer<IMU_RPY_FILTERED,5> imu;
	RingBuffer<LUX_DATA,5> lux;
	RingBuffer<SOLAR_DATA,5> sol;
	RingBuffer<IR_DATA,5> ir;
	RingBuffer<CURRENT_DATA, 5> current;





};

#endif /* COMMUNICATION_TELEMETRY_H_ */
