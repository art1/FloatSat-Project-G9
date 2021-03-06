/*
 * mainThread.cpp
 *
 *  Created on: Oct 31, 2015
 *      Author: arthur
 */

//#include "Template.h"
#include "mainThread.h"


//#include "Communication/spiconnector.h"
static Application mainThread("mainThread",20);

// load all appropriate classes (-> must be public, initializing classes within a class is not working... ?)

#ifdef IMU_ENABLE
IMU imu;
#endif
#ifdef TTNC_ENABLE
Telecommand tc;
Telemetry tm;
#ifdef WIFI_ENABLE
WiFi wifi;
#else
Bluetooth blauzahn;
#endif
#endif
#ifdef FUSION_ENABLE
sensorFusion fusion;
#endif
#ifdef LIGHT_ENABLE
lightSensor light;
#endif
#ifdef SOLAR_ADC_ENABLE
SolarPanels solar;
#endif
#ifdef CAMERA_ENABLE
Camera camera;
#endif
#ifdef IR_ENABLE
InfraredSensors irSensors;
#endif
#ifdef MOTOR_ENABLE
MotorControlThread motorControl;
Motor motor;
#endif
#ifdef KNIFE_ENABLE
ThermalKnife knife;
#endif
#ifdef SUNFINDER_ENABLE
SunFinder sunFinder;
#endif
#ifdef CURRENT_ENABLE
currentSensors current;
#endif

HAL_GPIO laser(GPIO_015);


//RESUMER THREADS
/**************************** IMU MESSAGES **************************************/
#ifdef FUSION_ENABLE
struct receiver_Fusion : public Subscriber, public Thread {
	receiver_Fusion(const char* _name) : Subscriber(imu_rawData,"IMU Raw Data"),Thread("IMU->Fusion Raw",119,500) {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		fusion.newData(*(IMU_DATA_RAW*)data);
#ifdef MOTOR_ENABLE
		motorControl.setNewData(*(IMU_DATA_RAW*)data);
#endif
#ifdef TTNC_ENABLE
		tm.setNewData(*(IMU_DATA_RAW*)data);
#endif
		fusion.resume();
		return 1;
	}
	void run(){}
} fusion_receiver_thread("Fusion Receiver");

struct receiver_filtered : public Subscriber, public Thread {
	receiver_filtered() : Subscriber(imu_filtered,"IMU Filtered Data"), Thread("filtered IMU Data",118,500) {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
#ifdef TTNC_ENABLE
		tm.setNewData(*(IMU_RPY_FILTERED*)data);
#endif
#ifdef SUNFINDER_ENABLE
		// if sunfinder is active, set the data to the INTERCOMM too, to avoid corrupted data!
		if(sunFinder.isActive())tempComm.imuData = *(IMU_RPY_FILTERED*)data;
#endif
#ifdef MOTOR_ENABLE
		motorControl.setNewData(*(IMU_RPY_FILTERED*)data);
#endif
		return 1;
	}
	void run(){}
} filtered_receiver_thread;
#endif

/**************************** TTnC MESSAGES **************************************/
#ifdef TTNC_ENABLE
struct receiver_telecommand : public Subscriber, public Thread {
	receiver_telecommand() : Subscriber(tcRaw,"TC Raw Data"), Thread("Comm->TC Handler",121,500){}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		tc.setNewData(*(UDPMsg*)data);
		tc.resume();
		return 1;
	}
	void run(){}
} tc_receiver_thread;

// and now the other way round -> TM to Wifi
struct receiver_telemetry : public Subscriber, public Thread {
	receiver_telemetry() : Subscriber(tmPlFrame,"TelemetryPayloadFrame"), Thread("TM-Handler -> Comm",122,500) {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
#ifdef WIFI_ENABLE
		wifi.setNewData(*(UDPMsg*)data);
		wifi.resume();
#else
		blauzahn.setNewData(*(UDPMsg*)data);
		blauzahn.resume();
#endif
		return 1;
	}
	void run(){}
} wifi_receiver_thread;

// and now COmmand Messages to Main Control Thread
struct receiver_tcControl : public Subscriber, public Thread {
	receiver_tcControl() : Subscriber(commandFrame,"TC Control Data"), Thread("TC Handler -> Main",120,500) {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		mainT.setNewData(*(COMMAND_FRAME*)data);
		mainT.resume();
		return 1;
	}
	void run(){}
} tcControl_receiver_thread;
#endif


// the following thread is for Inter-Thread Communication of Sensors only, except for IMU Data!
// Why? because the amount of threads got too high and RODOS doesn't like that! (-> xmalloc for threads?? -> see github commit history in ahrs branch)
struct sensorsCommThread : public Subscriber, public Thread {
	sensorsCommThread() : Subscriber(interThreadComm,"Inter Thread Communication for Sensors"), Thread("sensor Comm",123,1000) {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		INTERCOMM tmp = *(INTERCOMM*)data;
		//		PRINTF("InterThread called with %d\n",tmp.changedVal);
		switch (tmp.changedVal) {
		case LUX_CHANGED:
#ifdef SUNFINDER_ENABLE
			//update sunfinder lux AND IMU data at the same time, to have the same amount of lux and imu measurements
			if(sunFinder.isActive()){
				sunFinder.setNewData(tempComm.imuData); // imu Data has been set to the intercomm temp !
				sunFinder.setNewData(tmp.luxData);
			}
#endif
#ifdef LIGHT_ENABLE
			light.setActive(tmp.luxData);
#ifdef TTNC_ENABLE
			tm.setNewData(tmp.luxData);
#endif
			light.resume();
#endif
			break;
		case SOLAR_CHANGED:
#ifdef SOLAR_ADC_ENABLE
			solar.setNewData(tmp.solData);
#ifdef TTNC_ENABLE
			tm.setNewData(tmp.solData);
#endif
			solar.resume();
#endif
			break;
		case IR_CHANGED:
#ifdef IR_ENABLE
			irSensors.setNewData(tmp.irData);
#ifdef TTNC_ENABLE
			tm.setNewData(tmp.irData);
#endif
			irSensors.resume();
#endif
			break;
		case KNIFE_CHANGED:
#ifdef KNIFE_ENABLE
			knife.setNewData(tmp.knifeData);
			knife.resume();
#endif
			break;
		case CAM_CHANGED:
#ifdef CAMERA_ENABLE
			if(tmp.camData.activateCamera || tmp.camData.capture){
				camera.setNewData(tmp.camData);
				camera.resume();
			} else {
				//#ifdef TELEMETRY_ENABLE
				tm.sendPayload(tmp.camData);
			}
			//#endif
#endif
			break;
		case CURRENT_CHANGED:
#ifdef CURRENT_ENABLE
#ifdef TTNC_ENABLE
			tm.setNewData(tmp.currentData);
#endif
#endif
			break;
		case VARIABLE_CHANGED:
#ifdef MOTOR_ENABLE
			motorControl.setNewData(&tmp.varControlData);
#endif
			break;
		case SUNFINDER_TM_CHANGED:

			break;
		default:
			break;

		}
		return 1;
	}
	void run(){}
} sensorsComm;

mainThread::mainThread(const char* name) : Thread(name){

}

void mainThread::setNewData(COMMAND_FRAME _t){
	this->cmd = _t;
	PRINTF("set new command values %f\n",cmd.commandValue);
	VAR_CONTROL var;
	var.changedVal = cmd.command;
	var.value = cmd.commandValue;

	switch (cmd.command) {
	case SET_BETA_GAIN:
		PRINTF("Setting new Beta Gain %f\n",var.value);
#ifdef FUSION_ENABLE
		fusion.setNewData(&var);
#endif
		break;
	case SET_ANGLE_P:
	case SET_ANGLE_I:
	case SET_ANGLE_D:
	case SET_ROTAT_P:
	case SET_ROTAT_I:
	case SET_ROTAT_D:
		PRINTF("SEtting new PID values...\n");
#ifdef MOTOR_ENABLE
		motorControl.setNewData(&var);
#endif
		break;
	case ENABLE_TELEMETRY:
		//#ifdef TELEMETRY_ENABLE
		if((int)cmd.commandValue == 1) {
			PRINTF("Enabling Telemetry\n");
			tm.setActive(true);
			tm.resume();
		}else{
			tm.setActive(false);
		}
		//#endif
		break;
	case CALIBRATE_IMU:
		PRINTF("calibrating Magnetometer\n");
		imu.calMagnSpinning = true;
		imu.calibrateSensors();
		break;
	case CUT_THERMAL_KNIFE:
		tempComm.knifeData.activated = true;
		tempComm.changedVal = KNIFE_CHANGED;
		interThreadComm.publish(tempComm);
		break;
	default:
		break;
	}
}

void mainThread::init(){
	PRINTF("init mainThread\r\n");

	//enable GPIOD channel clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIODEN,ENABLE);
	// init LEDs -> not using rodos because reasons
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = LED_GREEN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		// TBD what speed to use????
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = LED_ORANGE;
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = LED_RED;
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = LED_BLUE;
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	currentSystemMode.activeMode = STANDBY;
	cmd.command = -1;

}

/**
 * mainThread used to control the different satellite modes
 * Thread is resumed when new commands from the groundstatoin arrives!
 */
void mainThread::run(){
	laser.init(true,1,0);

#ifndef WIFI_ENABLE
	bt_uart.init(BLUETOOTH_BAUDRATE);
#endif


	//enable ADC1 channel with 12 Bit Resolution
	adc1.config(ADC_PARAMETER_RESOLUTION,ADC1_RESOLUTION);
#ifdef IMU_ENABLE
	//	if(imu.calMagnSpinning){
	//		motorControl.setRotationSpeed(10);
	//	}
	i2c2.init(400000);
	imu.regInit();
	while(!imu.initFinished());
	//	if(imu.calMagnSpinning){
	//		motorControl.setMotor(false);
	//	}
	//	while(!imu.calibrationFinished);
	//imu.resume();
#endif
	suspendCallerUntil(NOW()+2000*MILLISECONDS);
#ifdef CAMERA_ENABLE
	//	camera.initCamera();
	//	i2c1.init();
	while(!camera.initFinished());
#endif

#ifdef LIGHT_ENABLE
	i2c1.init(400000);
	//	LUX_DATA temp;
	//	temp.activated = true;
	//	tempComm.luxData = temp;
	//	tempComm.changedVal = LUX_CHANGED;
	//	interThreadComm.publish(tempComm);
#endif
#ifdef CURRENT_ENABLE
#ifndef LIGHT_ENABLE
	i2c1.init(400000);
#endif
#endif

#ifdef IR_ENABLE
	IR_DATA tmp2;
	irSensors.init();
	tmp2.activated = true;
	//	PRINTF("publishing Data\n");
	tempComm.irData = tmp2;
	tempComm.changedVal = IR_CHANGED;
	interThreadComm.publish(tempComm);
#endif
#ifdef KNIFE_ENABLE
//		tempComm.knifeData.activated = true;
//		tempComm.changedVal = KNIFE_CHANGED;
//		interThreadComm.publish(tempComm);
#endif

	//	while(1){


	//		PRINTF("and now im here\n");
	//		suspendCallerUntil(END_OF_TIME);
	//	}

	PRINTF("SYSTEM HELLO!\n");
	suspendCallerUntil(NOW()+3*SECONDS);
	bool switchedMode = false;
	while(1){
		//check which mode
		if(cmd.command == GOTO_MODE){
			currentSystemMode.activeMode = (int) cmd.commandValue;
			tm.setNewData(currentSystemMode);
			PRINTF("here cmd, going to mode %d\n",currentSystemMode.activeMode);
			if(currentSystemMode.activeMode == STANDBY){
				switchedMode = true;				// going to standby immediately after received command
				ORANGE_ON;
			} else ORANGE_OFF;
			cmd.command = -1;
		}
		if(!(cmd.command == GOTO_MODE) || switchedMode) {
			switchedMode = false;
			PRINTF("hello, active Mode: %d\n",currentSystemMode.activeMode);
			switch (currentSystemMode.activeMode) {
			case STANDBY:
				// do nothing, only blink a led or some shit
				PRINTF("DALEK waiting for commands!\n");
				switch(cmd.command){
				case TAKE_PICTURE_AT:
					// taking picture at angle!
#ifdef CAMERA_ENABLE
					PRINTF("enabling cam in 1 secs...\n");
					Delay_millis(100);
					PRINTF("should be enabled in a few msecs\n");
					CAM_DATA tmp3;
					tmp3.activateCamera = true;
					tmp3.capture = true;
					tempComm.camData = tmp3;
					tempComm.changedVal = CAM_CHANGED;
					interThreadComm.publish(tempComm);
#endif
					break;
				default:
					break;
				}
#ifdef MOTOR_ENABLE
				motorControl.setMotor(false);
#endif
				break;
				case SUN_FINDING:
#ifdef SUNFINDER_ENABLE
					PRINTF("sun finding mode!\n");
					sunFinder.setActive(true);
					sunFinder.resume();

					switch (cmd.command) {
					case SET_ROTATION_SPEED:

						motorControl.setRotationSpeed(cmd.commandValue);
						break;
					case GOTO_ANGLE:
						motorControl.gotoAngle(cmd.commandValue);
						break;
					default:
						break;
					}
#endif
					break;
					case MOTOR_CONTROL:
#ifdef MOTOR_ENABLE
						// seting motor speed,
						PRINTF("motor control mode with command %d\n",cmd.command);

						switch (cmd.command) {
						case SET_ROTATION_SPEED:

							motorControl.setRotationSpeed(cmd.commandValue);
							break;
						case CONTROL_MOTOR:
							if((int)cmd.commandValue == 1){
								motorControl.setMotor(true);
							} else if((int)cmd.commandValue == 0){
								motorControl.setMotor(false);
							}
							break;
						case GOTO_ANGLE:
							motorControl.gotoAngle(cmd.commandValue);
							break;
						default:
							break;
						}
#endif
						break;
						//				break;
						case MISSION:
							// execute mission
							PRINTF("mission mode!\n");
							//					cmd.command = TAKE_PICTURE_AT;
							switch (cmd.command) {
							case TAKE_PICTURE_AT:
								// taking picture at angle!
#ifdef CAMERA_ENABLE
								PRINTF("enabling cam in 1 secs...\n");
								Delay_millis(100);
								PRINTF("should be enabled in a few msecs\n");
								CAM_DATA tmp3;
								tmp3.activateCamera = true;
								tmp3.capture = true;
								tempComm.camData = tmp3;
								tempComm.changedVal = CAM_CHANGED;
								interThreadComm.publish(tempComm);
#endif
								break;
							case EXTERMINATE:
								if((int)cmd.commandValue == 1) laser.setPins(1);
								else laser.setPins(0);
								break;
							case SET_ROTATION_SPEED:

								motorControl.setRotationSpeed(cmd.commandValue);
								break;
							case CONTROL_MOTOR:
								if((int)cmd.commandValue == 1){
									motorControl.setMotor(true);
								} else if((int)cmd.commandValue == 0){
									motorControl.setMotor(false);
								}
								break;
							case GOTO_ANGLE:
								motorControl.gotoAngle(cmd.commandValue);
								break;
							default:
								break;
							}
							break;
							default:
								break;
			}
		}
		suspendCallerUntil(END_OF_TIME);

	}



}

