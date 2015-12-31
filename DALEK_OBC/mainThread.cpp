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
#ifndef BLUETOOTH_FALLBACK
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
#endif
#ifdef KNIFE_ENABLE
ThermalKnife knife;
#endif
#ifdef SUNFINDER_ENABLE
SunFinder sunFinder;
#endif

//RESUMER THREADS
/**************************** IMU MESSAGES **************************************/
#ifdef FUSION_ENABLE
struct receiver_Fusion : public Subscriber, public Thread {
	receiver_Fusion(const char* _name) : Subscriber(imu_rawData,"IMU Raw Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		fusion.newData(*(IMU_DATA_RAW*)data);
#ifdef TTNC_ENABLE
		tm.setNewData(*(IMU_DATA_RAW*)data);
#endif
		fusion.resume();
		return 1;
	}
	void run(){}
} fusion_receiver_thread("Fusion Receiver");

struct receiver_filtered : public Subscriber, public Thread {
	receiver_filtered() : Subscriber(imu_filtered,"IMU Filtered Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
#ifdef TTNC_ENABLE
		tm.setNewData(*(IMU_RPY_FILTERED*)data);
#endif
		// if sunfinder is active, set the data to the INTERCOMM too, to avoid corrupted data!
		if(sunFinder.isActive())tempComm.imuData = *(IMU_RPY_FILTERED*)data;
		return 1;
	}
	void run(){}
} filtered_receiver_thread;
#endif

/**************************** TTnC MESSAGES **************************************/
#ifdef TTNC_ENABLE
struct receiver_telecommand : public Subscriber, public Thread {
	receiver_telecommand(const char* _name,int _prio, int _stackSize) : Subscriber(tcRaw,"TC Raw Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		tc.setNewData(*(UDPMsg*)data);
		tc.resume();
		return 1;
	}
	void run(){}
} tc_receiver_thread("TC Receiver",102,200);

// and now the other way round -> TM to Wifi
struct receiver_telemetry : public Subscriber, public Thread {
	receiver_telemetry() : Subscriber(tmPlFrame,"TelemetryPayloadFrame") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
#ifndef BLUETOOTH_FALLBACK
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
	receiver_tcControl(const char* _name,int _prio, int _stackSize) : Subscriber(commandFrame,"TC Control Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		mainT.setNewData(*(COMMAND_FRAME*)data);
		mainT.resume();
		return 1;
	}
	void run(){}
} tcControl_receiver_thread("TC Control Receiver", 100, 500);
#endif


// the following thread is for Inter-Thread Communication of Sensors only, except for IMU Data!
// Why? because the amount of threads got too high and RODOS doesn't like that! (-> xmalloc for threads?? -> see github commit history in ahrs branch)
struct sensorsCommThread : public Subscriber, public Thread {
	sensorsCommThread(const char* _name,int _prio,int _stacksize) : Subscriber(interThreadComm,"Inter Thread Communication for Sensors") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		INTERCOMM tmp = *(INTERCOMM*)data;
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
			camera.setNewData(tmp.camControl);
			camera.resume();
#endif
			break;
		default:
			break;
		}
		return 1;
	}
	void run(){}
} sensorsComm("Inter-Thread Comm",101,500);

mainThread::mainThread(const char* name) : Thread(name){

}

void mainThread::setNewData(COMMAND_FRAME _t){
	this->cmd = _t;
	PRINTF("set new values %f\n",cmd.commandValue);
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
#ifdef BLUETOOTH_FALLBACK
	bt_uart.init(BLUETOOTH_BAUDRATE);
#endif

	//enable ADC1 channel with 12 Bit Resolution
	adc1.config(ADC_PARAMETER_RESOLUTION,ADC1_RESOLUTION);
#ifdef IMU_ENABLE
	i2c2.init(400000);
	imu.regInit();
	imu.setTime(500*MILLISECONDS);
#endif

#ifdef LIGHT_ENABLE
	i2c1.init(400000);
	LUX_DATA temp;
	temp.activated = true;
	tempComm.luxData = temp;
	tempComm.changedVal = LUX_CHANGED;
	interThreadComm.publish(tempComm);
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

//	while(1){

#ifdef CAMERA_ENABLE
		PRINTF("enabling cam in 1 secs...\n");
		Delay_millis(100);
		PRINTF("should be enabled in a few msecs\n");
		CAM_CONTROL tmp3;
		tmp3.activateCamera = true;
		tmp3.capture = true;
		tempComm.camControl = tmp3;
		tempComm.changedVal = CAM_CHANGED;
		interThreadComm.publish(tempComm);
#endif
		//		PRINTF("and now im here\n");
		//		suspendCallerUntil(END_OF_TIME);
//	}

	PRINTF("SYSTEM HELLO!\n");

	while(1){
		//check which mode
		if(cmd.command == GOTO_MODE){
			currentSystemMode.activeMode = (int) cmd.commandValue;
			PRINTF("here cmd\n");
		} else {
			//			PRINTF("hello, active Mode: %d\n",currentSystemMode.activeMode);
			switch (currentSystemMode.activeMode) {
			case STANDBY:
				// do nothing, only blink a led or some shit
				PRINTF("DALEK waiting for commands!\n");
				break;
			case SUN_FINDING:
#ifdef SUNFINDER_ENABLE
				PRINTF("sun finding mode!\n");
				sunFinder.setActive(true);
#endif
				break;
			case MOTOR_CONTROL:
#ifdef MOTOR_ENABLE
				// seting motor speed,
				PRINTF("motor control mode with command %d\n",cmd.command);

				switch (cmd.command) {
				case SET_ROTATION_SPEED:

					motorControl.setMotorSpeed(cmd.commandValue);
					break;
				case CONTROL_MOTOR:
					if((int)cmd.commandValue == 1){
						motorControl.setMotor(true);
					} else if((int)cmd.commandValue == 0){
						motorControl.setMotor(false);
					}
					break;
				case GOTO_ANGLE:
					break;
				default:

					break;
				}
#endif
				break;
				case MISSION:
					// execute mission
					PRINTF("mission mode!\n");
					break;
				default:
					break;
			}
		}
		suspendCallerUntil(END_OF_TIME);
	}


	RED_TOGGLE;

}

