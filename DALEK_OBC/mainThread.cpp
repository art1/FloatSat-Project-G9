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
//GPIO_LED leds("LEDs");

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
Camera camera; // needs to be extern because interrupt handler!
#endif
#ifdef MOTOR_ENABLE
MotorControlThread motorControl;
#endif
#ifdef IR_ENABLE
InfraredSensors irSensors;
#endif
#ifdef KNIFE_ENABLE
ThermalKnife knife;
#endif

//RESUMER THREADS
/**************************** IMU MESSAGES **************************************/
#ifdef FUSION_ENABLE
struct receiver_Fusion : public Subscriber, public Thread {
	receiver_Fusion() : Subscriber(imu_rawData,"IMU Raw Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		fusion.newData(*(IMU_DATA_RAW*)data);
#ifdef TTNC_ENABLE
		tm.setNewData(*(IMU_DATA_RAW*)data);
#endif
		fusion.resume();
		return 1;
	}
	void run(){}
} fusion_receiver_thread;

struct receiver_filtered : public Subscriber, public Thread {
	receiver_filtered() : Subscriber(imu_filtered,"IMU Filtered Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
#ifdef TTNC_ENABLE
		tm.setNewData(*(IMU_RPY_FILTERED*)data);
#endif
		return 1;
	}
	void run(){}
} filtered_receiver_thread;
#endif
/**************************** LIGHT MESSAGES ************************************/
#ifdef LIGHT_ENABLE
struct receiver_light : public Subscriber, public Thread {
	receiver_light() : Subscriber(lux_data,"Lux Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		light.setActive(*(LUX_DATA*)data);
#ifdef TTNC_ENABLE
		tm.setNewData(*(LUX_DATA*)data);
#endif
		light.resume();
		return 1;
	}
	void run(){}
} light_receiver_thread;
#endif
/**************************** Solar Panel MESSAGES ************************************/
#ifdef SOLAR_ADC_ENABLE
struct receiver_solar : public Subscriber, public Thread {
	receiver_solar() : Subscriber(solar_data,"Solar Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		solar.setActive(*(SOLAR_DATA*)data);
#ifdef TTNC_ENABLE
		tm.setNewData(*(SOLAR_DATA*)data);
#endif
		solar.resume();
		return 1;
	}
	void run(){}
} solar_receiver_thread;
#endif
/**************************** IR Sensors MESSAGES ************************************/
#ifdef IR_ENABLE
struct receiver_irSensors : public Subscriber, public Thread {
	receiver_irSensors() : Subscriber(ir_data,"Infrared Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		irSensors.setActive(*(IR_DATA*)data);
#ifdef TTNC_ENABLE
		tm.setNewData(*(IR_DATA*) data);
#endif
		irSensors.resume();
		return 1;
	}
	void run(){}
} ir_sensors_receiver_thread;
#endif
/**************************** TTnC MESSAGES **************************************/
#ifdef KNIFE_ENABLE
struct receiver_knife : public Subscriber, public Thread {
	receiver_knife() : Subscriber(knife_data,"Knife Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		knife.setNewData(*(KNIFE_DATA*)data);
		return 1;
	}
	void run(){}
} knife_receiver_thread;
#endif
/**************************** Camera MESSAGES *****************************************/
#ifdef CAMERA_ENABLE
struct receiver_camera : public Subscriber, public Thread {
	receiver_camera() : Subscriber(cam_control,"Camera Control") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		camera.setNewData(*(CAM_CONTROL*)data);
		camera.resume();
		return 1;
	}
	void run(){}
} camera_receiver_thread;
#endif
/**************************** TTnC MESSAGES **************************************/
#ifdef TTNC_ENABLE
struct receiver_telecommand : public Subscriber, public Thread {
	receiver_telecommand() : Subscriber(tcRaw,"TC Raw Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		tc.setNewData(*(UDPMsg*)data);
//		PRINTF("im here\n");
		tc.resume();
		return 1;
	}
	void run(){}
} tc_receiver_thread;
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
	receiver_tcControl() : Subscriber(commandFrame,"TC Control Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		//		this.setNewData(*(COMMAND_FRAME*)data);
		mainT.setNewData(*(COMMAND_FRAME*)data);
//		PRINTF("setting control data\n");

		//		control.resume();
		mainT.resume();
		return 1;
	}
	void run(){}
} tcControl_receiver_thread;
#endif

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
	currentSystemMode.activeMode = MOTOR_CONTROL;
	cmd.command = -1;

}


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

#ifdef CAMERA_ENABLE
	PRINTF("enabling cam\n");
	CAM_CONTROL tmp3;
	tmp3.activateCamera = true;
	tmp3.capture = true;
#endif
#ifdef LIGHT_ENABLE
	i2c1.init(400000);
	LUX_DATA temp;
	temp.activated = true;
	lux_data.publish(temp);
#endif

#ifdef IR_ENABLE
	IR_DATA tmp2;
	irSensors.init();
	tmp2.activated = true;
	//	PRINTF("publishing Data\n");
	ir_data.publish(tmp2);
#endif

	while(1){

#ifdef CAMERA_ENABLE
		PRINTF("enabling cam in 1 secs...\n");
		Delay_millis(100);
		PRINTF("should be enabled in a few msecs\n");
		cam_control.publish(tmp3);
#endif
		PRINTF("and now im here\n");
		suspendCallerUntil(END_OF_TIME);
	}


	while(1){
		//check which mode
		PRINTF("SYSTEM HELLO!\n");
		if(cmd.command == GOTO_MODE){
			currentSystemMode.activeMode = (int) cmd.commandValue;
			PRINTF("here cmd\n");
		} else {
			PRINTF("hello active %d\n",currentSystemMode.activeMode);
			switch (currentSystemMode.activeMode) {
			case STANDBY:
				// do nothing, only blink a led or some shit
				PRINTF("waiting for the DALEK Brain for commands!\n");
				break;
			case SUN_FINDING:
				// here: search for sun,
				PRINTF("sun finding mode!\n");
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
					PRINTF("default shit\n");
					break;
			}
		}
		suspendCallerUntil(END_OF_TIME);
	}


	RED_TOGGLE;

}

