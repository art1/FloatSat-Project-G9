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
ControlThread control;
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
HAL_GPIO HBRIDGE_EN(GPIO_066);

//RESUMER THREADS
///**************************** GPIO MESSAGES *************************************/
//CommBuffer<LED_SWITCH> inputMessageBuffer;
//struct GPIO_Receiver :  public Subscriber,  public Thread  {
//  GPIO_Receiver() : Subscriber(led_switch,"led switcher") { }
//
//  long put(const long topicId, const long len, const void* data, const NetMsgInfo& netMsgInfo) {
//      leds.setNextMode(*(LED_SWITCH*)data);
//      leds.resume();                         // not to publish from interrupt, call a thread to do it
//      return 1;
//   }
//
//  void run () { }
//} gpio_receiver_thread;
/**************************** IMU MESSAGES **************************************/
#ifdef FUSION_ENABLE
struct receiver_Fusion : public Subscriber, public Thread {
	receiver_Fusion() : Subscriber(imu_rawData,"IMU Raw Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		fusion.newData(*(IMU_DATA_RAW*)data);
		tm.setNewData(*(IMU_DATA_RAW*)data);
		fusion.resume();
//		PRINTF("some stuff there!\n\n");
		return 1;
	}
	void run(){}
} fusion_receiver_thread;

struct receiver_filtered : public Subscriber, public Thread {
	receiver_filtered() : Subscriber(imu_filtered,"IMU Filtered Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		tm.setNewData(*(IMU_RPY_FILTERED*)data);
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
		tm.setNewData(*(LUX_DATA*)data);
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
		tm.setNewData(*(SOLAR_DATA*)data);
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
		tm.setNewData(*(IR_DATA*) data);
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
} ir_sensors_receiver_thread;
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
	receiver_tcControl() : Subscriber(commandFrame,"TC COntrol Data") {}
	long put(const long topicId, const long len,const void* data, const NetMsgInfo& netMsgInfo){
		control.setNewData(*(COMMAND_FRAME*)data);
		control.resume();
		return 1;
	}
	void run(){}
} tcControl_receiver_thread;
#endif

mainThread::mainThread(const char* name) : Thread(name){

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


}


void mainThread::run(){

#ifdef BLUETOOTH_FALLBACK
	bt_uart.init(BLUETOOTH_BAUDRATE);
#endif
    HBRIDGE_EN.init(true, 1, 1);

	//enable ADC1 channel with 12 Bit Resolution
	adc1.config(ADC_PARAMETER_RESOLUTION,ADC1_RESOLUTION);
	#ifdef IMU_ENABLE
	imu.regInit();
	imu.setTime(500*MILLISECONDS);
	#endif

#ifdef LIGHT_ENABLE
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
#ifdef CAMERA_ENABLE
	PRINTF("enabling cam\n");
	CAM_CONTROL tmp3;
	tmp3.shoot = true;
	cam_control.publish(tmp3);
#endif


	while(1){
		suspendCallerUntil(NOW()+500*MILLISECONDS);
		RED_TOGGLE;
	}
}

