/*
 * currentSensors.cpp
 *
 *  Created on: Jan 4, 2016
 *      Author: arthur
 */

#include "currentSensors.h"

currentSensors::currentSensors() : Thread("Current Sensor Thread",111,1000) {

	current.changedVal = CURRENT_CHANGED;
}

currentSensors::~currentSensors() {

}

void currentSensors::init(){

}

void currentSensors::run(){
	suspendCallerUntil(NOW()+2000*MILLISECONDS);

	configSensors();


	while(1){

		readRawData();
		interThreadComm.publish(current);
//		PRINTF("Current: %f, Voltage: %f\n",current.currentData.batteryCurrent,current.currentData.batteryVoltage);
		suspendCallerUntil(NOW()+CURRENT_SAMPLERATE*MILLISECONDS);
	}
}

void currentSensors::configSensors(){
	// Using 32V
	txBuf[0] = CURRENT_CONFIG_REG; // 3C 1F
	txBuf[1] = 0x59; 	// upper byte of configuration register -> set range to 32V
	txBuf[2] = 0x9f;	// lower byte of config register ->continuous
	i2c1.write(CURRENT_SENSOR_ADRESS,txBuf,3);

	// VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
	// VSHUNT_MAX = 15          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
	// RSHUNT = 0.002               (Resistor value in ohms)

	// 1. Determine max possible current
	// MaxPossible_I = VSHUNT_MAX / RSHUNT
	// MaxPossible_I = 7500A

	// 2. Determine max expected current
	// MaxExpected_I = 4.0A

	// 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	// MinimumLSB = MaxExpected_I/32767
	// MinimumLSB = 0.0001220740379 	//0.00006103701895              (122uA per bit)
	// MaximumLSB = MaxExpected_I/4096
	// MaximumLSB = 0.0009765625      //0.00048828125	              (977uA per bit)

	// 4. Choose an LSB between the min and max values
	//    (Preferrably a roundish number close to MinLSB)
	// CurrentLSB = 0.0002 (200uA per bit)

	// 5. Compute the calibration register
	// Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
	// Cal = 10240 (0x1000)

	calVal = 10240; //(high byte 0x29, low byte 0x00)

	// 6. Calculate the power LSB
	// PowerLSB = 20 * CurrentLSB
	// PowerLSB = 0.004 (4mW per bit)

	// 7. Compute the maximum current and shunt voltage values before overflow
	//
	// Max_Current = Current_LSB * 32767
	// Max_Current = 131.068A before overflow
	//
	// If Max_Current > Max_Possible_I then
	//    Max_Current_Before_Overflow = MaxPossible_I
	// Else
	//    Max_Current_Before_Overflow = Max_Current
	// End If
	//
	// Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
	// Max_ShuntVoltage = 0.008V
	//
	// If Max_ShuntVoltage >= VSHUNT_MAX
	//    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
	// Else
	//    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
	// End If

	// 8. Compute the Maximum Power
	// MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
	// MaximumPower = 4 * 32V
	// MaximumPower = 128W

	// Set multipliers to convert raw current/power values
	currentDivider_mA = 1;  // Current LSB = 100uA per bit (1000/100 = 10)
	/** TODO proper currentDivider to mA */
	powerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)

	txBuf[0] = CURRENT_CALIBRATION;
	txBuf[1] = (calVal >> 8) & 0xFF;
	txBuf[2] = calVal & 0xFF;
	i2c1.write(CURRENT_SENSOR_ADRESS,txBuf,3);


}

void currentSensors::readRawData(){

	txBuf[0] = CURRENT_BUS_VOLTAGE;
	i2c1.writeRead(CURRENT_SENSOR_ADRESS,txBuf,1,rxBuf,2);

	current.currentData.batteryVoltage  =
			(((((uint16_t) rxBuf[0] << 8) | rxBuf[1]) >> 3) * 4) * 0.001;	// Voltage in Volt

	// Sometimes a sharp load will reset the INA219, which will
	// reset the cal register, meaning CURRENT and POWER will
	// not be available ... avoid this by always setting a cal
	// value even if it's an unfortunate extra step
//	configSensors();

	txBuf[0] = CURRENT_CALIBRATION;
	txBuf[1] = (calVal >> 8) & 0xFF;
	txBuf[2] = calVal & 0xFF;
	i2c1.write(CURRENT_SENSOR_ADRESS,txBuf,3);

	txBuf[0] = CURRENT_CURRENT_REG;
	i2c1.writeRead(CURRENT_SENSOR_ADRESS,txBuf,1,rxBuf,2);

	current.currentData.batteryCurrent =
			((float)(((uint16_t) rxBuf[0] << 8) | rxBuf[1])) / (float)currentDivider_mA;	// Current in mA
}

