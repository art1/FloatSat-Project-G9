#include <new>
#include "rodos.h"
#include "hal/hal_gpio.h"

#include "stm32f4xx_gpio.h"

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif


void initEXTInterrupts(){
	static bool init=false;
	if(!init){
		init=true;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

		NVIC_EnableIRQ(EXTI0_IRQn);
		NVIC_EnableIRQ(EXTI1_IRQn);
		NVIC_EnableIRQ(EXTI2_IRQn);
		NVIC_EnableIRQ(EXTI3_IRQn);
		NVIC_EnableIRQ(EXTI4_IRQn);
		NVIC_EnableIRQ(EXTI9_5_IRQn);
		NVIC_EnableIRQ(EXTI15_10_IRQn);
	}
}



class HW_HAL_GPIO {
public:
	HW_HAL_GPIO(GPIO_PIN pinIdx, uint8_t numOfPins, bool isOutput):
		pinIdx(pinIdx),numOfPins(numOfPins),isOutput(isOutput){
		setPinMask();
		setGPIOPortVars();

		GPIO_StructInit(&GPIO_InitStruct);
		GPIO_InitStruct.GPIO_Pin = pinMask;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;

		irqSensitivity = GPIO_IRQ_SENS_BOTH;
		interruptEventOcured = false;
	};

	GPIO_PIN pinIdx;
	HAL_GPIO* hal_gpio;
	uint8_t numOfPins;
	bool isOutput;
	uint16_t pinMask;

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_TypeDef *PORT;
	uint32_t RCC_AHB1Periph;

	//int32_t extiLine;
	GPIO_IRQ_SENSITIVITY irqSensitivity;
	bool interruptEventOcured;

	void setPinMask();
	void EXTIRQHandler();

private:
	void setGPIOPortVars();
};


HW_HAL_GPIO* extInterruptLines[16];

void HW_HAL_GPIO::setPinMask(){
	if (numOfPins+(pinIdx & 0xF) > 16) { // pin-group exceeds port boundary ! only the pins up to most significant pin of port will be set
		pinMask = 0xFFFF << (pinIdx & 0xF);
	}
	else{
		pinMask = 0xFFFF >> (16 - numOfPins);
		pinMask = pinMask << (pinIdx&0xF);
	}
}

void HW_HAL_GPIO::setGPIOPortVars(){
	if(pinIdx < 16){
		PORT = GPIOA;
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOA;
	}
	else if(pinIdx < 32){
		PORT = GPIOB;
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOB;
	}
	else if(pinIdx < 48){
		PORT = GPIOC;
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOC;
	}
	else if(pinIdx < 64){
		PORT = GPIOD;
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOD;
	}
	else if(pinIdx < 80){
		PORT = GPIOE;
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOE;
	}
	else if(pinIdx < 96){
		PORT = GPIOF;
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOF;
	}
	else if(pinIdx < 112){
		PORT = GPIOG;
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOG;
	}
	else if(pinIdx < 128){
		PORT = GPIOH;
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOH;
	}
	else if(pinIdx < 140){
		PORT = GPIOI;
		RCC_AHB1Periph = RCC_AHB1Periph_GPIOI;
	}
	else{
		PORT = NULL;
		RCC_AHB1Periph = 0;
	}
}

void HW_HAL_GPIO::EXTIRQHandler(){
	interruptEventOcured=true;
	hal_gpio->upCallDataReady();
	EXTI->PR = 1 << (pinIdx % 16);
}

HAL_GPIO::HAL_GPIO(GPIO_PIN pinIdx) {
	//context = new HW_HAL_GPIO(pinIdx,1,false);
	context = (HW_HAL_GPIO*)xmalloc(sizeof(HW_HAL_GPIO)); // dynamic memory allocation with RODOS function
	context = new (context) HW_HAL_GPIO(pinIdx,1,false); // placement new to call constructor
	context->hal_gpio = this;
}


int32_t HAL_GPIO::init(bool isOutput, uint32_t numOfPins, uint32_t initVal){
	if (numOfPins > 0) context->numOfPins = numOfPins; // numOfPins has to be > 0 -> if new value is 0 keep the default value

	context->isOutput = isOutput;
	context->setPinMask();

	if (context->PORT == NULL) return -1;

	RCC_AHB1PeriphClockCmd(context->RCC_AHB1Periph, ENABLE);

	if (context->isOutput){
		config(GPIO_CFG_OUTPUT_ENABLE, 1);
		setPins(initVal);
	} else {
		config(GPIO_CFG_OUTPUT_ENABLE, 0);
	}

	initEXTInterrupts();
	return 0;
}


int32_t HAL_GPIO::config(GPIO_CFG_TYPE type, uint32_t paramVal){
	switch (type){
		case GPIO_CFG_OUTPUT_ENABLE:
			if (paramVal > 0){
				context->isOutput = true;
				context->GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
			}
			else {
				context->isOutput = false;
				context->GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
			}
			GPIO_Init(context->PORT, &context->GPIO_InitStruct);
			return 0;

		case GPIO_CFG_NUM_OF_PINS:
			if (paramVal > 0 && paramVal < 256) { // numOfPins has to be > 0 and < 256 -> uint8_t
				context->numOfPins = paramVal;
				context->setPinMask();
				context->GPIO_InitStruct.GPIO_Pin = context->pinMask;
				GPIO_Init(context->PORT, &context->GPIO_InitStruct);
				return 0;
			}
			return -1;

		case GPIO_CFG_PULLUP_ENABLE:
			if (paramVal > 0){
				if (paramVal){
					context->GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
				}else{
					context->GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
				}
				GPIO_Init(context->PORT, &context->GPIO_InitStruct);
				return 0;
			}
			return -1;

		case GPIO_CFG_PULLDOWN_ENABLE:
			if (paramVal > 0){
				if (paramVal){
					context->GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
				}else{
					context->GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
				}
				GPIO_Init(context->PORT, &context->GPIO_InitStruct);
				return 0;
			}
			return -1;
		case GPIO_CFG_IRQ_SENSITIVITY:
			if (paramVal >= 0 && paramVal <= GPIO_IRQ_SENS_FALLING){
				context->irqSensitivity=(RODOS::GPIO_IRQ_SENSITIVITY)paramVal;
				return 0;
			}
			return -1;

		default: return -1;
	}
}


void HAL_GPIO::reset(){
	interruptEnable(false);
	config(GPIO_CFG_OUTPUT_ENABLE, 0);
	GPIO_DeInit(context->PORT);
	RCC_AHB1PeriphClockCmd(context->RCC_AHB1Periph, DISABLE);
}


void HAL_GPIO::setPins(uint32_t val) {
	uint16_t newPinVal = 0;

	if (context->isOutput){
		PRIORITY_CEILING{
			//read the whole port, change only the selected pins and write the value
			newPinVal = GPIO_ReadOutputData(context->PORT) & ~context->pinMask; // get current pinvalues of whole port and clear pinvalues we want to set new
			newPinVal |= (val << (context->pinIdx & 0x0F) ) & context->pinMask; // set new pinvalues
			GPIO_Write(context->PORT,newPinVal);
		}
	}
}


uint32_t HAL_GPIO::readPins(){
	if (context->isOutput){
		return (GPIO_ReadOutputData(context->PORT) & context->pinMask) >> (context->pinIdx & 0xF);
	}
	return (GPIO_ReadInputData(context->PORT) & context->pinMask) >> (context->pinIdx & 0xF);
}


bool HAL_GPIO::isDataReady(){
	return context->interruptEventOcured;
}

//When rising and/or falling edge occures dataReady() == true
void HAL_GPIO::interruptEnable(bool enable){
	int portNum = context->pinIdx / 16;
	int  pinNum = context->pinIdx % 16;
	int32_t exti=pinNum;

	if(enable){//enable Interrupt
		if(extInterruptLines[exti]==context){
			return; //Already enabled
		}else if(extInterruptLines[exti]==0){
			if(context->numOfPins > 1){
				ERROR("IRQ not possible with numOfPins > 1");
				return;
			}

			SYSCFG_EXTILineConfig(portNum,pinNum);

			if(context->irqSensitivity == GPIO_IRQ_SENS_RISING  || context->irqSensitivity == GPIO_IRQ_SENS_BOTH){
				EXTI->RTSR |= 1 << pinNum;
			}
			if(context->irqSensitivity == GPIO_IRQ_SENS_FALLING || context->irqSensitivity == GPIO_IRQ_SENS_BOTH){
				EXTI->FTSR |= 1 << pinNum;
			}

			extInterruptLines[exti]=context;
			context->interruptEventOcured=false;
			EXTI->IMR |= 1 << pinNum;

		}else{
			ERROR("External IRQ Line already used by another HAL_GPIO");
			return;
		}


	}else{//disable Interrupt
		if(extInterruptLines[exti]==context){
			EXTI->IMR &= ~(1 << pinNum);
			EXTI->RTSR &= ~(1 << pinNum);
			EXTI->FTSR &= ~(1 << pinNum);
			extInterruptLines[exti]=0;
		}
	}

}

// dataReady == false
void HAL_GPIO::resetInterruptEventStatus(){
	context->interruptEventOcured=false;
}


#ifndef NO_RODOS_NAMESPACE
}
#endif



extern "C" {
	void EXTI0_IRQHandler(){
		if(extInterruptLines[0]){
			extInterruptLines[0]->EXTIRQHandler();
		}
		NVIC_ClearPendingIRQ(EXTI0_IRQn);
	}
	void EXTI1_IRQHandler(){
		if(extInterruptLines[1]){
			extInterruptLines[1]->EXTIRQHandler();
		}
		NVIC_ClearPendingIRQ(EXTI1_IRQn);
	}
	void EXTI2_IRQHandler(){
		if(extInterruptLines[2]){
			extInterruptLines[2]->EXTIRQHandler();
		}
		NVIC_ClearPendingIRQ(EXTI2_IRQn);
	}
	void EXTI3_IRQHandler(){
		if(extInterruptLines[3]){
			extInterruptLines[3]->EXTIRQHandler();
		}
		NVIC_ClearPendingIRQ(EXTI3_IRQn);
	}
	void EXTI4_IRQHandler(){
		if(extInterruptLines[4]){
			extInterruptLines[4]->EXTIRQHandler();
		}
		NVIC_ClearPendingIRQ(EXTI4_IRQn);
	}

	void EXTI9_5_IRQHandler(){
		uint32_t pending = EXTI->PR;
		for(int i=5;i<=9;i++){
			if(((pending >> i) & 0x01) && extInterruptLines[i]){
				extInterruptLines[i]->EXTIRQHandler();
			}
		}
		NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	}
	void EXTI15_10_IRQHandler(){
		uint32_t pending = EXTI->PR;
		for(int i=10;i<=15;i++){
			if(((pending >> i) & 0x01) && extInterruptLines[i]){
				extInterruptLines[i]->EXTIRQHandler();
			}
		}
		NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	}
}
