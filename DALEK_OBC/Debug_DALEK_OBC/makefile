################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Hardware/Motor/subdir.mk
-include Hardware/Camera/Supps/subdir.mk
-include Hardware/Camera/subdir.mk
-include Hardware/Audio/subdir.mk
-include Hardware/AHRS/Filter/subdir.mk
-include Hardware/AHRS/subdir.mk
-include Hardware/subdir.mk
-include Communication/subdir.mk
-include Basic/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
DALEK_OBC.hex \

SECONDARY_SIZE += \
DALEK_OBC.siz \


# All Target
all: DALEK_OBC.elf secondary-outputs

# Tool invocations
DALEK_OBC.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -std=c++11 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -T "/Users/arthur/Documents/Coding/CanSat/workspace_cansat/rodos/src/bare-metal/stm32f4/scripts/stm32_flash.ld" -nostartfiles -nodefaultlibs -nostdlib -Xlinker --gc-sections -L"/Users/arthur/Documents/Coding/CanSat/workspace_cansat/rodos/Debug_STM32F4" -L"/Users/arthur/Documents/Coding/CanSat/workspace_cansat/support_libs/Debug_STM32F4" -Wl,-Map,"DALEK_OBC.map" -g -O0 -o "DALEK_OBC.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

DALEK_OBC.hex: DALEK_OBC.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "DALEK_OBC.elf"  "DALEK_OBC.hex"
	@echo 'Finished building: $@'
	@echo ' '

DALEK_OBC.siz: DALEK_OBC.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "DALEK_OBC.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(OBJS)$(C_UPPER_DEPS)$(CXX_DEPS)$(SECONDARY_FLASH)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS)$(CPP_DEPS) DALEK_OBC.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
