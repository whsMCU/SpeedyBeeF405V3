################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../src/main/startup/startup_stm32f405rgtx.s 

OBJS += \
./src/main/startup/startup_stm32f405rgtx.o 

S_DEPS += \
./src/main/startup/startup_stm32f405rgtx.d 


# Each subdirectory must supply rules for building sources it contributes
src/main/startup/%.o: ../src/main/startup/%.s src/main/startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-src-2f-main-2f-startup

clean-src-2f-main-2f-startup:
	-$(RM) ./src/main/startup/startup_stm32f405rgtx.d ./src/main/startup/startup_stm32f405rgtx.o

.PHONY: clean-src-2f-main-2f-startup

