################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main/flight/dyn_notch_filter.c \
../src/main/flight/failsafe.c \
../src/main/flight/feedforward.c \
../src/main/flight/imu.c \
../src/main/flight/mixer.c \
../src/main/flight/mixer_init.c \
../src/main/flight/pid.c \
../src/main/flight/pid_init.c \
../src/main/flight/position.c 

OBJS += \
./src/main/flight/dyn_notch_filter.o \
./src/main/flight/failsafe.o \
./src/main/flight/feedforward.o \
./src/main/flight/imu.o \
./src/main/flight/mixer.o \
./src/main/flight/mixer_init.o \
./src/main/flight/pid.o \
./src/main/flight/pid_init.o \
./src/main/flight/position.o 

C_DEPS += \
./src/main/flight/dyn_notch_filter.d \
./src/main/flight/failsafe.d \
./src/main/flight/feedforward.d \
./src/main/flight/imu.d \
./src/main/flight/mixer.d \
./src/main/flight/mixer_init.d \
./src/main/flight/pid.d \
./src/main/flight/pid_init.d \
./src/main/flight/position.d 


# Each subdirectory must supply rules for building sources it contributes
src/main/flight/%.o src/main/flight/%.su src/main/flight/%.cyclo: ../src/main/flight/%.c src/main/flight/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DSTM32F40_41xxx -c -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/Third_Party/FatFs/src" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/App" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/Target" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/common" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/config" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/scheduler" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/accgyro" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/barometer" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/BMI270-Sensor-API" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/compass" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/gps" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/sdcard" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/fc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/flight" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/hw" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/io" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/msp" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/rx" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/sensors" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/target" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-main-2f-flight

clean-src-2f-main-2f-flight:
	-$(RM) ./src/main/flight/dyn_notch_filter.cyclo ./src/main/flight/dyn_notch_filter.d ./src/main/flight/dyn_notch_filter.o ./src/main/flight/dyn_notch_filter.su ./src/main/flight/failsafe.cyclo ./src/main/flight/failsafe.d ./src/main/flight/failsafe.o ./src/main/flight/failsafe.su ./src/main/flight/feedforward.cyclo ./src/main/flight/feedforward.d ./src/main/flight/feedforward.o ./src/main/flight/feedforward.su ./src/main/flight/imu.cyclo ./src/main/flight/imu.d ./src/main/flight/imu.o ./src/main/flight/imu.su ./src/main/flight/mixer.cyclo ./src/main/flight/mixer.d ./src/main/flight/mixer.o ./src/main/flight/mixer.su ./src/main/flight/mixer_init.cyclo ./src/main/flight/mixer_init.d ./src/main/flight/mixer_init.o ./src/main/flight/mixer_init.su ./src/main/flight/pid.cyclo ./src/main/flight/pid.d ./src/main/flight/pid.o ./src/main/flight/pid.su ./src/main/flight/pid_init.cyclo ./src/main/flight/pid_init.d ./src/main/flight/pid_init.o ./src/main/flight/pid_init.su ./src/main/flight/position.cyclo ./src/main/flight/position.d ./src/main/flight/position.o ./src/main/flight/position.su

.PHONY: clean-src-2f-main-2f-flight

