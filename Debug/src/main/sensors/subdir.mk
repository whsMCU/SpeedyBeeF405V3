################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main/sensors/acceleration.c \
../src/main/sensors/acceleration_init.c \
../src/main/sensors/adcinternal.c \
../src/main/sensors/barometer.c \
../src/main/sensors/battery.c \
../src/main/sensors/boardalignment.c \
../src/main/sensors/compass.c \
../src/main/sensors/current.c \
../src/main/sensors/gyro.c \
../src/main/sensors/gyro_init.c \
../src/main/sensors/sensors.c \
../src/main/sensors/voltage.c 

OBJS += \
./src/main/sensors/acceleration.o \
./src/main/sensors/acceleration_init.o \
./src/main/sensors/adcinternal.o \
./src/main/sensors/barometer.o \
./src/main/sensors/battery.o \
./src/main/sensors/boardalignment.o \
./src/main/sensors/compass.o \
./src/main/sensors/current.o \
./src/main/sensors/gyro.o \
./src/main/sensors/gyro_init.o \
./src/main/sensors/sensors.o \
./src/main/sensors/voltage.o 

C_DEPS += \
./src/main/sensors/acceleration.d \
./src/main/sensors/acceleration_init.d \
./src/main/sensors/adcinternal.d \
./src/main/sensors/barometer.d \
./src/main/sensors/battery.d \
./src/main/sensors/boardalignment.d \
./src/main/sensors/compass.d \
./src/main/sensors/current.d \
./src/main/sensors/gyro.d \
./src/main/sensors/gyro_init.d \
./src/main/sensors/sensors.d \
./src/main/sensors/voltage.d 


# Each subdirectory must supply rules for building sources it contributes
src/main/sensors/%.o src/main/sensors/%.su src/main/sensors/%.cyclo: ../src/main/sensors/%.c src/main/sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DSTM32F40_41xxx -c -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/Third_Party/FatFs/src" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/App" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/Target" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/common" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/config" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/scheduler" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/accgyro" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/barometer" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/BMI270-Sensor-API" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/compass" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/gps" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/sdcard" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/fc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/flight" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/hw" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/io" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/msp" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/rx" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/sensors" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/target" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-main-2f-sensors

clean-src-2f-main-2f-sensors:
	-$(RM) ./src/main/sensors/acceleration.cyclo ./src/main/sensors/acceleration.d ./src/main/sensors/acceleration.o ./src/main/sensors/acceleration.su ./src/main/sensors/acceleration_init.cyclo ./src/main/sensors/acceleration_init.d ./src/main/sensors/acceleration_init.o ./src/main/sensors/acceleration_init.su ./src/main/sensors/adcinternal.cyclo ./src/main/sensors/adcinternal.d ./src/main/sensors/adcinternal.o ./src/main/sensors/adcinternal.su ./src/main/sensors/barometer.cyclo ./src/main/sensors/barometer.d ./src/main/sensors/barometer.o ./src/main/sensors/barometer.su ./src/main/sensors/battery.cyclo ./src/main/sensors/battery.d ./src/main/sensors/battery.o ./src/main/sensors/battery.su ./src/main/sensors/boardalignment.cyclo ./src/main/sensors/boardalignment.d ./src/main/sensors/boardalignment.o ./src/main/sensors/boardalignment.su ./src/main/sensors/compass.cyclo ./src/main/sensors/compass.d ./src/main/sensors/compass.o ./src/main/sensors/compass.su ./src/main/sensors/current.cyclo ./src/main/sensors/current.d ./src/main/sensors/current.o ./src/main/sensors/current.su ./src/main/sensors/gyro.cyclo ./src/main/sensors/gyro.d ./src/main/sensors/gyro.o ./src/main/sensors/gyro.su ./src/main/sensors/gyro_init.cyclo ./src/main/sensors/gyro_init.d ./src/main/sensors/gyro_init.o ./src/main/sensors/gyro_init.su ./src/main/sensors/sensors.cyclo ./src/main/sensors/sensors.d ./src/main/sensors/sensors.o ./src/main/sensors/sensors.su ./src/main/sensors/voltage.cyclo ./src/main/sensors/voltage.d ./src/main/sensors/voltage.o ./src/main/sensors/voltage.su

.PHONY: clean-src-2f-main-2f-sensors

