################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main/telemetry/crsf.c \
../src/main/telemetry/telemetry.c 

OBJS += \
./src/main/telemetry/crsf.o \
./src/main/telemetry/telemetry.o 

C_DEPS += \
./src/main/telemetry/crsf.d \
./src/main/telemetry/telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
src/main/telemetry/%.o src/main/telemetry/%.su src/main/telemetry/%.cyclo: ../src/main/telemetry/%.c src/main/telemetry/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DSTM32F40_41xxx -c -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/Third_Party/FatFs/src" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/App" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/Target" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/common" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/config" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/scheduler" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/accgyro" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/barometer" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/BMI270-Sensor-API" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/compass" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/gps" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/sdcard" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/fc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/flight" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/hw" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/io" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/msp" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/rx" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/sensors" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/target" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-main-2f-telemetry

clean-src-2f-main-2f-telemetry:
	-$(RM) ./src/main/telemetry/crsf.cyclo ./src/main/telemetry/crsf.d ./src/main/telemetry/crsf.o ./src/main/telemetry/crsf.su ./src/main/telemetry/telemetry.cyclo ./src/main/telemetry/telemetry.d ./src/main/telemetry/telemetry.o ./src/main/telemetry/telemetry.su

.PHONY: clean-src-2f-main-2f-telemetry

