################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main/drivers/BMI270-Sensor-API/bmi2.c \
../src/main/drivers/BMI270-Sensor-API/bmi270.c \
../src/main/drivers/BMI270-Sensor-API/bmi270_context.c \
../src/main/drivers/BMI270-Sensor-API/bmi270_hc.c \
../src/main/drivers/BMI270-Sensor-API/bmi270_maximum_fifo.c \
../src/main/drivers/BMI270-Sensor-API/bmi270_wh.c \
../src/main/drivers/BMI270-Sensor-API/bmi2_ois.c 

OBJS += \
./src/main/drivers/BMI270-Sensor-API/bmi2.o \
./src/main/drivers/BMI270-Sensor-API/bmi270.o \
./src/main/drivers/BMI270-Sensor-API/bmi270_context.o \
./src/main/drivers/BMI270-Sensor-API/bmi270_hc.o \
./src/main/drivers/BMI270-Sensor-API/bmi270_maximum_fifo.o \
./src/main/drivers/BMI270-Sensor-API/bmi270_wh.o \
./src/main/drivers/BMI270-Sensor-API/bmi2_ois.o 

C_DEPS += \
./src/main/drivers/BMI270-Sensor-API/bmi2.d \
./src/main/drivers/BMI270-Sensor-API/bmi270.d \
./src/main/drivers/BMI270-Sensor-API/bmi270_context.d \
./src/main/drivers/BMI270-Sensor-API/bmi270_hc.d \
./src/main/drivers/BMI270-Sensor-API/bmi270_maximum_fifo.d \
./src/main/drivers/BMI270-Sensor-API/bmi270_wh.d \
./src/main/drivers/BMI270-Sensor-API/bmi2_ois.d 


# Each subdirectory must supply rules for building sources it contributes
src/main/drivers/BMI270-Sensor-API/%.o src/main/drivers/BMI270-Sensor-API/%.su src/main/drivers/BMI270-Sensor-API/%.cyclo: ../src/main/drivers/BMI270-Sensor-API/%.c src/main/drivers/BMI270-Sensor-API/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DSTM32F40_41xxx -c -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/Third_Party/FatFs/src" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/App" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/Target" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/common" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/config" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/scheduler" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/accgyro" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/barometer" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/BMI270-Sensor-API" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/compass" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/gps" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/sdcard" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/fc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/flight" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/hw" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/io" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/msp" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/rx" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/sensors" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/target" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-main-2f-drivers-2f-BMI270-2d-Sensor-2d-API

clean-src-2f-main-2f-drivers-2f-BMI270-2d-Sensor-2d-API:
	-$(RM) ./src/main/drivers/BMI270-Sensor-API/bmi2.cyclo ./src/main/drivers/BMI270-Sensor-API/bmi2.d ./src/main/drivers/BMI270-Sensor-API/bmi2.o ./src/main/drivers/BMI270-Sensor-API/bmi2.su ./src/main/drivers/BMI270-Sensor-API/bmi270.cyclo ./src/main/drivers/BMI270-Sensor-API/bmi270.d ./src/main/drivers/BMI270-Sensor-API/bmi270.o ./src/main/drivers/BMI270-Sensor-API/bmi270.su ./src/main/drivers/BMI270-Sensor-API/bmi270_context.cyclo ./src/main/drivers/BMI270-Sensor-API/bmi270_context.d ./src/main/drivers/BMI270-Sensor-API/bmi270_context.o ./src/main/drivers/BMI270-Sensor-API/bmi270_context.su ./src/main/drivers/BMI270-Sensor-API/bmi270_hc.cyclo ./src/main/drivers/BMI270-Sensor-API/bmi270_hc.d ./src/main/drivers/BMI270-Sensor-API/bmi270_hc.o ./src/main/drivers/BMI270-Sensor-API/bmi270_hc.su ./src/main/drivers/BMI270-Sensor-API/bmi270_maximum_fifo.cyclo ./src/main/drivers/BMI270-Sensor-API/bmi270_maximum_fifo.d ./src/main/drivers/BMI270-Sensor-API/bmi270_maximum_fifo.o ./src/main/drivers/BMI270-Sensor-API/bmi270_maximum_fifo.su ./src/main/drivers/BMI270-Sensor-API/bmi270_wh.cyclo ./src/main/drivers/BMI270-Sensor-API/bmi270_wh.d ./src/main/drivers/BMI270-Sensor-API/bmi270_wh.o ./src/main/drivers/BMI270-Sensor-API/bmi270_wh.su ./src/main/drivers/BMI270-Sensor-API/bmi2_ois.cyclo ./src/main/drivers/BMI270-Sensor-API/bmi2_ois.d ./src/main/drivers/BMI270-Sensor-API/bmi2_ois.o ./src/main/drivers/BMI270-Sensor-API/bmi2_ois.su

.PHONY: clean-src-2f-main-2f-drivers-2f-BMI270-2d-Sensor-2d-API

