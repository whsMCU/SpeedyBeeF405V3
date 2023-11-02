################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main/drivers/buf_writer.c \
../src/main/drivers/display.c \
../src/main/drivers/display_canvas.c \
../src/main/drivers/max7456.c \
../src/main/drivers/motor.c \
../src/main/drivers/pwm_output.c \
../src/main/drivers/stack_check.c 

OBJS += \
./src/main/drivers/buf_writer.o \
./src/main/drivers/display.o \
./src/main/drivers/display_canvas.o \
./src/main/drivers/max7456.o \
./src/main/drivers/motor.o \
./src/main/drivers/pwm_output.o \
./src/main/drivers/stack_check.o 

C_DEPS += \
./src/main/drivers/buf_writer.d \
./src/main/drivers/display.d \
./src/main/drivers/display_canvas.d \
./src/main/drivers/max7456.d \
./src/main/drivers/motor.d \
./src/main/drivers/pwm_output.d \
./src/main/drivers/stack_check.d 


# Each subdirectory must supply rules for building sources it contributes
src/main/drivers/%.o src/main/drivers/%.su src/main/drivers/%.cyclo: ../src/main/drivers/%.c src/main/drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DSTM32F40_41xxx -c -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/Third_Party/FatFs/src" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/App" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/Target" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/common" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/config" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/scheduler" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/accgyro" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/barometer" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/BMI270-Sensor-API" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/compass" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/gps" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/sdcard" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/fc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/flight" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/hw" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/io" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/msp" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/rx" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/sensors" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/target" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-main-2f-drivers

clean-src-2f-main-2f-drivers:
	-$(RM) ./src/main/drivers/buf_writer.cyclo ./src/main/drivers/buf_writer.d ./src/main/drivers/buf_writer.o ./src/main/drivers/buf_writer.su ./src/main/drivers/display.cyclo ./src/main/drivers/display.d ./src/main/drivers/display.o ./src/main/drivers/display.su ./src/main/drivers/display_canvas.cyclo ./src/main/drivers/display_canvas.d ./src/main/drivers/display_canvas.o ./src/main/drivers/display_canvas.su ./src/main/drivers/max7456.cyclo ./src/main/drivers/max7456.d ./src/main/drivers/max7456.o ./src/main/drivers/max7456.su ./src/main/drivers/motor.cyclo ./src/main/drivers/motor.d ./src/main/drivers/motor.o ./src/main/drivers/motor.su ./src/main/drivers/pwm_output.cyclo ./src/main/drivers/pwm_output.d ./src/main/drivers/pwm_output.o ./src/main/drivers/pwm_output.su ./src/main/drivers/stack_check.cyclo ./src/main/drivers/stack_check.d ./src/main/drivers/stack_check.o ./src/main/drivers/stack_check.su

.PHONY: clean-src-2f-main-2f-drivers

