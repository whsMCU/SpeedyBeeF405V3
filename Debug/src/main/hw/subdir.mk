################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main/hw/adc.c \
../src/main/hw/cdc.c \
../src/main/hw/cli.c \
../src/main/hw/dma.c \
../src/main/hw/fatfs.c \
../src/main/hw/files.c \
../src/main/hw/flash.c \
../src/main/hw/gpio.c \
../src/main/hw/i2c.c \
../src/main/hw/led.c \
../src/main/hw/rtc.c \
../src/main/hw/sd.c \
../src/main/hw/spi.c \
../src/main/hw/timer.c \
../src/main/hw/uart.c \
../src/main/hw/usb.c 

OBJS += \
./src/main/hw/adc.o \
./src/main/hw/cdc.o \
./src/main/hw/cli.o \
./src/main/hw/dma.o \
./src/main/hw/fatfs.o \
./src/main/hw/files.o \
./src/main/hw/flash.o \
./src/main/hw/gpio.o \
./src/main/hw/i2c.o \
./src/main/hw/led.o \
./src/main/hw/rtc.o \
./src/main/hw/sd.o \
./src/main/hw/spi.o \
./src/main/hw/timer.o \
./src/main/hw/uart.o \
./src/main/hw/usb.o 

C_DEPS += \
./src/main/hw/adc.d \
./src/main/hw/cdc.d \
./src/main/hw/cli.d \
./src/main/hw/dma.d \
./src/main/hw/fatfs.d \
./src/main/hw/files.d \
./src/main/hw/flash.d \
./src/main/hw/gpio.d \
./src/main/hw/i2c.d \
./src/main/hw/led.d \
./src/main/hw/rtc.d \
./src/main/hw/sd.d \
./src/main/hw/spi.d \
./src/main/hw/timer.d \
./src/main/hw/uart.d \
./src/main/hw/usb.d 


# Each subdirectory must supply rules for building sources it contributes
src/main/hw/%.o src/main/hw/%.su src/main/hw/%.cyclo: ../src/main/hw/%.c src/main/hw/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DSTM32F40_41xxx -c -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/Third_Party/FatFs/src" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/App" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/Target" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/common" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/config" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/scheduler" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/accgyro" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/barometer" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/BMI270-Sensor-API" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/compass" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/gps" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/sdcard" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/fc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/flight" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/hw" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/io" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/msp" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/rx" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/sensors" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/target" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-main-2f-hw

clean-src-2f-main-2f-hw:
	-$(RM) ./src/main/hw/adc.cyclo ./src/main/hw/adc.d ./src/main/hw/adc.o ./src/main/hw/adc.su ./src/main/hw/cdc.cyclo ./src/main/hw/cdc.d ./src/main/hw/cdc.o ./src/main/hw/cdc.su ./src/main/hw/cli.cyclo ./src/main/hw/cli.d ./src/main/hw/cli.o ./src/main/hw/cli.su ./src/main/hw/dma.cyclo ./src/main/hw/dma.d ./src/main/hw/dma.o ./src/main/hw/dma.su ./src/main/hw/fatfs.cyclo ./src/main/hw/fatfs.d ./src/main/hw/fatfs.o ./src/main/hw/fatfs.su ./src/main/hw/files.cyclo ./src/main/hw/files.d ./src/main/hw/files.o ./src/main/hw/files.su ./src/main/hw/flash.cyclo ./src/main/hw/flash.d ./src/main/hw/flash.o ./src/main/hw/flash.su ./src/main/hw/gpio.cyclo ./src/main/hw/gpio.d ./src/main/hw/gpio.o ./src/main/hw/gpio.su ./src/main/hw/i2c.cyclo ./src/main/hw/i2c.d ./src/main/hw/i2c.o ./src/main/hw/i2c.su ./src/main/hw/led.cyclo ./src/main/hw/led.d ./src/main/hw/led.o ./src/main/hw/led.su ./src/main/hw/rtc.cyclo ./src/main/hw/rtc.d ./src/main/hw/rtc.o ./src/main/hw/rtc.su ./src/main/hw/sd.cyclo ./src/main/hw/sd.d ./src/main/hw/sd.o ./src/main/hw/sd.su ./src/main/hw/spi.cyclo ./src/main/hw/spi.d ./src/main/hw/spi.o ./src/main/hw/spi.su ./src/main/hw/timer.cyclo ./src/main/hw/timer.d ./src/main/hw/timer.o ./src/main/hw/timer.su ./src/main/hw/uart.cyclo ./src/main/hw/uart.d ./src/main/hw/uart.o ./src/main/hw/uart.su ./src/main/hw/usb.cyclo ./src/main/hw/usb.d ./src/main/hw/usb.o ./src/main/hw/usb.su

.PHONY: clean-src-2f-main-2f-hw

