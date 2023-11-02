################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main/common/bitarray.c \
../src/main/common/colorconversion.c \
../src/main/common/crc.c \
../src/main/common/encoding.c \
../src/main/common/explog_approx.c \
../src/main/common/filter.c \
../src/main/common/gps_conversion.c \
../src/main/common/huffman.c \
../src/main/common/huffman_table.c \
../src/main/common/maths.c \
../src/main/common/printf.c \
../src/main/common/printf_serial.c \
../src/main/common/ring_buffer.c \
../src/main/common/sdft.c \
../src/main/common/sensor_alignment.c \
../src/main/common/streambuf.c \
../src/main/common/string_light.c \
../src/main/common/strtol.c \
../src/main/common/time.c \
../src/main/common/typeconversion.c 

OBJS += \
./src/main/common/bitarray.o \
./src/main/common/colorconversion.o \
./src/main/common/crc.o \
./src/main/common/encoding.o \
./src/main/common/explog_approx.o \
./src/main/common/filter.o \
./src/main/common/gps_conversion.o \
./src/main/common/huffman.o \
./src/main/common/huffman_table.o \
./src/main/common/maths.o \
./src/main/common/printf.o \
./src/main/common/printf_serial.o \
./src/main/common/ring_buffer.o \
./src/main/common/sdft.o \
./src/main/common/sensor_alignment.o \
./src/main/common/streambuf.o \
./src/main/common/string_light.o \
./src/main/common/strtol.o \
./src/main/common/time.o \
./src/main/common/typeconversion.o 

C_DEPS += \
./src/main/common/bitarray.d \
./src/main/common/colorconversion.d \
./src/main/common/crc.d \
./src/main/common/encoding.d \
./src/main/common/explog_approx.d \
./src/main/common/filter.d \
./src/main/common/gps_conversion.d \
./src/main/common/huffman.d \
./src/main/common/huffman_table.d \
./src/main/common/maths.d \
./src/main/common/printf.d \
./src/main/common/printf_serial.d \
./src/main/common/ring_buffer.d \
./src/main/common/sdft.d \
./src/main/common/sensor_alignment.d \
./src/main/common/streambuf.d \
./src/main/common/string_light.d \
./src/main/common/strtol.d \
./src/main/common/time.d \
./src/main/common/typeconversion.d 


# Each subdirectory must supply rules for building sources it contributes
src/main/common/%.o src/main/common/%.su src/main/common/%.cyclo: ../src/main/common/%.c src/main/common/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DSTM32F40_41xxx -c -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/Third_Party/FatFs/src" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/App" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/Target" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/common" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/config" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/scheduler" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/accgyro" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/barometer" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/BMI270-Sensor-API" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/compass" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/gps" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/sdcard" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/fc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/flight" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/hw" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/io" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/msp" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/rx" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/sensors" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/target" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-main-2f-common

clean-src-2f-main-2f-common:
	-$(RM) ./src/main/common/bitarray.cyclo ./src/main/common/bitarray.d ./src/main/common/bitarray.o ./src/main/common/bitarray.su ./src/main/common/colorconversion.cyclo ./src/main/common/colorconversion.d ./src/main/common/colorconversion.o ./src/main/common/colorconversion.su ./src/main/common/crc.cyclo ./src/main/common/crc.d ./src/main/common/crc.o ./src/main/common/crc.su ./src/main/common/encoding.cyclo ./src/main/common/encoding.d ./src/main/common/encoding.o ./src/main/common/encoding.su ./src/main/common/explog_approx.cyclo ./src/main/common/explog_approx.d ./src/main/common/explog_approx.o ./src/main/common/explog_approx.su ./src/main/common/filter.cyclo ./src/main/common/filter.d ./src/main/common/filter.o ./src/main/common/filter.su ./src/main/common/gps_conversion.cyclo ./src/main/common/gps_conversion.d ./src/main/common/gps_conversion.o ./src/main/common/gps_conversion.su ./src/main/common/huffman.cyclo ./src/main/common/huffman.d ./src/main/common/huffman.o ./src/main/common/huffman.su ./src/main/common/huffman_table.cyclo ./src/main/common/huffman_table.d ./src/main/common/huffman_table.o ./src/main/common/huffman_table.su ./src/main/common/maths.cyclo ./src/main/common/maths.d ./src/main/common/maths.o ./src/main/common/maths.su ./src/main/common/printf.cyclo ./src/main/common/printf.d ./src/main/common/printf.o ./src/main/common/printf.su ./src/main/common/printf_serial.cyclo ./src/main/common/printf_serial.d ./src/main/common/printf_serial.o ./src/main/common/printf_serial.su ./src/main/common/ring_buffer.cyclo ./src/main/common/ring_buffer.d ./src/main/common/ring_buffer.o ./src/main/common/ring_buffer.su ./src/main/common/sdft.cyclo ./src/main/common/sdft.d ./src/main/common/sdft.o ./src/main/common/sdft.su ./src/main/common/sensor_alignment.cyclo ./src/main/common/sensor_alignment.d ./src/main/common/sensor_alignment.o ./src/main/common/sensor_alignment.su ./src/main/common/streambuf.cyclo ./src/main/common/streambuf.d ./src/main/common/streambuf.o ./src/main/common/streambuf.su ./src/main/common/string_light.cyclo ./src/main/common/string_light.d ./src/main/common/string_light.o ./src/main/common/string_light.su ./src/main/common/strtol.cyclo ./src/main/common/strtol.d ./src/main/common/strtol.o ./src/main/common/strtol.su ./src/main/common/time.cyclo ./src/main/common/time.d ./src/main/common/time.o ./src/main/common/time.su ./src/main/common/typeconversion.cyclo ./src/main/common/typeconversion.d ./src/main/common/typeconversion.o ./src/main/common/typeconversion.su

.PHONY: clean-src-2f-main-2f-common

