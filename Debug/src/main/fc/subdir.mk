################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main/fc/board_info.c \
../src/main/fc/controlrate_profile.c \
../src/main/fc/core.c \
../src/main/fc/dispatch.c \
../src/main/fc/init.c \
../src/main/fc/rc.c \
../src/main/fc/rc_adjustments.c \
../src/main/fc/rc_controls.c \
../src/main/fc/rc_modes.c \
../src/main/fc/runtime_config.c \
../src/main/fc/stats.c 

OBJS += \
./src/main/fc/board_info.o \
./src/main/fc/controlrate_profile.o \
./src/main/fc/core.o \
./src/main/fc/dispatch.o \
./src/main/fc/init.o \
./src/main/fc/rc.o \
./src/main/fc/rc_adjustments.o \
./src/main/fc/rc_controls.o \
./src/main/fc/rc_modes.o \
./src/main/fc/runtime_config.o \
./src/main/fc/stats.o 

C_DEPS += \
./src/main/fc/board_info.d \
./src/main/fc/controlrate_profile.d \
./src/main/fc/core.d \
./src/main/fc/dispatch.d \
./src/main/fc/init.d \
./src/main/fc/rc.d \
./src/main/fc/rc_adjustments.d \
./src/main/fc/rc_controls.d \
./src/main/fc/rc_modes.d \
./src/main/fc/runtime_config.d \
./src/main/fc/stats.d 


# Each subdirectory must supply rules for building sources it contributes
src/main/fc/%.o src/main/fc/%.su src/main/fc/%.cyclo: ../src/main/fc/%.c src/main/fc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DSTM32F40_41xxx -c -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/Third_Party/FatFs/src" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/App" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/Target" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/common" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/config" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/scheduler" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/accgyro" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/barometer" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/BMI270-Sensor-API" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/compass" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/gps" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/sdcard" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/fc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/flight" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/hw" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/io" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/msp" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/rx" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/sensors" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/target" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-main-2f-fc

clean-src-2f-main-2f-fc:
	-$(RM) ./src/main/fc/board_info.cyclo ./src/main/fc/board_info.d ./src/main/fc/board_info.o ./src/main/fc/board_info.su ./src/main/fc/controlrate_profile.cyclo ./src/main/fc/controlrate_profile.d ./src/main/fc/controlrate_profile.o ./src/main/fc/controlrate_profile.su ./src/main/fc/core.cyclo ./src/main/fc/core.d ./src/main/fc/core.o ./src/main/fc/core.su ./src/main/fc/dispatch.cyclo ./src/main/fc/dispatch.d ./src/main/fc/dispatch.o ./src/main/fc/dispatch.su ./src/main/fc/init.cyclo ./src/main/fc/init.d ./src/main/fc/init.o ./src/main/fc/init.su ./src/main/fc/rc.cyclo ./src/main/fc/rc.d ./src/main/fc/rc.o ./src/main/fc/rc.su ./src/main/fc/rc_adjustments.cyclo ./src/main/fc/rc_adjustments.d ./src/main/fc/rc_adjustments.o ./src/main/fc/rc_adjustments.su ./src/main/fc/rc_controls.cyclo ./src/main/fc/rc_controls.d ./src/main/fc/rc_controls.o ./src/main/fc/rc_controls.su ./src/main/fc/rc_modes.cyclo ./src/main/fc/rc_modes.d ./src/main/fc/rc_modes.o ./src/main/fc/rc_modes.su ./src/main/fc/runtime_config.cyclo ./src/main/fc/runtime_config.d ./src/main/fc/runtime_config.o ./src/main/fc/runtime_config.su ./src/main/fc/stats.cyclo ./src/main/fc/stats.d ./src/main/fc/stats.o ./src/main/fc/stats.su

.PHONY: clean-src-2f-main-2f-fc

