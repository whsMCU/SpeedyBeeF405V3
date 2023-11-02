################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
../lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
../lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c 

OBJS += \
./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.o \
./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.o \
./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.o 

C_DEPS += \
./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.d \
./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.d \
./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.o lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.su lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.cyclo: ../lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.c lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DSTM32F40_41xxx -c -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/Middlewares/Third_Party/FatFs/src" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/App" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/lib/USB_DEVICE/Target" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/common" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/config" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/scheduler" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/accgyro" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/barometer" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/BMI270-Sensor-API" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/compass" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/gps" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/drivers/sdcard" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/fc" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/flight" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/hw" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/io" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/msp" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/rx" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/sensors" -I"C:/Users/jjins/Documents/Project/SpeedyBeeF405V3/src/main/target" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Core-2f-Src

clean-lib-2f-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Core-2f-Src:
	-$(RM) ./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.cyclo ./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.d ./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.o ./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.su ./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.cyclo ./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.d ./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.o ./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.su ./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.cyclo ./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.d ./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.o ./lib/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.su

.PHONY: clean-lib-2f-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Core-2f-Src

