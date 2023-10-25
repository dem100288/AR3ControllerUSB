################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/ProjectOffice/AR3/ARController/AR3ControllerUSB/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
D:/ProjectOffice/AR3/ARController/AR3ControllerUSB/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
D:/ProjectOffice/AR3/ARController/AR3ControllerUSB/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
D:/ProjectOffice/AR3/ARController/AR3ControllerUSB/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/usbd_cdc.o \
./Middlewares/ST/STM32_USB_Device_Library/usbd_core.o \
./Middlewares/ST/STM32_USB_Device_Library/usbd_ctlreq.o \
./Middlewares/ST/STM32_USB_Device_Library/usbd_ioreq.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/usbd_cdc.d \
./Middlewares/ST/STM32_USB_Device_Library/usbd_core.d \
./Middlewares/ST/STM32_USB_Device_Library/usbd_ctlreq.d \
./Middlewares/ST/STM32_USB_Device_Library/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/usbd_cdc.o: D:/ProjectOffice/AR3/ARController/AR3ControllerUSB/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c Middlewares/ST/STM32_USB_Device_Library/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/STM32_USB_Device_Library/usbd_core.o: D:/ProjectOffice/AR3/ARController/AR3ControllerUSB/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c Middlewares/ST/STM32_USB_Device_Library/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/STM32_USB_Device_Library/usbd_ctlreq.o: D:/ProjectOffice/AR3/ARController/AR3ControllerUSB/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c Middlewares/ST/STM32_USB_Device_Library/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/STM32_USB_Device_Library/usbd_ioreq.o: D:/ProjectOffice/AR3/ARController/AR3ControllerUSB/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c Middlewares/ST/STM32_USB_Device_Library/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library

clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library:
	-$(RM) ./Middlewares/ST/STM32_USB_Device_Library/usbd_cdc.cyclo ./Middlewares/ST/STM32_USB_Device_Library/usbd_cdc.d ./Middlewares/ST/STM32_USB_Device_Library/usbd_cdc.o ./Middlewares/ST/STM32_USB_Device_Library/usbd_cdc.su ./Middlewares/ST/STM32_USB_Device_Library/usbd_core.cyclo ./Middlewares/ST/STM32_USB_Device_Library/usbd_core.d ./Middlewares/ST/STM32_USB_Device_Library/usbd_core.o ./Middlewares/ST/STM32_USB_Device_Library/usbd_core.su ./Middlewares/ST/STM32_USB_Device_Library/usbd_ctlreq.cyclo ./Middlewares/ST/STM32_USB_Device_Library/usbd_ctlreq.d ./Middlewares/ST/STM32_USB_Device_Library/usbd_ctlreq.o ./Middlewares/ST/STM32_USB_Device_Library/usbd_ctlreq.su ./Middlewares/ST/STM32_USB_Device_Library/usbd_ioreq.cyclo ./Middlewares/ST/STM32_USB_Device_Library/usbd_ioreq.d ./Middlewares/ST/STM32_USB_Device_Library/usbd_ioreq.o ./Middlewares/ST/STM32_USB_Device_Library/usbd_ioreq.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library

