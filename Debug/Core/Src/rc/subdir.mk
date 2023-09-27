################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/rc/rc_interface.c 

OBJS += \
./Core/Src/rc/rc_interface.o 

C_DEPS += \
./Core/Src/rc/rc_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/rc/%.o Core/Src/rc/%.su Core/Src/rc/%.cyclo: ../Core/Src/rc/%.c Core/Src/rc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-rc

clean-Core-2f-Src-2f-rc:
	-$(RM) ./Core/Src/rc/rc_interface.cyclo ./Core/Src/rc/rc_interface.d ./Core/Src/rc/rc_interface.o ./Core/Src/rc/rc_interface.su

.PHONY: clean-Core-2f-Src-2f-rc

