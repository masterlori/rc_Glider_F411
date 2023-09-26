################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/modem/modem_interface.c 

OBJS += \
./Core/Src/modem/modem_interface.o 

C_DEPS += \
./Core/Src/modem/modem_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/modem/%.o Core/Src/modem/%.su Core/Src/modem/%.cyclo: ../Core/Src/modem/%.c Core/Src/modem/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-modem

clean-Core-2f-Src-2f-modem:
	-$(RM) ./Core/Src/modem/modem_interface.cyclo ./Core/Src/modem/modem_interface.d ./Core/Src/modem/modem_interface.o ./Core/Src/modem/modem_interface.su

.PHONY: clean-Core-2f-Src-2f-modem

