################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/autopilot/autopilot.c 

OBJS += \
./Core/Src/autopilot/autopilot.o 

C_DEPS += \
./Core/Src/autopilot/autopilot.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/autopilot/%.o Core/Src/autopilot/%.su Core/Src/autopilot/%.cyclo: ../Core/Src/autopilot/%.c Core/Src/autopilot/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-autopilot

clean-Core-2f-Src-2f-autopilot:
	-$(RM) ./Core/Src/autopilot/autopilot.cyclo ./Core/Src/autopilot/autopilot.d ./Core/Src/autopilot/autopilot.o ./Core/Src/autopilot/autopilot.su

.PHONY: clean-Core-2f-Src-2f-autopilot

