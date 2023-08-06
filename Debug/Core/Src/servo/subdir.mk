################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/servo/servo_interface.c 

OBJS += \
./Core/Src/servo/servo_interface.o 

C_DEPS += \
./Core/Src/servo/servo_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/servo/%.o Core/Src/servo/%.su Core/Src/servo/%.cyclo: ../Core/Src/servo/%.c Core/Src/servo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-servo

clean-Core-2f-Src-2f-servo:
	-$(RM) ./Core/Src/servo/servo_interface.cyclo ./Core/Src/servo/servo_interface.d ./Core/Src/servo/servo_interface.o ./Core/Src/servo/servo_interface.su

.PHONY: clean-Core-2f-Src-2f-servo

