################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/st7789h2/st7789h2.c 

OBJS += \
./Drivers/Components/st7789h2/st7789h2.o 

C_DEPS += \
./Drivers/Components/st7789h2/st7789h2.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/st7789h2/%.o Drivers/Components/st7789h2/%.su Drivers/Components/st7789h2/%.cyclo: ../Drivers/Components/st7789h2/%.c Drivers/Components/st7789h2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I/Users/oliviertrudeau/STM32CubeIDE/workspace_1.17.0/finalProject/Drivers/Components -I/Users/oliviertrudeau/STM32CubeIDE/workspace_1.17.0/finalProject/Drivers/Components/lsm6dsl -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-st7789h2

clean-Drivers-2f-Components-2f-st7789h2:
	-$(RM) ./Drivers/Components/st7789h2/st7789h2.cyclo ./Drivers/Components/st7789h2/st7789h2.d ./Drivers/Components/st7789h2/st7789h2.o ./Drivers/Components/st7789h2/st7789h2.su

.PHONY: clean-Drivers-2f-Components-2f-st7789h2

