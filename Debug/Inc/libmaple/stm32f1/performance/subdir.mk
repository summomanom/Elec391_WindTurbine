################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../Inc/libmaple/stm32f1/performance/isrs.S \
../Inc/libmaple/stm32f1/performance/vector_table.S 

OBJS += \
./Inc/libmaple/stm32f1/performance/isrs.o \
./Inc/libmaple/stm32f1/performance/vector_table.o 

S_UPPER_DEPS += \
./Inc/libmaple/stm32f1/performance/isrs.d \
./Inc/libmaple/stm32f1/performance/vector_table.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/libmaple/stm32f1/performance/%.o: ../Inc/libmaple/stm32f1/performance/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"C:/Users/oconn/Desktop/391/code/Inc" -I"C:/Users/oconn/Desktop/391/code/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/oconn/Desktop/391/code/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/oconn/Desktop/391/code/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/oconn/Desktop/391/code/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


