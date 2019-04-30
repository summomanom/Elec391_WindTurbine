################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Inc/libmaple/adc.c \
../Inc/libmaple/adc_f1.c \
../Inc/libmaple/bkp_f1.c \
../Inc/libmaple/dac.c \
../Inc/libmaple/dma.c \
../Inc/libmaple/dma_f1.c \
../Inc/libmaple/exti.c \
../Inc/libmaple/exti_f1.c \
../Inc/libmaple/flash.c \
../Inc/libmaple/fsmc_f1.c \
../Inc/libmaple/gpio.c \
../Inc/libmaple/gpio_f1.c \
../Inc/libmaple/i2c.c \
../Inc/libmaple/i2c_f1.c \
../Inc/libmaple/iwdg.c \
../Inc/libmaple/nvic.c \
../Inc/libmaple/pwr.c \
../Inc/libmaple/rcc.c \
../Inc/libmaple/rcc_f1.c \
../Inc/libmaple/spi.c \
../Inc/libmaple/spi_f1.c \
../Inc/libmaple/systick.c \
../Inc/libmaple/timer.c \
../Inc/libmaple/timer_f1.c \
../Inc/libmaple/usart.c \
../Inc/libmaple/usart_f1.c \
../Inc/libmaple/usart_private.c \
../Inc/libmaple/util.c 

S_UPPER_SRCS += \
../Inc/libmaple/exc.S 

OBJS += \
./Inc/libmaple/adc.o \
./Inc/libmaple/adc_f1.o \
./Inc/libmaple/bkp_f1.o \
./Inc/libmaple/dac.o \
./Inc/libmaple/dma.o \
./Inc/libmaple/dma_f1.o \
./Inc/libmaple/exc.o \
./Inc/libmaple/exti.o \
./Inc/libmaple/exti_f1.o \
./Inc/libmaple/flash.o \
./Inc/libmaple/fsmc_f1.o \
./Inc/libmaple/gpio.o \
./Inc/libmaple/gpio_f1.o \
./Inc/libmaple/i2c.o \
./Inc/libmaple/i2c_f1.o \
./Inc/libmaple/iwdg.o \
./Inc/libmaple/nvic.o \
./Inc/libmaple/pwr.o \
./Inc/libmaple/rcc.o \
./Inc/libmaple/rcc_f1.o \
./Inc/libmaple/spi.o \
./Inc/libmaple/spi_f1.o \
./Inc/libmaple/systick.o \
./Inc/libmaple/timer.o \
./Inc/libmaple/timer_f1.o \
./Inc/libmaple/usart.o \
./Inc/libmaple/usart_f1.o \
./Inc/libmaple/usart_private.o \
./Inc/libmaple/util.o 

S_UPPER_DEPS += \
./Inc/libmaple/exc.d 

C_DEPS += \
./Inc/libmaple/adc.d \
./Inc/libmaple/adc_f1.d \
./Inc/libmaple/bkp_f1.d \
./Inc/libmaple/dac.d \
./Inc/libmaple/dma.d \
./Inc/libmaple/dma_f1.d \
./Inc/libmaple/exti.d \
./Inc/libmaple/exti_f1.d \
./Inc/libmaple/flash.d \
./Inc/libmaple/fsmc_f1.d \
./Inc/libmaple/gpio.d \
./Inc/libmaple/gpio_f1.d \
./Inc/libmaple/i2c.d \
./Inc/libmaple/i2c_f1.d \
./Inc/libmaple/iwdg.d \
./Inc/libmaple/nvic.d \
./Inc/libmaple/pwr.d \
./Inc/libmaple/rcc.d \
./Inc/libmaple/rcc_f1.d \
./Inc/libmaple/spi.d \
./Inc/libmaple/spi_f1.d \
./Inc/libmaple/systick.d \
./Inc/libmaple/timer.d \
./Inc/libmaple/timer_f1.d \
./Inc/libmaple/usart.d \
./Inc/libmaple/usart_f1.d \
./Inc/libmaple/usart_private.d \
./Inc/libmaple/util.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/libmaple/%.o: ../Inc/libmaple/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"C:/Users/oconn/Desktop/391/code/Inc" -I"C:/Users/oconn/Desktop/391/code/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/oconn/Desktop/391/code/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/oconn/Desktop/391/code/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/oconn/Desktop/391/code/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Inc/libmaple/%.o: ../Inc/libmaple/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"C:/Users/oconn/Desktop/391/code/Inc" -I"C:/Users/oconn/Desktop/391/code/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/oconn/Desktop/391/code/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/oconn/Desktop/391/code/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/oconn/Desktop/391/code/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


