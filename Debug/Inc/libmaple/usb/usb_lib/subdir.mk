################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Inc/libmaple/usb/usb_lib/usb_core.c \
../Inc/libmaple/usb/usb_lib/usb_init.c \
../Inc/libmaple/usb/usb_lib/usb_mem.c \
../Inc/libmaple/usb/usb_lib/usb_regs.c 

OBJS += \
./Inc/libmaple/usb/usb_lib/usb_core.o \
./Inc/libmaple/usb/usb_lib/usb_init.o \
./Inc/libmaple/usb/usb_lib/usb_mem.o \
./Inc/libmaple/usb/usb_lib/usb_regs.o 

C_DEPS += \
./Inc/libmaple/usb/usb_lib/usb_core.d \
./Inc/libmaple/usb/usb_lib/usb_init.d \
./Inc/libmaple/usb/usb_lib/usb_mem.d \
./Inc/libmaple/usb/usb_lib/usb_regs.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/libmaple/usb/usb_lib/%.o: ../Inc/libmaple/usb/usb_lib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"C:/Users/oconn/Desktop/391/code/Inc" -I"C:/Users/oconn/Desktop/391/code/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/oconn/Desktop/391/code/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/oconn/Desktop/391/code/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/oconn/Desktop/391/code/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


