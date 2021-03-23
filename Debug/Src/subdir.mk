################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FIR_5Order_RingBuffer.c \
../Src/Filter_3_Order.c \
../Src/H_Bridge_Sinwave.c \
../Src/IIR_1_Order.c \
../Src/Lichtschranke.c \
../Src/SchrittMotor_Y.c \
../Src/SchrittMotor_Z.c \
../Src/Temperatur_Regelung.c \
../Src/i2c-lcd.c \
../Src/main.c \
../Src/stm32f3xx_hal_msp.c \
../Src/stm32f3xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f3xx.c 

OBJS += \
./Src/FIR_5Order_RingBuffer.o \
./Src/Filter_3_Order.o \
./Src/H_Bridge_Sinwave.o \
./Src/IIR_1_Order.o \
./Src/Lichtschranke.o \
./Src/SchrittMotor_Y.o \
./Src/SchrittMotor_Z.o \
./Src/Temperatur_Regelung.o \
./Src/i2c-lcd.o \
./Src/main.o \
./Src/stm32f3xx_hal_msp.o \
./Src/stm32f3xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f3xx.o 

C_DEPS += \
./Src/FIR_5Order_RingBuffer.d \
./Src/Filter_3_Order.d \
./Src/H_Bridge_Sinwave.d \
./Src/IIR_1_Order.d \
./Src/Lichtschranke.d \
./Src/SchrittMotor_Y.d \
./Src/SchrittMotor_Z.d \
./Src/Temperatur_Regelung.d \
./Src/i2c-lcd.d \
./Src/main.d \
./Src/stm32f3xx_hal_msp.d \
./Src/stm32f3xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/Home/Desktop/Drucker_USB_stick/SLA_3d_printer/3d-printer_project/Inc" -I"C:/Users/Home/Desktop/Drucker_USB_stick/SLA_3d_printer/3d-printer_project/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/Home/Desktop/Drucker_USB_stick/SLA_3d_printer/3d-printer_project/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Home/Desktop/Drucker_USB_stick/SLA_3d_printer/3d-printer_project/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/Home/Desktop/Drucker_USB_stick/SLA_3d_printer/3d-printer_project/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


