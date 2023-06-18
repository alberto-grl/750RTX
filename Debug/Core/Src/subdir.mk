################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CWDecoder.c \
../Core/Src/CWKeyer.c \
../Core/Src/DCF77.c \
../Core/Src/MyFFT.c \
../Core/Src/SDR_func.c \
../Core/Src/SDR_math.c \
../Core/Src/WSPR.c \
../Core/Src/acqTrigger.c \
../Core/Src/dataAcq.c \
../Core/Src/main.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/usb_descriptors.c 

OBJS += \
./Core/Src/CWDecoder.o \
./Core/Src/CWKeyer.o \
./Core/Src/DCF77.o \
./Core/Src/MyFFT.o \
./Core/Src/SDR_func.o \
./Core/Src/SDR_math.o \
./Core/Src/WSPR.o \
./Core/Src/acqTrigger.o \
./Core/Src/dataAcq.o \
./Core/Src/main.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/usb_descriptors.o 

C_DEPS += \
./Core/Src/CWDecoder.d \
./Core/Src/CWKeyer.d \
./Core/Src/DCF77.d \
./Core/Src/MyFFT.d \
./Core/Src/SDR_func.d \
./Core/Src/SDR_math.d \
./Core/Src/WSPR.d \
./Core/Src/acqTrigger.d \
./Core/Src/dataAcq.d \
./Core/Src/main.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/usb_descriptors.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H750xx -DUSE_HAL_DRIVER -DARM_MATH_CM7 -DDEBUG -c -I../Core/Inc -I"G:/Il mio Drive/750RTX/tinyusb-master/src" -I"G:/Il mio Drive/750RTX/tinyusb-master/src/device" -I"C:/Users/alberto/STM32Cube/Repository/STM32Cube_FW_H7_V1.9.0/Drivers/CMSIS/DSP/Include" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/CWDecoder.cyclo ./Core/Src/CWDecoder.d ./Core/Src/CWDecoder.o ./Core/Src/CWDecoder.su ./Core/Src/CWKeyer.cyclo ./Core/Src/CWKeyer.d ./Core/Src/CWKeyer.o ./Core/Src/CWKeyer.su ./Core/Src/DCF77.cyclo ./Core/Src/DCF77.d ./Core/Src/DCF77.o ./Core/Src/DCF77.su ./Core/Src/MyFFT.cyclo ./Core/Src/MyFFT.d ./Core/Src/MyFFT.o ./Core/Src/MyFFT.su ./Core/Src/SDR_func.cyclo ./Core/Src/SDR_func.d ./Core/Src/SDR_func.o ./Core/Src/SDR_func.su ./Core/Src/SDR_math.cyclo ./Core/Src/SDR_math.d ./Core/Src/SDR_math.o ./Core/Src/SDR_math.su ./Core/Src/WSPR.cyclo ./Core/Src/WSPR.d ./Core/Src/WSPR.o ./Core/Src/WSPR.su ./Core/Src/acqTrigger.cyclo ./Core/Src/acqTrigger.d ./Core/Src/acqTrigger.o ./Core/Src/acqTrigger.su ./Core/Src/dataAcq.cyclo ./Core/Src/dataAcq.d ./Core/Src/dataAcq.o ./Core/Src/dataAcq.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/usb_descriptors.cyclo ./Core/Src/usb_descriptors.d ./Core/Src/usb_descriptors.o ./Core/Src/usb_descriptors.su

.PHONY: clean-Core-2f-Src

