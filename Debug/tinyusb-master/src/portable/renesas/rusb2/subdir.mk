################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb-master/src/portable/renesas/rusb2/dcd_rusb2.c \
../tinyusb-master/src/portable/renesas/rusb2/hcd_rusb2.c 

OBJS += \
./tinyusb-master/src/portable/renesas/rusb2/dcd_rusb2.o \
./tinyusb-master/src/portable/renesas/rusb2/hcd_rusb2.o 

C_DEPS += \
./tinyusb-master/src/portable/renesas/rusb2/dcd_rusb2.d \
./tinyusb-master/src/portable/renesas/rusb2/hcd_rusb2.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb-master/src/portable/renesas/rusb2/%.o tinyusb-master/src/portable/renesas/rusb2/%.su tinyusb-master/src/portable/renesas/rusb2/%.cyclo: ../tinyusb-master/src/portable/renesas/rusb2/%.c tinyusb-master/src/portable/renesas/rusb2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H750xx -DUSE_HAL_DRIVER -DARM_MATH_CM7 -DDEBUG -c -I../Core/Inc -I"C:/Users/alberto/Documents/750RTXLocal/750RTX/tinyusb-master/src" -I"C:/Users/alberto/Documents/750RTXLocal/750RTX/tinyusb-master/src/device" -I"C:/Users/alberto/STM32Cube/Repository/STM32Cube_FW_H7_V1.9.0/Drivers/CMSIS/DSP/Include" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/albytest/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/Drivers/CMSIS/DSP/Include" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2d-master-2f-src-2f-portable-2f-renesas-2f-rusb2

clean-tinyusb-2d-master-2f-src-2f-portable-2f-renesas-2f-rusb2:
	-$(RM) ./tinyusb-master/src/portable/renesas/rusb2/dcd_rusb2.cyclo ./tinyusb-master/src/portable/renesas/rusb2/dcd_rusb2.d ./tinyusb-master/src/portable/renesas/rusb2/dcd_rusb2.o ./tinyusb-master/src/portable/renesas/rusb2/dcd_rusb2.su ./tinyusb-master/src/portable/renesas/rusb2/hcd_rusb2.cyclo ./tinyusb-master/src/portable/renesas/rusb2/hcd_rusb2.d ./tinyusb-master/src/portable/renesas/rusb2/hcd_rusb2.o ./tinyusb-master/src/portable/renesas/rusb2/hcd_rusb2.su

.PHONY: clean-tinyusb-2d-master-2f-src-2f-portable-2f-renesas-2f-rusb2

