################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb-master/src/portable/bridgetek/ft9xx/dcd_ft9xx.c 

OBJS += \
./tinyusb-master/src/portable/bridgetek/ft9xx/dcd_ft9xx.o 

C_DEPS += \
./tinyusb-master/src/portable/bridgetek/ft9xx/dcd_ft9xx.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb-master/src/portable/bridgetek/ft9xx/%.o tinyusb-master/src/portable/bridgetek/ft9xx/%.su tinyusb-master/src/portable/bridgetek/ft9xx/%.cyclo: ../tinyusb-master/src/portable/bridgetek/ft9xx/%.c tinyusb-master/src/portable/bridgetek/ft9xx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H750xx -DUSE_HAL_DRIVER -DARM_MATH_CM7 -DDEBUG -c -I../Core/Inc -I"C:/Users/albytest/750RTXlocal/750RTX/tinyusb-master/src" -I"C:/Users/albytest/750RTXlocal/750RTX/tinyusb-master/src/device" -I"C:/Users/alberto/STM32Cube/Repository/STM32Cube_FW_H7_V1.9.0/Drivers/CMSIS/DSP/Include" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/albytest/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.1/Drivers/CMSIS/DSP/Include" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2d-master-2f-src-2f-portable-2f-bridgetek-2f-ft9xx

clean-tinyusb-2d-master-2f-src-2f-portable-2f-bridgetek-2f-ft9xx:
	-$(RM) ./tinyusb-master/src/portable/bridgetek/ft9xx/dcd_ft9xx.cyclo ./tinyusb-master/src/portable/bridgetek/ft9xx/dcd_ft9xx.d ./tinyusb-master/src/portable/bridgetek/ft9xx/dcd_ft9xx.o ./tinyusb-master/src/portable/bridgetek/ft9xx/dcd_ft9xx.su

.PHONY: clean-tinyusb-2d-master-2f-src-2f-portable-2f-bridgetek-2f-ft9xx

