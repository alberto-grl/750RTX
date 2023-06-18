################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb-master/src/portable/nordic/nrf5x/dcd_nrf5x.c 

OBJS += \
./tinyusb-master/src/portable/nordic/nrf5x/dcd_nrf5x.o 

C_DEPS += \
./tinyusb-master/src/portable/nordic/nrf5x/dcd_nrf5x.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb-master/src/portable/nordic/nrf5x/%.o tinyusb-master/src/portable/nordic/nrf5x/%.su tinyusb-master/src/portable/nordic/nrf5x/%.cyclo: ../tinyusb-master/src/portable/nordic/nrf5x/%.c tinyusb-master/src/portable/nordic/nrf5x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H750xx -DUSE_HAL_DRIVER -DARM_MATH_CM7 -DDEBUG -c -I../Core/Inc -I"G:/Il mio Drive/750RTX/tinyusb-master/src" -I"G:/Il mio Drive/750RTX/tinyusb-master/src/device" -I"C:/Users/alberto/STM32Cube/Repository/STM32Cube_FW_H7_V1.9.0/Drivers/CMSIS/DSP/Include" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2d-master-2f-src-2f-portable-2f-nordic-2f-nrf5x

clean-tinyusb-2d-master-2f-src-2f-portable-2f-nordic-2f-nrf5x:
	-$(RM) ./tinyusb-master/src/portable/nordic/nrf5x/dcd_nrf5x.cyclo ./tinyusb-master/src/portable/nordic/nrf5x/dcd_nrf5x.d ./tinyusb-master/src/portable/nordic/nrf5x/dcd_nrf5x.o ./tinyusb-master/src/portable/nordic/nrf5x/dcd_nrf5x.su

.PHONY: clean-tinyusb-2d-master-2f-src-2f-portable-2f-nordic-2f-nrf5x

