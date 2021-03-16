################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32h750vbtx.s 

OBJS += \
./Core/Startup/startup_stm32h750vbtx.o 

S_DEPS += \
./Core/Startup/startup_stm32h750vbtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32h750vbtx.o: ../Core/Startup/startup_stm32h750vbtx.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32h750vbtx.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

