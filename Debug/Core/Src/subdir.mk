################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/communication.c \
../Core/Src/control.c \
../Core/Src/current_measurement.c \
../Core/Src/encoder.c \
../Core/Src/foc.c \
../Core/Src/foc_math.c \
../Core/Src/main.c \
../Core/Src/observer.c \
../Core/Src/overcurrent_overvoltage_protection.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/svm.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/task.c 

OBJS += \
./Core/Src/communication.o \
./Core/Src/control.o \
./Core/Src/current_measurement.o \
./Core/Src/encoder.o \
./Core/Src/foc.o \
./Core/Src/foc_math.o \
./Core/Src/main.o \
./Core/Src/observer.o \
./Core/Src/overcurrent_overvoltage_protection.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/svm.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/task.o 

C_DEPS += \
./Core/Src/communication.d \
./Core/Src/control.d \
./Core/Src/current_measurement.d \
./Core/Src/encoder.d \
./Core/Src/foc.d \
./Core/Src/foc_math.d \
./Core/Src/main.d \
./Core/Src/observer.d \
./Core/Src/overcurrent_overvoltage_protection.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/svm.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/task.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/observer.o: ../Core/Src/observer.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/communication.cyclo ./Core/Src/communication.d ./Core/Src/communication.o ./Core/Src/communication.su ./Core/Src/control.cyclo ./Core/Src/control.d ./Core/Src/control.o ./Core/Src/control.su ./Core/Src/current_measurement.cyclo ./Core/Src/current_measurement.d ./Core/Src/current_measurement.o ./Core/Src/current_measurement.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/foc.cyclo ./Core/Src/foc.d ./Core/Src/foc.o ./Core/Src/foc.su ./Core/Src/foc_math.cyclo ./Core/Src/foc_math.d ./Core/Src/foc_math.o ./Core/Src/foc_math.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/observer.cyclo ./Core/Src/observer.d ./Core/Src/observer.o ./Core/Src/observer.su ./Core/Src/overcurrent_overvoltage_protection.cyclo ./Core/Src/overcurrent_overvoltage_protection.d ./Core/Src/overcurrent_overvoltage_protection.o ./Core/Src/overcurrent_overvoltage_protection.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/svm.cyclo ./Core/Src/svm.d ./Core/Src/svm.o ./Core/Src/svm.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/task.cyclo ./Core/Src/task.d ./Core/Src/task.o ./Core/Src/task.su

.PHONY: clean-Core-2f-Src

