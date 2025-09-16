################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ICM20948.c \
../Core/Src/control.c \
../Core/Src/freertos.c \
../Core/Src/imu.c \
../Core/Src/madgwick.c \
../Core/Src/main.c \
../Core/Src/motor.c \
../Core/Src/movements.c \
../Core/Src/oled.c \
../Core/Src/pid.c \
../Core/Src/sensor.c \
../Core/Src/servo.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/userButton.c 

OBJS += \
./Core/Src/ICM20948.o \
./Core/Src/control.o \
./Core/Src/freertos.o \
./Core/Src/imu.o \
./Core/Src/madgwick.o \
./Core/Src/main.o \
./Core/Src/motor.o \
./Core/Src/movements.o \
./Core/Src/oled.o \
./Core/Src/pid.o \
./Core/Src/sensor.o \
./Core/Src/servo.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/userButton.o 

C_DEPS += \
./Core/Src/ICM20948.d \
./Core/Src/control.d \
./Core/Src/freertos.d \
./Core/Src/imu.d \
./Core/Src/madgwick.d \
./Core/Src/main.d \
./Core/Src/motor.d \
./Core/Src/movements.d \
./Core/Src/oled.d \
./Core/Src/pid.d \
./Core/Src/sensor.d \
./Core/Src/servo.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/userButton.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ICM20948.cyclo ./Core/Src/ICM20948.d ./Core/Src/ICM20948.o ./Core/Src/ICM20948.su ./Core/Src/control.cyclo ./Core/Src/control.d ./Core/Src/control.o ./Core/Src/control.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/imu.cyclo ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/madgwick.cyclo ./Core/Src/madgwick.d ./Core/Src/madgwick.o ./Core/Src/madgwick.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/movements.cyclo ./Core/Src/movements.d ./Core/Src/movements.o ./Core/Src/movements.su ./Core/Src/oled.cyclo ./Core/Src/oled.d ./Core/Src/oled.o ./Core/Src/oled.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/sensor.cyclo ./Core/Src/sensor.d ./Core/Src/sensor.o ./Core/Src/sensor.su ./Core/Src/servo.cyclo ./Core/Src/servo.d ./Core/Src/servo.o ./Core/Src/servo.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/userButton.cyclo ./Core/Src/userButton.d ./Core/Src/userButton.o ./Core/Src/userButton.su

.PHONY: clean-Core-2f-Src

