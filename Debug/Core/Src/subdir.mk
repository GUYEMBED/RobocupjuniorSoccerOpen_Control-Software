################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/PeripheralCallbackHandler.cpp \
../Core/Src/SystemGPIO.cpp \
../Core/Src/Timer.cpp \
../Core/Src/bno05.cpp \
../Core/Src/main.cpp 

C_SRCS += \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

C_DEPS += \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 

OBJS += \
./Core/Src/PeripheralCallbackHandler.o \
./Core/Src/SystemGPIO.o \
./Core/Src/Timer.o \
./Core/Src/bno05.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

CPP_DEPS += \
./Core/Src/PeripheralCallbackHandler.d \
./Core/Src/SystemGPIO.d \
./Core/Src/Timer.d \
./Core/Src/bno05.d \
./Core/Src/main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Sensor" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Robot_Navigation" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Robot_BallRelated" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Robot_Movement" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Sensor" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Robot_Navigation" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Robot_BallRelated" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/PID_Controller" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/PID_Controller" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Robot_Movement" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/PeripheralCallbackHandler.cyclo ./Core/Src/PeripheralCallbackHandler.d ./Core/Src/PeripheralCallbackHandler.o ./Core/Src/PeripheralCallbackHandler.su ./Core/Src/SystemGPIO.cyclo ./Core/Src/SystemGPIO.d ./Core/Src/SystemGPIO.o ./Core/Src/SystemGPIO.su ./Core/Src/Timer.cyclo ./Core/Src/Timer.d ./Core/Src/Timer.o ./Core/Src/Timer.su ./Core/Src/bno05.cyclo ./Core/Src/bno05.d ./Core/Src/bno05.o ./Core/Src/bno05.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

