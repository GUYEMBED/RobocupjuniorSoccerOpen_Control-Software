################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Robot_Movement/InverseKinematicCalculation.cpp \
../Core/Src/Robot_Movement/KinematicData.cpp \
../Core/Src/Robot_Movement/motorControl.cpp \
../Core/Src/Robot_Movement/movementSystem.cpp 

OBJS += \
./Core/Src/Robot_Movement/InverseKinematicCalculation.o \
./Core/Src/Robot_Movement/KinematicData.o \
./Core/Src/Robot_Movement/motorControl.o \
./Core/Src/Robot_Movement/movementSystem.o 

CPP_DEPS += \
./Core/Src/Robot_Movement/InverseKinematicCalculation.d \
./Core/Src/Robot_Movement/KinematicData.d \
./Core/Src/Robot_Movement/motorControl.d \
./Core/Src/Robot_Movement/movementSystem.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Robot_Movement/%.o Core/Src/Robot_Movement/%.su Core/Src/Robot_Movement/%.cyclo: ../Core/Src/Robot_Movement/%.cpp Core/Src/Robot_Movement/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Sensor" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Robot_Navigation" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Robot_BallRelated" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Robot_Movement" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Sensor" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Robot_Navigation" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Robot_BallRelated" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/PID_Controller" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/PID_Controller" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Robot_Movement" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Robot_Movement

clean-Core-2f-Src-2f-Robot_Movement:
	-$(RM) ./Core/Src/Robot_Movement/InverseKinematicCalculation.cyclo ./Core/Src/Robot_Movement/InverseKinematicCalculation.d ./Core/Src/Robot_Movement/InverseKinematicCalculation.o ./Core/Src/Robot_Movement/InverseKinematicCalculation.su ./Core/Src/Robot_Movement/KinematicData.cyclo ./Core/Src/Robot_Movement/KinematicData.d ./Core/Src/Robot_Movement/KinematicData.o ./Core/Src/Robot_Movement/KinematicData.su ./Core/Src/Robot_Movement/motorControl.cyclo ./Core/Src/Robot_Movement/motorControl.d ./Core/Src/Robot_Movement/motorControl.o ./Core/Src/Robot_Movement/motorControl.su ./Core/Src/Robot_Movement/movementSystem.cyclo ./Core/Src/Robot_Movement/movementSystem.d ./Core/Src/Robot_Movement/movementSystem.o ./Core/Src/Robot_Movement/movementSystem.su

.PHONY: clean-Core-2f-Src-2f-Robot_Movement

