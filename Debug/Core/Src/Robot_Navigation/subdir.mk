################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Robot_Navigation/Localization.cpp \
../Core/Src/Robot_Navigation/Navigation.cpp \
../Core/Src/Robot_Navigation/fieldData.cpp 

OBJS += \
./Core/Src/Robot_Navigation/Localization.o \
./Core/Src/Robot_Navigation/Navigation.o \
./Core/Src/Robot_Navigation/fieldData.o 

CPP_DEPS += \
./Core/Src/Robot_Navigation/Localization.d \
./Core/Src/Robot_Navigation/Navigation.d \
./Core/Src/Robot_Navigation/fieldData.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Robot_Navigation/%.o Core/Src/Robot_Navigation/%.su Core/Src/Robot_Navigation/%.cyclo: ../Core/Src/Robot_Navigation/%.cpp Core/Src/Robot_Navigation/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Sensor" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Robot_Navigation" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Robot_BallRelated" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Robot_Movement" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Sensor" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Robot_Navigation" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Robot_BallRelated" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/PID_Controller" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/PID_Controller" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Robot_Movement" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Robot_Navigation

clean-Core-2f-Src-2f-Robot_Navigation:
	-$(RM) ./Core/Src/Robot_Navigation/Localization.cyclo ./Core/Src/Robot_Navigation/Localization.d ./Core/Src/Robot_Navigation/Localization.o ./Core/Src/Robot_Navigation/Localization.su ./Core/Src/Robot_Navigation/Navigation.cyclo ./Core/Src/Robot_Navigation/Navigation.d ./Core/Src/Robot_Navigation/Navigation.o ./Core/Src/Robot_Navigation/Navigation.su ./Core/Src/Robot_Navigation/fieldData.cyclo ./Core/Src/Robot_Navigation/fieldData.d ./Core/Src/Robot_Navigation/fieldData.o ./Core/Src/Robot_Navigation/fieldData.su

.PHONY: clean-Core-2f-Src-2f-Robot_Navigation

