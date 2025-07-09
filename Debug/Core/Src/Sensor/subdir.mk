################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Sensor/AS5600.cpp \
../Core/Src/Sensor/BNO055.cpp \
../Core/Src/Sensor/Encoder.cpp \
../Core/Src/Sensor/LSM6DSOX.cpp \
../Core/Src/Sensor/LidarReader.cpp \
../Core/Src/Sensor/SSD1306_HardwareI2C.cpp \
../Core/Src/Sensor/SSD1306_SoftwareI2C.cpp \
../Core/Src/Sensor/UartComm.cpp 

OBJS += \
./Core/Src/Sensor/AS5600.o \
./Core/Src/Sensor/BNO055.o \
./Core/Src/Sensor/Encoder.o \
./Core/Src/Sensor/LSM6DSOX.o \
./Core/Src/Sensor/LidarReader.o \
./Core/Src/Sensor/SSD1306_HardwareI2C.o \
./Core/Src/Sensor/SSD1306_SoftwareI2C.o \
./Core/Src/Sensor/UartComm.o 

CPP_DEPS += \
./Core/Src/Sensor/AS5600.d \
./Core/Src/Sensor/BNO055.d \
./Core/Src/Sensor/Encoder.d \
./Core/Src/Sensor/LSM6DSOX.d \
./Core/Src/Sensor/LidarReader.d \
./Core/Src/Sensor/SSD1306_HardwareI2C.d \
./Core/Src/Sensor/SSD1306_SoftwareI2C.d \
./Core/Src/Sensor/UartComm.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Sensor/%.o Core/Src/Sensor/%.su Core/Src/Sensor/%.cyclo: ../Core/Src/Sensor/%.cpp Core/Src/Sensor/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Sensor" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Robot_Navigation" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Robot_BallRelated" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/Robot_Movement" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Sensor" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Robot_Navigation" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Robot_BallRelated" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/PID_Controller" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Src/PID_Controller" -I"C:/Users/Guy/STM32CubeIDE/workspace_1.12.1/STM32_Blackpill/Core/Inc/Robot_Movement" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Sensor

clean-Core-2f-Src-2f-Sensor:
	-$(RM) ./Core/Src/Sensor/AS5600.cyclo ./Core/Src/Sensor/AS5600.d ./Core/Src/Sensor/AS5600.o ./Core/Src/Sensor/AS5600.su ./Core/Src/Sensor/BNO055.cyclo ./Core/Src/Sensor/BNO055.d ./Core/Src/Sensor/BNO055.o ./Core/Src/Sensor/BNO055.su ./Core/Src/Sensor/Encoder.cyclo ./Core/Src/Sensor/Encoder.d ./Core/Src/Sensor/Encoder.o ./Core/Src/Sensor/Encoder.su ./Core/Src/Sensor/LSM6DSOX.cyclo ./Core/Src/Sensor/LSM6DSOX.d ./Core/Src/Sensor/LSM6DSOX.o ./Core/Src/Sensor/LSM6DSOX.su ./Core/Src/Sensor/LidarReader.cyclo ./Core/Src/Sensor/LidarReader.d ./Core/Src/Sensor/LidarReader.o ./Core/Src/Sensor/LidarReader.su ./Core/Src/Sensor/SSD1306_HardwareI2C.cyclo ./Core/Src/Sensor/SSD1306_HardwareI2C.d ./Core/Src/Sensor/SSD1306_HardwareI2C.o ./Core/Src/Sensor/SSD1306_HardwareI2C.su ./Core/Src/Sensor/SSD1306_SoftwareI2C.cyclo ./Core/Src/Sensor/SSD1306_SoftwareI2C.d ./Core/Src/Sensor/SSD1306_SoftwareI2C.o ./Core/Src/Sensor/SSD1306_SoftwareI2C.su ./Core/Src/Sensor/UartComm.cyclo ./Core/Src/Sensor/UartComm.d ./Core/Src/Sensor/UartComm.o ./Core/Src/Sensor/UartComm.su

.PHONY: clean-Core-2f-Src-2f-Sensor

