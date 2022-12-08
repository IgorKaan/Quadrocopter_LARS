################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/MadgwickFilter/MadgwickAHRS.cpp 

OBJS += \
./Core/Src/MadgwickFilter/MadgwickAHRS.o 

CPP_DEPS += \
./Core/Src/MadgwickFilter/MadgwickAHRS.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/MadgwickFilter/%.o Core/Src/MadgwickFilter/%.su: ../Core/Src/MadgwickFilter/%.cpp Core/Src/MadgwickFilter/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"C:/Users/LKI2/Desktop/Quadrocopter_LARS/GD103C8T6_Quadro_sensor_module/Core/Src/MadgwickFilter" -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-MadgwickFilter

clean-Core-2f-Src-2f-MadgwickFilter:
	-$(RM) ./Core/Src/MadgwickFilter/MadgwickAHRS.d ./Core/Src/MadgwickFilter/MadgwickAHRS.o ./Core/Src/MadgwickFilter/MadgwickAHRS.su

.PHONY: clean-Core-2f-Src-2f-MadgwickFilter

