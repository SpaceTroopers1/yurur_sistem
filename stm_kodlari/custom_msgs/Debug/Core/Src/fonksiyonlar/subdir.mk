################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fonksiyonlar/camera_kontrol.c \
../Core/Src/fonksiyonlar/motor_hiz_kontrol.c \
../Core/Src/fonksiyonlar/tanimlamalar.c 

OBJS += \
./Core/Src/fonksiyonlar/camera_kontrol.o \
./Core/Src/fonksiyonlar/motor_hiz_kontrol.o \
./Core/Src/fonksiyonlar/tanimlamalar.o 

C_DEPS += \
./Core/Src/fonksiyonlar/camera_kontrol.d \
./Core/Src/fonksiyonlar/motor_hiz_kontrol.d \
./Core/Src/fonksiyonlar/tanimlamalar.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/fonksiyonlar/%.o Core/Src/fonksiyonlar/%.su Core/Src/fonksiyonlar/%.cyclo: ../Core/Src/fonksiyonlar/%.c Core/Src/fonksiyonlar/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Core/src/fonksiyonlar -I../Core/Src/fonksiyonlar -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-fonksiyonlar

clean-Core-2f-Src-2f-fonksiyonlar:
	-$(RM) ./Core/Src/fonksiyonlar/camera_kontrol.cyclo ./Core/Src/fonksiyonlar/camera_kontrol.d ./Core/Src/fonksiyonlar/camera_kontrol.o ./Core/Src/fonksiyonlar/camera_kontrol.su ./Core/Src/fonksiyonlar/motor_hiz_kontrol.cyclo ./Core/Src/fonksiyonlar/motor_hiz_kontrol.d ./Core/Src/fonksiyonlar/motor_hiz_kontrol.o ./Core/Src/fonksiyonlar/motor_hiz_kontrol.su ./Core/Src/fonksiyonlar/tanimlamalar.cyclo ./Core/Src/fonksiyonlar/tanimlamalar.d ./Core/Src/fonksiyonlar/tanimlamalar.o ./Core/Src/fonksiyonlar/tanimlamalar.su

.PHONY: clean-Core-2f-Src-2f-fonksiyonlar

