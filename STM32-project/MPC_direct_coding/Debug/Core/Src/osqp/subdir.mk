################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/osqp/auxil.c \
../Core/Src/osqp/error.c \
../Core/Src/osqp/lin_alg.c \
../Core/Src/osqp/osqp.c \
../Core/Src/osqp/proj.c \
../Core/Src/osqp/qdldl.c \
../Core/Src/osqp/qdldl_interface.c \
../Core/Src/osqp/scaling.c \
../Core/Src/osqp/util.c \
../Core/Src/osqp/workspace.c 

OBJS += \
./Core/Src/osqp/auxil.o \
./Core/Src/osqp/error.o \
./Core/Src/osqp/lin_alg.o \
./Core/Src/osqp/osqp.o \
./Core/Src/osqp/proj.o \
./Core/Src/osqp/qdldl.o \
./Core/Src/osqp/qdldl_interface.o \
./Core/Src/osqp/scaling.o \
./Core/Src/osqp/util.o \
./Core/Src/osqp/workspace.o 

C_DEPS += \
./Core/Src/osqp/auxil.d \
./Core/Src/osqp/error.d \
./Core/Src/osqp/lin_alg.d \
./Core/Src/osqp/osqp.d \
./Core/Src/osqp/proj.d \
./Core/Src/osqp/qdldl.d \
./Core/Src/osqp/qdldl_interface.d \
./Core/Src/osqp/scaling.d \
./Core/Src/osqp/util.d \
./Core/Src/osqp/workspace.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/osqp/%.o Core/Src/osqp/%.su: ../Core/Src/osqp/%.c Core/Src/osqp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-osqp

clean-Core-2f-Src-2f-osqp:
	-$(RM) ./Core/Src/osqp/auxil.d ./Core/Src/osqp/auxil.o ./Core/Src/osqp/auxil.su ./Core/Src/osqp/error.d ./Core/Src/osqp/error.o ./Core/Src/osqp/error.su ./Core/Src/osqp/lin_alg.d ./Core/Src/osqp/lin_alg.o ./Core/Src/osqp/lin_alg.su ./Core/Src/osqp/osqp.d ./Core/Src/osqp/osqp.o ./Core/Src/osqp/osqp.su ./Core/Src/osqp/proj.d ./Core/Src/osqp/proj.o ./Core/Src/osqp/proj.su ./Core/Src/osqp/qdldl.d ./Core/Src/osqp/qdldl.o ./Core/Src/osqp/qdldl.su ./Core/Src/osqp/qdldl_interface.d ./Core/Src/osqp/qdldl_interface.o ./Core/Src/osqp/qdldl_interface.su ./Core/Src/osqp/scaling.d ./Core/Src/osqp/scaling.o ./Core/Src/osqp/scaling.su ./Core/Src/osqp/util.d ./Core/Src/osqp/util.o ./Core/Src/osqp/util.su ./Core/Src/osqp/workspace.d ./Core/Src/osqp/workspace.o ./Core/Src/osqp/workspace.su

.PHONY: clean-Core-2f-Src-2f-osqp

