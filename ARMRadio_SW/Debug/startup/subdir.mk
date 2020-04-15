################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32.s 

C_SRCS += \
../startup/sysmem.c 

OBJS += \
./startup/startup_stm32.o \
./startup/sysmem.o 

C_DEPS += \
./startup/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"C:/Users/user/ArmRadio_SW/ARMRadio_SW/inc" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

startup/%.o: ../startup/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F429ZITx -D__FPU_USED -DSTM32F429_439xx -D__FPU_PRESENT -DUSE_STDPERIPH_DRIVER -DSTM32F429_439xx -DUSE_STM32F429I_DISCO -DSTM32F429I_DISC1 -DSTM32F4 -DSTM32 -DDEBUG -DARM_MATH_CM4 -I"C:/Users/user/ArmRadio_SW/ARMRadio_SW/inc" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Utilities/STM32F429I-Discovery" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Libraries/CMSIS/Include" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Libraries/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Libraries/STM32F4xx_StdPeriph_Driver/inc" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Utilities/Common" -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


