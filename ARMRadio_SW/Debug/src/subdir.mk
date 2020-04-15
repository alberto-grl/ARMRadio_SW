################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/SDR_InitPeriph.c \
../src/SDR_func.c \
../src/SDR_math.c \
../src/main.c \
C:/STM32F429I-Discovery_FW_V1.0.1/Libraries/STM32F4xx_StdPeriph_Driver/src/misc.c \
../src/stm32f429i_discovery.c \
../src/stm32f429i_discovery_ioe.c \
../src/stm32f429i_discovery_lcd.c \
../src/stm32f429i_discovery_sdram.c \
../src/stm32f4xx_adc.c \
../src/stm32f4xx_dac.c \
../src/stm32f4xx_dma.c \
../src/stm32f4xx_dma2d.c \
../src/stm32f4xx_exti.c \
../src/stm32f4xx_flash.c \
../src/stm32f4xx_fmc.c \
../src/stm32f4xx_gpio.c \
../src/stm32f4xx_i2c.c \
../src/stm32f4xx_it.c \
../src/stm32f4xx_ltdc.c \
../src/stm32f4xx_rcc.c \
../src/stm32f4xx_spi.c \
../src/stm32f4xx_syscfg.c \
../src/stm32f4xx_tim.c \
../src/system_stm32f4xx.c 

OBJS += \
./src/SDR_InitPeriph.o \
./src/SDR_func.o \
./src/SDR_math.o \
./src/main.o \
./src/misc.o \
./src/stm32f429i_discovery.o \
./src/stm32f429i_discovery_ioe.o \
./src/stm32f429i_discovery_lcd.o \
./src/stm32f429i_discovery_sdram.o \
./src/stm32f4xx_adc.o \
./src/stm32f4xx_dac.o \
./src/stm32f4xx_dma.o \
./src/stm32f4xx_dma2d.o \
./src/stm32f4xx_exti.o \
./src/stm32f4xx_flash.o \
./src/stm32f4xx_fmc.o \
./src/stm32f4xx_gpio.o \
./src/stm32f4xx_i2c.o \
./src/stm32f4xx_it.o \
./src/stm32f4xx_ltdc.o \
./src/stm32f4xx_rcc.o \
./src/stm32f4xx_spi.o \
./src/stm32f4xx_syscfg.o \
./src/stm32f4xx_tim.o \
./src/system_stm32f4xx.o 

C_DEPS += \
./src/SDR_InitPeriph.d \
./src/SDR_func.d \
./src/SDR_math.d \
./src/main.d \
./src/misc.d \
./src/stm32f429i_discovery.d \
./src/stm32f429i_discovery_ioe.d \
./src/stm32f429i_discovery_lcd.d \
./src/stm32f429i_discovery_sdram.d \
./src/stm32f4xx_adc.d \
./src/stm32f4xx_dac.d \
./src/stm32f4xx_dma.d \
./src/stm32f4xx_dma2d.d \
./src/stm32f4xx_exti.d \
./src/stm32f4xx_flash.d \
./src/stm32f4xx_fmc.d \
./src/stm32f4xx_gpio.d \
./src/stm32f4xx_i2c.d \
./src/stm32f4xx_it.d \
./src/stm32f4xx_ltdc.d \
./src/stm32f4xx_rcc.d \
./src/stm32f4xx_spi.d \
./src/stm32f4xx_syscfg.d \
./src/stm32f4xx_tim.d \
./src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F429ZITx -D__FPU_USED -DSTM32F429_439xx -D__FPU_PRESENT -DUSE_STDPERIPH_DRIVER -DSTM32F429_439xx -DUSE_STM32F429I_DISCO -DSTM32F429I_DISC1 -DSTM32F4 -DSTM32 -DDEBUG -DARM_MATH_CM4 -I"C:/Users/user/ArmRadio_SW/ARMRadio_SW/inc" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Utilities/STM32F429I-Discovery" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Libraries/CMSIS/Include" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Libraries/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Libraries/STM32F4xx_StdPeriph_Driver/inc" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Utilities/Common" -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/misc.o: C:/STM32F429I-Discovery_FW_V1.0.1/Libraries/STM32F4xx_StdPeriph_Driver/src/misc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F429ZITx -D__FPU_USED -DSTM32F429_439xx -D__FPU_PRESENT -DUSE_STDPERIPH_DRIVER -DSTM32F429_439xx -DUSE_STM32F429I_DISCO -DSTM32F429I_DISC1 -DSTM32F4 -DSTM32 -DDEBUG -DARM_MATH_CM4 -I"C:/Users/user/ArmRadio_SW/ARMRadio_SW/inc" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Utilities/STM32F429I-Discovery" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Libraries/CMSIS/Include" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Libraries/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Libraries/STM32F4xx_StdPeriph_Driver/inc" -I"C:/STM32F429I-Discovery_FW_V1.0.1/Utilities/Common" -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


