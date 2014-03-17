################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Arduino/hardware/arduino/sam/cores/arduino/avr/dtostrf.c 

C_DEPS += \
./arduino/dtostrf.c.d 

AR_OBJ += \
./arduino/dtostrf.c.o 


# Each subdirectory must supply rules for building sources it contributes
arduino/dtostrf.c.o: C:/Arduino/hardware/arduino/sam/cores/arduino/avr/dtostrf.c
	@echo 'Building file: $<'
	@echo 'Starting C compile'
	"C:/Arduino/hardware/tools/g++_arm_none_eabi/bin/arm-none-eabi-gcc" -c -g -Os -w -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -Dprintf=iprintf -mcpu=cortex-m3 -DF_CPU=84000000L -DARDUINO=155-r2 -DARDUINO_SAM_DUE -DARDUINO_ARCH_SAM -D__SAM3X8E__ -mthumb -DUSB_VID=0x2341 -DUSB_PID=0x003e -DUSBCON '-DUSB_MANUFACTURER="Unknown"' '-DUSB_PRODUCT="Arduino Due"' "-IC:/Arduino/hardware/arduino/sam/system/libsam" "-IC:/Arduino/hardware/arduino/sam/system/CMSIS/CMSIS/Include/" "-IC:/Arduino/hardware/arduino/sam/system/CMSIS/Device/ATMEL/"   -I"C:\Arduino\hardware\arduino\sam\cores\arduino" -I"C:\Arduino\hardware\arduino\sam\variants\arduino_due_x" -I"C:\Arduino\libraries\Hexabot" -I"C:\Arduino\libraries\I2C_PWM_Servo_Driver" -I"C:\Arduino\hardware\arduino\sam\libraries\Wire" -I"C:\Arduino\libraries\DueTimer" -I"C:\Arduino\libraries\Servo" -I"C:\Arduino\libraries\Servo\src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -D__IN_ECLIPSE__=1 "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


