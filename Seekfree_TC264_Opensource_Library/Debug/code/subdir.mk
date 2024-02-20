################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/Camera.c \
../code/Cpu0Init.c \
../code/Cpu1Init.c \
../code/Encoder.c \
../code/Kalman.c \
../code/Motor.c \
../code/MotorRun.c \
../code/PID.c \
../code/Servo.c \
../code/lowpass_filter.c 

COMPILED_SRCS += \
./code/Camera.src \
./code/Cpu0Init.src \
./code/Cpu1Init.src \
./code/Encoder.src \
./code/Kalman.src \
./code/Motor.src \
./code/MotorRun.src \
./code/PID.src \
./code/Servo.src \
./code/lowpass_filter.src 

C_DEPS += \
./code/Camera.d \
./code/Cpu0Init.d \
./code/Cpu1Init.d \
./code/Encoder.d \
./code/Kalman.d \
./code/Motor.d \
./code/MotorRun.d \
./code/PID.d \
./code/Servo.d \
./code/lowpass_filter.d 

OBJS += \
./code/Camera.o \
./code/Cpu0Init.o \
./code/Cpu1Init.o \
./code/Encoder.o \
./code/Kalman.o \
./code/Motor.o \
./code/MotorRun.o \
./code/PID.o \
./code/Servo.o \
./code/lowpass_filter.o 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/TC264/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/code\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-code

clean-code:
	-$(RM) ./code/Camera.d ./code/Camera.o ./code/Camera.src ./code/Cpu0Init.d ./code/Cpu0Init.o ./code/Cpu0Init.src ./code/Cpu1Init.d ./code/Cpu1Init.o ./code/Cpu1Init.src ./code/Encoder.d ./code/Encoder.o ./code/Encoder.src ./code/Kalman.d ./code/Kalman.o ./code/Kalman.src ./code/Motor.d ./code/Motor.o ./code/Motor.src ./code/MotorRun.d ./code/MotorRun.o ./code/MotorRun.src ./code/PID.d ./code/PID.o ./code/PID.src ./code/Servo.d ./code/Servo.o ./code/Servo.src ./code/lowpass_filter.d ./code/lowpass_filter.o ./code/lowpass_filter.src

.PHONY: clean-code

