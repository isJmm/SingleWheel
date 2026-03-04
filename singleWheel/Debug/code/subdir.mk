################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/ADIS16505-2.c \
../code/Kalman.c \
../code/Menu.c \
../code/control.c \
../code/dp.c \
../code/gyro.c \
../code/image.c \
../code/image_helper.c \
../code/imu.c \
../code/init.c \
../code/motor.c \
../code/my_math.c \
../code/navigation.c \
../code/pid.c \
../code/quater.c 

COMPILED_SRCS += \
./code/ADIS16505-2.src \
./code/Kalman.src \
./code/Menu.src \
./code/control.src \
./code/dp.src \
./code/gyro.src \
./code/image.src \
./code/image_helper.src \
./code/imu.src \
./code/init.src \
./code/motor.src \
./code/my_math.src \
./code/navigation.src \
./code/pid.src \
./code/quater.src 

C_DEPS += \
./code/ADIS16505-2.d \
./code/Kalman.d \
./code/Menu.d \
./code/control.d \
./code/dp.d \
./code/gyro.d \
./code/image.d \
./code/image_helper.d \
./code/imu.d \
./code/init.d \
./code/motor.d \
./code/my_math.d \
./code/navigation.d \
./code/pid.d \
./code/quater.d 

OBJS += \
./code/ADIS16505-2.o \
./code/Kalman.o \
./code/Menu.o \
./code/control.o \
./code/dp.o \
./code/gyro.o \
./code/image.o \
./code/image_helper.o \
./code/imu.o \
./code/init.o \
./code/motor.o \
./code/my_math.o \
./code/navigation.o \
./code/pid.o \
./code/quater.o 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/ÁÎ¾§/Desktop/DeskDoc/Document/19½ì¶ÀÂÖ/singleWheel_real/singleWheel/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
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
	-$(RM) ./code/ADIS16505-2.d ./code/ADIS16505-2.o ./code/ADIS16505-2.src ./code/Kalman.d ./code/Kalman.o ./code/Kalman.src ./code/Menu.d ./code/Menu.o ./code/Menu.src ./code/control.d ./code/control.o ./code/control.src ./code/dp.d ./code/dp.o ./code/dp.src ./code/gyro.d ./code/gyro.o ./code/gyro.src ./code/image.d ./code/image.o ./code/image.src ./code/image_helper.d ./code/image_helper.o ./code/image_helper.src ./code/imu.d ./code/imu.o ./code/imu.src ./code/init.d ./code/init.o ./code/init.src ./code/motor.d ./code/motor.o ./code/motor.src ./code/my_math.d ./code/my_math.o ./code/my_math.src ./code/navigation.d ./code/navigation.o ./code/navigation.src ./code/pid.d ./code/pid.o ./code/pid.src ./code/quater.d ./code/quater.o ./code/quater.src

.PHONY: clean-code

