################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/BarcodeDetector.cpp \
../src/DetectBarcode.cpp \
../src/MagickMat.cpp 

OBJS += \
./src/BarcodeDetector.o \
./src/DetectBarcode.o \
./src/MagickMat.o 

CPP_DEPS += \
./src/BarcodeDetector.d \
./src/DetectBarcode.d \
./src/MagickMat.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


