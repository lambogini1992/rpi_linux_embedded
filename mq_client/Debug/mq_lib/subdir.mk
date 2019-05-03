################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../mq_lib/msg_queue.c 

OBJS += \
./mq_lib/msg_queue.o 

C_DEPS += \
./mq_lib/msg_queue.d 


# Each subdirectory must supply rules for building sources it contributes
mq_lib/%.o: ../mq_lib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/anhpt26/eclipse-workspace/mq_client/mq_lib" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


