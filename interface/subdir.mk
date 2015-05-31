C_SRCS += \
main.c \
gpio.c \
timer32.c \
ssp.c \
uart.c \
clkconfig.c \
cr_startup_lpc13.c

ODIR = build

OBJS += \
main.o \
clkconfig.o \
gpio.o \
uart.o \
ssp.o \
timer32.o \
cr_startup_lpc13.o

_OBJS = $(patsubst %,$(ODIR)/%,$(OBJS))

C_DEPS += \
main.d \
clkconfig.d \
cr_startup_lpc13.d

_C_DEPS = $(patsubst %,$(ODIR)/%,$(C_DEPS))

INCLUDES = include/

EXECUTABLE = $(ODIR)/lpcbin


# Each subdirectory must supply rules for building sources it contributes
%.o: src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -I$(INCLUDES) -DDEBUG -D__USE_CMSIS=CMSISv1p30_LPC13xx -I"./vendor/CMSISv1p30_LPC13xx/inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(ODIR)/$(@:%.o=%.d)" -MT"$(ODIR)/$(@:%.o=%.d)" -std=c99 -o"$(ODIR)/$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


