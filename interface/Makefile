RM := rm -rf
LIBS := -lCMSISv1p30_LPC13xx

-include subdir.mk

all: main

# Tool invocations
main: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"./vendor/CMSISv1p30_LPC13xx/Debug" -Xlinker --gc-sections -Xlinker -Map=$(EXECUTABLE).map -mcpu=cortex-m3 -mthumb -T "linker/main.ld" -o"$(EXECUTABLE)" $(_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-@echo $(C_DEPS)
	-$(RM) $(_OBJS) $(_C_DEPS) $(EXECUTABLE) $(EXECUTABLE).map
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size $(EXECUTABLE); # arm-none-eabi-objcopy -O ihex blinky.axf blinky.hex ;
	-@echo ' '

flash:
	sudo openocd -f swd.cfg -f /usr/local/share/openocd/scripts/target/lpc13xx.cfg -c "program $(EXECUTABLE) reset exit"

setup-bb:
	sudo openocd -f board/dp_busblaster_v3.cfg -c "adapter_khz 1000; init; svf system.svf; shutdown"

debug:

.PHONY: all clean dependents
.SECONDARY: post-build
