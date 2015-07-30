################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /opt/ti/ccsv6/tools/compiler/arm_5.1.12

ORDERED_OBJS += \
"./GPS_task.obj" \
"./command_task.obj" \
"./compdcm_task.obj" \
"./ravvn.obj" \
"./ravvnuart.obj" \
"./startup_ccs.obj" \
"./utils/cmdline.obj" \
"./utils/flash_pb.obj" \
"./utils/locator.obj" \
"./utils/lwiplib.obj" \
"./utils/ringbuf.obj" \
"./utils/uartstdio.obj" \
"./utils/ustdlib.obj" \
"./third_party/FreeRTOS/Source/list.obj" \
"./third_party/FreeRTOS/Source/queue.obj" \
"./third_party/FreeRTOS/Source/tasks.obj" \
"./third_party/FreeRTOS/Source/timers.obj" \
"./third_party/FreeRTOS/Source/portable/MemMang/heap_1.obj" \
"./third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/port.obj" \
"./third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/portasm.obj" \
"./drivers/buttons.obj" \
"./drivers/pinout.obj" \
"../senshub_iot_ccs.cmd" \
$(GEN_CMDS__FLAG) \
-l"libc.a" \
-l"/home/chris/src/tivaware/examples/boards/ek-tm4c1294xl-boostxl-senshub/senshub_iot/ccs/../../../../../driverlib/ccs/Debug/driverlib.lib" \
-l"/home/chris/src/tivaware/examples/boards/ek-tm4c1294xl-boostxl-senshub/senshub_iot/ccs/../../../../../sensorlib/ccs/Debug/sensorlib.lib" \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include utils/subdir_vars.mk
-include third_party/FreeRTOS/Source/subdir_vars.mk
-include third_party/FreeRTOS/Source/portable/MemMang/subdir_vars.mk
-include third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/subdir_vars.mk
-include drivers/subdir_vars.mk
-include subdir_rules.mk
-include utils/subdir_rules.mk
-include third_party/FreeRTOS/Source/subdir_rules.mk
-include third_party/FreeRTOS/Source/portable/MemMang/subdir_rules.mk
-include third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/subdir_rules.mk
-include drivers/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
ravvn.out \

EXE_OUTPUTS__QUOTED += \
"ravvn.out" \

BIN_OUTPUTS += \
ravvn.bin \

BIN_OUTPUTS__QUOTED += \
"ravvn.bin" \


# All Target
all: ravvn.out

# Tool invocations
ravvn.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"/opt/ti/ccsv6/tools/compiler/arm_5.1.12/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 -g --gcc --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA0 --define=UART_BUFFERED --display_error_number --diag_warning=225 --diag_wrap=off --gen_func_subsections=on --ual -z -m"senshub_iot_ccs.map" --heap_size=0 --stack_size=16384 -i"/opt/ti/ccsv6/tools/compiler/arm_5.1.12/lib" -i"/opt/ti/ccsv6/tools/compiler/arm_5.1.12/include" --reread_libs --warn_sections --display_error_number --diag_wrap=off --xml_link_info="ravvn_linkInfo.xml" --rom_model -o "ravvn.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

ravvn.bin: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"/opt/ti/ccsv6/tools/compiler/arm_5.1.12/bin/armhex"  -o "ravvn.bin" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "GPS_task.pp" "command_task.pp" "compdcm_task.pp" "ravvn.pp" "ravvnuart.pp" "startup_ccs.pp" "utils/cmdline.pp" "utils/flash_pb.pp" "utils/locator.pp" "utils/lwiplib.pp" "utils/ringbuf.pp" "utils/uartstdio.pp" "utils/ustdlib.pp" "third_party/FreeRTOS/Source/list.pp" "third_party/FreeRTOS/Source/queue.pp" "third_party/FreeRTOS/Source/tasks.pp" "third_party/FreeRTOS/Source/timers.pp" "third_party/FreeRTOS/Source/portable/MemMang/heap_1.pp" "third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/port.pp" "drivers/buttons.pp" "drivers/pinout.pp" 
	-$(RM) "GPS_task.obj" "command_task.obj" "compdcm_task.obj" "ravvn.obj" "ravvnuart.obj" "startup_ccs.obj" "utils/cmdline.obj" "utils/flash_pb.obj" "utils/locator.obj" "utils/lwiplib.obj" "utils/ringbuf.obj" "utils/uartstdio.obj" "utils/ustdlib.obj" "third_party/FreeRTOS/Source/list.obj" "third_party/FreeRTOS/Source/queue.obj" "third_party/FreeRTOS/Source/tasks.obj" "third_party/FreeRTOS/Source/timers.obj" "third_party/FreeRTOS/Source/portable/MemMang/heap_1.obj" "third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/port.obj" "third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/portasm.obj" "drivers/buttons.obj" "drivers/pinout.obj" 
	-$(RM) "third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F/portasm.pp" 
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-"/opt/ti/ccsv6/utils/tiobj2bin/tiobj2bin" "ravvn.out" "ravvn.bin" "/opt/ti/ccsv6/tools/compiler/arm_5.1.12/bin/armofd" "/opt/ti/ccsv6/tools/compiler/arm_5.1.12/bin/armhex" "/opt/ti/ccsv6/utils/tiobj2bin/mkhex4bin"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets

