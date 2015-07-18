################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
GPS_task.obj: ../GPS_task.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/opt/ti/ccsv6/tools/compiler/arm_5.1.12/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/opt/ti/ccsv6/tools/compiler/arm_5.1.12/include" --include_path="/home/chris/workspace_sd/ravvn" --include_path="/home/chris/src/tivaware/examples/boards/ek-tm4c1294xl-boostxl-senshub" --include_path="/home/chris/src/tivaware" --include_path="/home/chris/src/tivaware/third_party" --include_path="/home/chris/src/tivaware/third_party/exosite" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS/Source/include" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/src/include" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/src/include/ipv4" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/apps" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/ports/tiva-tm4c129/include" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F" -g --gcc --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA0 --define=UART_BUFFERED --display_error_number --diag_warning=225 --diag_wrap=off --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="GPS_task.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

command_task.obj: ../command_task.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/opt/ti/ccsv6/tools/compiler/arm_5.1.12/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/opt/ti/ccsv6/tools/compiler/arm_5.1.12/include" --include_path="/home/chris/workspace_sd/ravvn" --include_path="/home/chris/src/tivaware/examples/boards/ek-tm4c1294xl-boostxl-senshub" --include_path="/home/chris/src/tivaware" --include_path="/home/chris/src/tivaware/third_party" --include_path="/home/chris/src/tivaware/third_party/exosite" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS/Source/include" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/src/include" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/src/include/ipv4" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/apps" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/ports/tiva-tm4c129/include" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F" -g --gcc --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA0 --define=UART_BUFFERED --display_error_number --diag_warning=225 --diag_wrap=off --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="command_task.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

compdcm_task.obj: ../compdcm_task.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/opt/ti/ccsv6/tools/compiler/arm_5.1.12/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/opt/ti/ccsv6/tools/compiler/arm_5.1.12/include" --include_path="/home/chris/workspace_sd/ravvn" --include_path="/home/chris/src/tivaware/examples/boards/ek-tm4c1294xl-boostxl-senshub" --include_path="/home/chris/src/tivaware" --include_path="/home/chris/src/tivaware/third_party" --include_path="/home/chris/src/tivaware/third_party/exosite" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS/Source/include" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/src/include" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/src/include/ipv4" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/apps" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/ports/tiva-tm4c129/include" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F" -g --gcc --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA0 --define=UART_BUFFERED --display_error_number --diag_warning=225 --diag_wrap=off --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="compdcm_task.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

senshub_iot.obj: ../senshub_iot.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/opt/ti/ccsv6/tools/compiler/arm_5.1.12/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/opt/ti/ccsv6/tools/compiler/arm_5.1.12/include" --include_path="/home/chris/workspace_sd/ravvn" --include_path="/home/chris/src/tivaware/examples/boards/ek-tm4c1294xl-boostxl-senshub" --include_path="/home/chris/src/tivaware" --include_path="/home/chris/src/tivaware/third_party" --include_path="/home/chris/src/tivaware/third_party/exosite" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS/Source/include" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/src/include" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/src/include/ipv4" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/apps" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/ports/tiva-tm4c129/include" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F" -g --gcc --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA0 --define=UART_BUFFERED --display_error_number --diag_warning=225 --diag_wrap=off --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="senshub_iot.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

startup_ccs.obj: ../startup_ccs.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/opt/ti/ccsv6/tools/compiler/arm_5.1.12/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -O2 --include_path="/opt/ti/ccsv6/tools/compiler/arm_5.1.12/include" --include_path="/home/chris/workspace_sd/ravvn" --include_path="/home/chris/src/tivaware/examples/boards/ek-tm4c1294xl-boostxl-senshub" --include_path="/home/chris/src/tivaware" --include_path="/home/chris/src/tivaware/third_party" --include_path="/home/chris/src/tivaware/third_party/exosite" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS/Source/include" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/src/include" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/src/include/ipv4" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/apps" --include_path="/home/chris/src/tivaware/third_party/lwip-1.4.1/ports/tiva-tm4c129/include" --include_path="/home/chris/src/tivaware/third_party/FreeRTOS/Source/portable/CCS/ARM_CM4F" -g --gcc --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA0 --define=UART_BUFFERED --display_error_number --diag_warning=225 --diag_wrap=off --gen_func_subsections=on --ual --preproc_with_compile --preproc_dependency="startup_ccs.pp" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


