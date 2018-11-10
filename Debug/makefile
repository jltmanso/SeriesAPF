################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /home/jltm/Programas/CodeComposerStudio/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./DSP2833x_ADC_cal.obj" \
"./DSP2833x_CodeStartBranch.obj" \
"./DSP2833x_CpuTimers.obj" \
"./DSP2833x_DefaultIsr.obj" \
"./DSP2833x_GlobalVariableDefs.obj" \
"./DSP2833x_Gpio.obj" \
"./DSP2833x_PieCtrl.obj" \
"./DSP2833x_PieVect.obj" \
"./DSP2833x_SysCtrl.obj" \
"./DSP2833x_usDelay.obj" \
"./main.obj" \
"./SAPF/src/SAPF_Control.obj" \
"./SAPF/src/SAPF_GPIO.obj" \
"./System/src/System.obj" \
$(GEN_CMDS__FLAG) \
-lrts2800_fpu32.lib \
-l28335_RAM_lnk.cmd \
-lDSP2833x_Headers_nonBIOS.cmd \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include SAPF/src/subdir_vars.mk
-include System/src/subdir_vars.mk
-include subdir_rules.mk
-include SAPF/src/subdir_rules.mk
-include System/src/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
F28335_SAPF_Control.out \

EXE_OUTPUTS__QUOTED += \
"F28335_SAPF_Control.out" \

BIN_OUTPUTS += \
F28335_SAPF_Control.hex \

BIN_OUTPUTS__QUOTED += \
"F28335_SAPF_Control.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "F28335_SAPF_Control.out"

# Tool invocations
F28335_SAPF_Control.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"/home/jltm/Programas/CodeComposerStudio/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --diag_warning=225 --diag_wrap=off --display_error_number -z -m"F28335_SAPF_Control.map" --stack_size=0x300 --warn_sections -i"/home/jltm/Programas/CodeComposerStudio/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/lib" -i"/home/jltm/ti/c2000/C2000Ware_1_00_05_00/device_support/f2833x/headers/cmd" -i"/home/jltm/ti/c2000/C2000Ware_1_00_05_00/device_support/f2833x/common/cmd" -i"/home/jltm/Programas/CodeComposerStudio/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --reread_libs --diag_wrap=off --display_error_number --xml_link_info="F28335_SAPF_Control_linkInfo.xml" --entry_point=code_start --rom_model -o "F28335_SAPF_Control.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

F28335_SAPF_Control.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: C2000 Hex Utility'
	"/home/jltm/Programas/CodeComposerStudio/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/hex2000"  -o "F28335_SAPF_Control.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "DSP2833x_ADC_cal.obj" "DSP2833x_CodeStartBranch.obj" "DSP2833x_CpuTimers.obj" "DSP2833x_DefaultIsr.obj" "DSP2833x_GlobalVariableDefs.obj" "DSP2833x_Gpio.obj" "DSP2833x_PieCtrl.obj" "DSP2833x_PieVect.obj" "DSP2833x_SysCtrl.obj" "DSP2833x_usDelay.obj" "main.obj" "SAPF/src/SAPF_Control.obj" "SAPF/src/SAPF_GPIO.obj" "System/src/System.obj" 
	-$(RM) "DSP2833x_CpuTimers.d" "DSP2833x_DefaultIsr.d" "DSP2833x_GlobalVariableDefs.d" "DSP2833x_Gpio.d" "DSP2833x_PieCtrl.d" "DSP2833x_PieVect.d" "DSP2833x_SysCtrl.d" "main.d" "SAPF/src/SAPF_Control.d" "SAPF/src/SAPF_GPIO.d" "System/src/System.d" 
	-$(RM) "DSP2833x_ADC_cal.d" "DSP2833x_CodeStartBranch.d" "DSP2833x_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

