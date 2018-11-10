################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/home/jltm/Programas/CodeComposerStudio/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="/home/jltm/Documents/CodeComposer_Projects/F28335_SAPF_Control" --include_path="/home/jltm/Programas/CodeComposerStudio/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="/home/jltm/ti/c2000/C2000Ware_1_00_05_00/device_support/f2833x/common/include" --include_path="/home/jltm/ti/c2000/C2000Ware_1_00_05_00/device_support/f2833x/headers/include" -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/home/jltm/Programas/CodeComposerStudio/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="/home/jltm/Documents/CodeComposer_Projects/F28335_SAPF_Control" --include_path="/home/jltm/Programas/CodeComposerStudio/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="/home/jltm/ti/c2000/C2000Ware_1_00_05_00/device_support/f2833x/common/include" --include_path="/home/jltm/ti/c2000/C2000Ware_1_00_05_00/device_support/f2833x/headers/include" -g --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


