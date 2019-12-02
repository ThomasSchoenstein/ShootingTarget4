################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include protocol/bluetooth/lib/EFR32BG1B/GCC/subdir.mk
-include platform/emdrv/tempdrv/src/subdir.mk
-include platform/emdrv/sleep/src/subdir.mk
-include platform/Device/SiliconLabs/BGM1/Source/GCC/subdir.mk
-include platform/Device/SiliconLabs/BGM1/Source/subdir.mk
-include hardware/kit/common/drivers/subdir.mk
-include hardware/kit/common/bsp/subdir.mk
-include emlib/subdir.mk
-include app/bluetooth/common/util/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: soc-thermometer_4.axf

# Tool invocations
soc-thermometer_4.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "C:\Users\jdoe8\Desktop\School\Embedded Systems\TargetCode\4 try\4th Try\hjk\bgm111a256v2.ld" -Xlinker --gc-sections -Xlinker -Map="soc-thermometer_4.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o soc-thermometer_4.axf -Wl,--start-group "./app/bluetooth/common/util/infrastructure.o" "./application_properties.o" "./gatt_db.o" "./init_app.o" "./init_board.o" "./init_mcu.o" "./main.o" "./pti.o" "./targetFunctions.o" "./emlib/em_acmp.o" "./emlib/em_adc.o" "./emlib/em_aes.o" "./emlib/em_assert.o" "./emlib/em_burtc.o" "./emlib/em_can.o" "./emlib/em_cmu.o" "./emlib/em_core.o" "./emlib/em_cryotimer.o" "./emlib/em_crypto.o" "./emlib/em_csen.o" "./emlib/em_dac.o" "./emlib/em_dbg.o" "./emlib/em_dma.o" "./emlib/em_ebi.o" "./emlib/em_emu.o" "./emlib/em_eusart.o" "./emlib/em_gpcrc.o" "./emlib/em_gpio.o" "./emlib/em_i2c.o" "./emlib/em_iadc.o" "./emlib/em_idac.o" "./emlib/em_int.o" "./emlib/em_lcd.o" "./emlib/em_ldma.o" "./emlib/em_lesense.o" "./emlib/em_letimer.o" "./emlib/em_leuart.o" "./emlib/em_mpu.o" "./emlib/em_msc.o" "./emlib/em_opamp.o" "./emlib/em_pcnt.o" "./emlib/em_pdm.o" "./emlib/em_prs.o" "./emlib/em_qspi.o" "./emlib/em_rmu.o" "./emlib/em_rtc.o" "./emlib/em_rtcc.o" "./emlib/em_se.o" "./emlib/em_system.o" "./emlib/em_timer.o" "./emlib/em_usart.o" "./emlib/em_vcmp.o" "./emlib/em_vdac.o" "./emlib/em_wdog.o" "./hardware/kit/common/bsp/bsp_stk.o" "./hardware/kit/common/drivers/i2cspm.o" "./hardware/kit/common/drivers/si7013.o" "./hardware/kit/common/drivers/tempsens.o" "./hardware/kit/common/drivers/udelay.o" "./platform/Device/SiliconLabs/BGM1/Source/GCC/startup_bgm1.o" "./platform/Device/SiliconLabs/BGM1/Source/system_bgm1.o" "./platform/emdrv/sleep/src/sleep.o" "./platform/emdrv/tempdrv/src/tempdrv.o" "C:\Users\jdoe8\Desktop\School\Embedded Systems\TargetCode\4 try\4th Try\hjk\protocol\bluetooth\lib\EFR32BG1B\GCC\libmbedtls.a" "C:\Users\jdoe8\Desktop\School\Embedded Systems\TargetCode\4 try\4th Try\hjk\protocol\bluetooth\lib\EFR32BG1B\GCC\libbluetooth.a" "C:\Users\jdoe8\Desktop\School\Embedded Systems\TargetCode\4 try\4th Try\hjk\protocol\bluetooth\lib\EFR32BG1B\GCC\libpsstore.a" "C:\Users\jdoe8\Desktop\School\Embedded Systems\TargetCode\4 try\4th Try\hjk\protocol\bluetooth\lib\EFR32BG1B\GCC\binapploader.o" "C:\Users\jdoe8\Desktop\School\Embedded Systems\TargetCode\4 try\4th Try\hjk\platform\radio\rail_lib\autogen\librail_release\librail_module_efr32xg1_gcc_release.a" "C:\Users\jdoe8\Desktop\School\Embedded Systems\TargetCode\4 try\4th Try\hjk\platform\radio\rail_lib\autogen\librail_release\librail_config_bgm111a256v2_gcc.a" -lm -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: soc-thermometer_4.hex'
	arm-none-eabi-objcopy -O ihex "soc-thermometer_4.axf" "soc-thermometer_4.hex"
	@echo ' '

	@echo 'Building bin file: soc-thermometer_4.bin'
	arm-none-eabi-objcopy -O binary "soc-thermometer_4.axf" "soc-thermometer_4.bin"
	@echo ' '

	@echo 'Building s37 file: soc-thermometer_4.s37'
	arm-none-eabi-objcopy -O srec "soc-thermometer_4.axf" "soc-thermometer_4.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "soc-thermometer_4.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) soc-thermometer_4.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
