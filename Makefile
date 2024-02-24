CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc

CMSIS_CORE_DIR=CMSIS/Core
CMSIS_DEVICE_DIR=cmsis_device_l4
HAL_DIR=stm32l4xx_hal_driver

INVERTER_SRC_DIR=STM32CubeIDE_Code/Core/Src/
CFLAGS += -DSTM32L475xx \
	-mcpu=cortex-m3 -mthumb \
	-ISTM32CubeIDE_Code/Core/Inc/ \
	-I$(HAL_DIR)/Inc \
	-I$(CMSIS_DEVICE_DIR)/Include/ \
	-I$(CMSIS_CORE_DIR)/Include/

LDFLAGS = -T./STM32CubeIDE_Code/STM32L475RCTX_FLASH.ld

inverter_SRCS = $(addprefix $(INVERTER_SRC_DIR)/, \
       dac.c \
       dfsdm.c \
       dma.c \
       gpio.c \
       main.c \
       PID.c \
       PR.c \
       stm32l4xx_hal_msp.c \
       stm32l4xx_it.c \
       syscalls.c \
       sysmem.c \
       system_stm32l4xx.c \
       tim.c \
) STM32CubeIDE_Code/Core/Startup/startup_stm32l475rctx.s

inverter_C_SRCS = $(filter %.c, $(inverter_SRCS))
inverter_ASM_SRCS = $(filter %.s, $(inverter_SRCS))

inverter_OBJS = $(inverter_C_SRCS:.c=.o) $(inverter_ASM_SRCS:.s=.o)

hal_SRCS = $(addprefix $(HAL_DIR)/Src/, \
    stm32l4xx_hal_adc.c \
    stm32l4xx_hal_adc_ex.c \
    stm32l4xx_hal.c \
    stm32l4xx_hal_can.c \
    stm32l4xx_hal_comp.c \
    stm32l4xx_hal_cortex.c \
    stm32l4xx_hal_crc.c \
    stm32l4xx_hal_crc_ex.c \
    stm32l4xx_hal_cryp.c \
    stm32l4xx_hal_cryp_ex.c \
    stm32l4xx_hal_dac.c \
    stm32l4xx_hal_dac_ex.c \
    stm32l4xx_hal_dcmi.c \
    stm32l4xx_hal_dfsdm.c \
    stm32l4xx_hal_dfsdm_ex.c \
    stm32l4xx_hal_dma2d.c \
    stm32l4xx_hal_dma.c \
    stm32l4xx_hal_dma_ex.c \
    stm32l4xx_hal_dsi.c \
    stm32l4xx_hal_exti.c \
    stm32l4xx_hal_firewall.c \
    stm32l4xx_hal_flash.c \
    stm32l4xx_hal_flash_ex.c \
    stm32l4xx_hal_flash_ramfunc.c \
    stm32l4xx_hal_gfxmmu.c \
    stm32l4xx_hal_gpio.c \
    stm32l4xx_hal_hash.c \
    stm32l4xx_hal_hash_ex.c \
    stm32l4xx_hal_hcd.c \
    stm32l4xx_hal_i2c.c \
    stm32l4xx_hal_i2c_ex.c \
    stm32l4xx_hal_irda.c \
    stm32l4xx_hal_iwdg.c \
    stm32l4xx_hal_lcd.c \
    stm32l4xx_hal_lptim.c \
    stm32l4xx_hal_ltdc.c \
    stm32l4xx_hal_ltdc_ex.c \
    stm32l4xx_hal_mmc.c \
    stm32l4xx_hal_mmc_ex.c \
    stm32l4xx_hal_nand.c \
    stm32l4xx_hal_nor.c \
    stm32l4xx_hal_opamp.c \
    stm32l4xx_hal_opamp_ex.c \
    stm32l4xx_hal_ospi.c \
    stm32l4xx_hal_pcd.c \
    stm32l4xx_hal_pcd_ex.c \
    stm32l4xx_hal_pka.c \
    stm32l4xx_hal_pssi.c \
    stm32l4xx_hal_pwr.c \
    stm32l4xx_hal_pwr_ex.c \
    stm32l4xx_hal_qspi.c \
    stm32l4xx_hal_rcc.c \
    stm32l4xx_hal_rcc_ex.c \
    stm32l4xx_hal_rng.c \
    stm32l4xx_hal_rng_ex.c \
    stm32l4xx_hal_rtc.c \
    stm32l4xx_hal_rtc_ex.c \
    stm32l4xx_hal_sai.c \
    stm32l4xx_hal_sai_ex.c \
    stm32l4xx_hal_sd.c \
    stm32l4xx_hal_sd_ex.c \
    stm32l4xx_hal_smartcard.c \
    stm32l4xx_hal_smartcard_ex.c \
    stm32l4xx_hal_smbus.c \
    stm32l4xx_hal_smbus_ex.c \
    stm32l4xx_hal_spi.c \
    stm32l4xx_hal_spi_ex.c \
    stm32l4xx_hal_sram.c \
    stm32l4xx_hal_swpmi.c \
    stm32l4xx_hal_tim.c \
    stm32l4xx_hal_tim_ex.c \
    stm32l4xx_hal_tsc.c \
    stm32l4xx_hal_uart.c \
    stm32l4xx_hal_uart_ex.c \
    stm32l4xx_hal_usart.c \
    stm32l4xx_hal_usart_ex.c \
    stm32l4xx_hal_wwdg.c \
    stm32l4xx_ll_adc.c \
    stm32l4xx_ll_comp.c \
    stm32l4xx_ll_crc.c \
    stm32l4xx_ll_crs.c \
    stm32l4xx_ll_dac.c \
    stm32l4xx_ll_dma2d.c \
    stm32l4xx_ll_dma.c \
    stm32l4xx_ll_exti.c \
    stm32l4xx_ll_fmc.c \
    stm32l4xx_ll_gpio.c \
    stm32l4xx_ll_i2c.c \
    stm32l4xx_ll_lptim.c \
    stm32l4xx_ll_lpuart.c \
    stm32l4xx_ll_opamp.c \
    stm32l4xx_ll_pka.c \
    stm32l4xx_ll_pwr.c \
    stm32l4xx_ll_rcc.c \
    stm32l4xx_ll_rng.c \
    stm32l4xx_ll_rtc.c \
    stm32l4xx_ll_sdmmc.c \
    stm32l4xx_ll_spi.c \
    stm32l4xx_ll_swpmi.c \
    stm32l4xx_ll_tim.c \
    stm32l4xx_ll_usart.c \
    stm32l4xx_ll_usb.c \
    stm32l4xx_ll_utils.c)

hal_OBJS = $(hal_SRCS:.c=.o)

all: inverter.elf

%.o: %.s
	$(CC) $(CFLAGS) -c $< -o $@

inverter.elf: $(inverter_OBJS) $(hal_OBJS)
	$(CC) $(LDFLAGS) $(inverter_OBJS) $(hal_OBJS) -o $@

clean:
	rm -f $(inverter_OBJS) $(hal_OBJS)
