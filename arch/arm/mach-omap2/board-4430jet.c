/*
 * Board support file for OMAP4430 JET
 *
 * Copyright (C) 2013 Recon Instruments
 *
 * Authors: Santosh Shilimkar <santosh.shilimkar@ti.com>
 * 	        Li Chen <li@reconinstruments.com>
 *          Gil Zhaiek <gil@reconinstruments.com>
 *
 * Based on mach-omap2/board-4430sdp.c / Copyright (C) Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/bq2415x.h>
#include <linux/gpio_keys.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/omapfb.h>
#include <linux/wl12xx.h>
#include <linux/memblock.h>
#include <linux/cdc_tcxo.h>

#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-micron.h>
#include <mach/dmm.h>
#include <mach/omap4_ion.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/android-display.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/omap4-keypad.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/remoteproc.h>
#include <video/omapdss.h>
#include <plat/vram.h>
#include <plat/omap-pm.h>
#include "board-jet.h"
#include "omap_ram_console.h"
#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "control.h"
#include "pm.h"
/* for TI WiLink devices */
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <plat/omap-serial.h>
#include <linux/omap4_duty_cycle_governor.h>

#if defined(CONFIG_JET_SUN) && defined(CONFIG_JET_V2)
#include <linux/jet_camera.h>
#endif

#if defined(CONFIG_JET_V2)
#define WILINK_UART_DEV_NAME "/dev/ttyO1" //ttyO0->uart1
#else
#define WILINK_UART_DEV_NAME "/dev/ttyO0" //ttyO0->uart1
#endif

#define GPIO_MFI_RESET      44
#define GPIO_WLAN_EN        43
#define GPIO_BT_EN          46
#define GPIO_TCXO_CLK_REQ_IN     48
#define GPIO_OFM_STANDBY    49
#define GPIO_WIFI_PMENA     GPIO_WLAN_EN
#define GPIO_WIFI_IRQ       53
#define GPIO_CAM_GLOBALRESET      83
#define GPIO_EYE_TRACING_INT   56
#define GPIO_SW_CAM_ON    175
#define GPIO_EVENT_INT    176

#define RFBI_A16_GPIO			163
#define A230_POWER_ENABLE_GPIO 52

#ifdef CONFIG_JET_V2
#define A230_CLK_TV_EN_GPIO		41
#define A230_DISP_RES_GPIO	42
#define LED_FLEX	32//Debug pin
#endif

#ifdef CONFIG_OMAP4_DUTY_CYCLE_GOVERNOR

static struct pcb_section omap4_duty_governor_pcb_sections[] = {
	{
		.pcb_temp_level         = 60,
		.max_opp            = 800000,
		.duty_cycle_enabled     = true,
		.tduty_params = {
			.nitro_rate     = 800000,
			.cooling_rate       = 300000,
			.nitro_interval     = 20000,
			.nitro_percentage   = 40,
		},
	},
	{
		.pcb_temp_level         = 70,
		.max_opp            = 800000,
		.duty_cycle_enabled     = true,
		.tduty_params = {
			.nitro_rate     = 800000,
			.cooling_rate       = 300000,
			.nitro_interval     = 20000,
			.nitro_percentage   = 35,
		},
	},
	{
		.pcb_temp_level         = 75,
		.max_opp            = 600000,
		.duty_cycle_enabled     = true,
		.tduty_params = {
			.nitro_rate     = 600000,
			.cooling_rate       = 300000,
			.nitro_interval     = 20000,
			.nitro_percentage   = 24,
		},
	},
	{
		.pcb_temp_level         = 80,
		.max_opp            = 600000,
		.duty_cycle_enabled     = true,
		.tduty_params = {
			.nitro_rate     = 600000,
			.cooling_rate       = 300000,
			.nitro_interval     = 20000,
			.nitro_percentage   = 10,
		},
	},
	{
		.pcb_temp_level         = 90,
		.max_opp            = 600000,
		.duty_cycle_enabled     = true,
		.tduty_params = {
			.nitro_rate     = 600000,
			.cooling_rate       = 300000,
			.nitro_interval     = 20000,
			.nitro_percentage   = 2,
		},
	},
	{
		.pcb_temp_level         = 110,
		.max_opp            = 600000,
		.duty_cycle_enabled     = true,
		.tduty_params = {
			.nitro_rate     = 600000,
			.cooling_rate       = 300000,
			.nitro_interval     = 10000,
			.nitro_percentage   = 1,
		},
	},
};

void init_duty_governor(void)
{
	omap4_duty_pcb_section_reg(omap4_duty_governor_pcb_sections,
		ARRAY_SIZE(omap4_duty_governor_pcb_sections));
}
#else
void init_duty_governor(void){}
#endif /*CONFIG_OMAP4_DUTY_CYCLE*/

/* TODO: handle suspend/resume here.
 * Upon every suspend, make sure the wilink chip is capable enough to wake-up the
 * OMAP host.
 */
static int plat_wlink_kim_suspend(struct platform_device *pdev, pm_message_t
		state)
{
	return 0;
}

static int plat_wlink_kim_resume(struct platform_device *pdev)
{
	return 0;
}

static bool uart_req;
static struct wake_lock st_wk_lock;
/* Call the uart disable of serial driver */
static int plat_uart_disable(void)
{
	int port_id = 0;
	int err = 0;
	if (uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_disable(port_id);
		if (!err)
			uart_req = false;
	}
	wake_unlock(&st_wk_lock);
	return err;
}

/* Call the uart enable of serial driver */
static int plat_uart_enable(void)
{
	int port_id = 0;
	int err = 0;
	if (!uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_enable(port_id);
		if (!err)
			uart_req = true;
	}
	wake_lock(&st_wk_lock);
	return err;
}

/* wl128x BT, FM, GPS connectivity chip */
static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = GPIO_BT_EN,
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3686400,
	.suspend = plat_wlink_kim_suspend,
	.resume = plat_wlink_kim_resume,
	.chip_asleep = plat_uart_disable,
	.chip_awake  = plat_uart_enable,
	.chip_enable = plat_uart_enable,
	.chip_disable = plat_uart_disable,
};

static struct platform_device wl128x_device = {
	.name       = "kim",
	.id     = -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

#if defined(CONFIG_JET_SUN) && defined(CONFIG_JET_V2)

static struct cam_plat_data cam_pdata = {
	.reset_gpio = GPIO_CAM_GLOBALRESET,
};
static struct platform_device camera_device = {
	.name = "cam_mt9m114",
	.id = -1,
	.dev.platform_data=&cam_pdata,
};

//#define CAMERA_INIT_ENABLE//enable this to enable camera in boot time
static void jet_camera_init(void)
{
	struct clk *src_clk, *parent_clk, *phy_ref_clk;
	int ret;

/*Configure clock*/
	//FREF_CLK1_OUT for CAM_CLK_A_12MP
	omap_mux_init_signal("fref_clk1_out.fref_clk1_out", OMAP_MUX_MODE0|OMAP_PIN_OUTPUT);

	src_clk = clk_get(NULL, "auxclk1_src_ck");
	if (IS_ERR(src_clk)) {
		pr_err("Cannot request source clock\n");
		goto src_clk_busy;
	}

	parent_clk=clk_get(NULL, "dpll_per_m3x2_ck");
	if (IS_ERR(parent_clk)) {
		pr_err("Cannot request dpll_per_m3x2_ck\n");
		goto parent_clk_busy;
	}
	ret = clk_set_parent(src_clk, parent_clk);
	if(ret<0) {
		pr_err("%s:%d:Cannot change clock \n", __FUNCTION__,__LINE__);
		goto parent_clk_busy;
	}
	ret = clk_set_rate(parent_clk, 192000000);
	if(ret<0){
		pr_err("%s:%d:Cannot set rate \n", __FUNCTION__,__LINE__);
		goto parent_clk_busy;
	}

	phy_ref_clk=clk_get(NULL, "auxclk1_ck");
	if (IS_ERR(phy_ref_clk)) {
		pr_err("Cannot request auxclk1_ck\n");
		goto aux_clk_busy;
	}
	ret = clk_set_rate(phy_ref_clk, 24000000);
	printk(KERN_INFO "%s:set rate return %d,0x%x\n", __FUNCTION__,ret,src_clk);
#ifdef CAMERA_INIT_ENABLE
	clk_enable(src_clk);
#else
	clk_disable(src_clk);
#endif

aux_clk_busy:
	clk_put(phy_ref_clk);
parent_clk_busy:
	clk_put(parent_clk);
src_clk_busy:
	if(ret<0)
		clk_put(src_clk);
	else
		cam_pdata.ext_clk=src_clk;

/*Configure pins*/
	//CAM_GLOBALRESET output low by default
	omap_mux_init_signal("cam_globalreset.gpio_83", OMAP_MUX_MODE3|OMAP_PIN_OUTPUT);
	gpio_request(cam_pdata.reset_gpio, "cam_reset");
#ifdef CAMERA_INIT_ENABLE
	mdelay(1);
	gpio_direction_output(cam_pdata.reset_gpio, 1);
#else
	gpio_direction_output(cam_pdata.reset_gpio, 0);
#endif
}
#endif
static struct platform_device *jet4430_devices[] __initdata = {
	&wl128x_device,
	&btwilink_device,
#if defined(CONFIG_JET_SUN) && defined(CONFIG_JET_V2)
	&camera_device,
#endif
};

static struct omap_board_config_kernel jet4430_config[] __initdata = {
};

static void __init omap_4430jet_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type     = MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode           = MUSB_OTG,
#else
	.mode           = MUSB_PERIPHERAL,
#endif
	.power          = 200,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc        = 2,
		.caps       = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd    = -EINVAL,
		.gpio_wp    = -EINVAL,
		.nonremovable   = true,
		.ocr_mask   = MMC_VDD_27_28,
		.no_off_init	= true,
		.built_in	= true,
	},
#ifndef CONFIG_JET_V2
	//TODO: disable second SD card interface in future
	{
		.mmc        = 1,
		.caps       = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA | MMC_CAP_1_8V_DDR,
		.gpio_wp    = -EINVAL,
	},
#endif
	{
		.mmc        = 5,
		.caps       = MMC_CAP_4_BIT_DATA| MMC_CAP_POWER_OFF_CARD,
		.pm_caps	= MMC_PM_KEEP_POWER,
		.gpio_cd    = -EINVAL,
		.gpio_wp    = -EINVAL,
		.ocr_mask   = MMC_VDD_165_195,
		.nonremovable   = true,
	},
	{}  /* Terminator */
};

static struct regulator_consumer_supply omap4_sdp4430_vmmc5_supply = {
	.supply = "vmmc",
	.dev_name = "omap_hsmmc.4",
};
static struct regulator_init_data sdp4430_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &omap4_sdp4430_vmmc5_supply,
};
static struct fixed_voltage_config sdp4430_vwlan = {
	.supply_name = "vwl1271",
	.microvolts = 1800000, /* 1.8V */
	.gpio = GPIO_WIFI_PMENA,
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &sdp4430_vmmc5,
};
static struct platform_device omap_vwlan_device = {
	.name       = "reg-fixed-voltage",
	.id     = 1,
	.dev = {
		.platform_data = &sdp4430_vwlan,
	}
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	/* Setting MMC5 SDIO card .built-in variable
	  * This is to make sure that if WiFi driver is not loaded
	  * at all, then the MMC/SD/SDIO driver does not keep
	  * turning on/off the voltage to the SDIO card
	  */
	//if (pdev->id == 4) {
		//ret = 0;
		//pdata->slots[0].mmc_data.built_in = 1;
		////pdata->slots[0].pm_caps = MMC_PM_KEEP_POWER;
	//}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed %s\n", __func__);
		return;
	}
	pdata = dev->platform_data;
	pdata->init =   omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(&c->pdev->dev);

	return 0;
}

static void __init jet_pmic_mux_init(void)
{
	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN);
}
static void omap4_audio_conf(void)
{
	/* I2S */
#if defined(CONFIG_JET_V2)
	omap_mux_init_signal("abe_mcbsp2_dr.abe_mcbsp2_dr", OMAP_MUX_MODE0|OMAP_PIN_INPUT);
	/* ABE_CLKS for DMIC(microphone) */
	omap_mux_init_signal("fref_clk2_out.fref_clk2_out", OMAP_MUX_MODE0|OMAP_PIN_OUTPUT);
#endif
	omap_mux_init_signal("abe_mcbsp2_dx.abe_mcbsp2_dx", OMAP_MUX_MODE0|OMAP_PIN_OUTPUT);
	omap_mux_init_signal("abe_mcbsp2_clkx.abe_mcbsp2_clkx", OMAP_MUX_MODE0|OMAP_PIN_OUTPUT);
	omap_mux_init_signal("abe_mcbsp2_fsx.abe_mcbsp2_fsx", OMAP_MUX_MODE0|OMAP_PIN_OUTPUT);
}
#if 0
static void omap4_extra_clk_init(void)
{
#if defined(CONFIG_JET_SUN) && defined(CONFIG_JET_V2)
/*FREF_CLK4_OUT for CLK_TV_OMAP, both snow and sun not use this clock*/
	omap_mux_init_signal("fref_clk4_out.fref_clk4_out", OMAP_MUX_MODE0|OMAP_PIN_OUTPUT);
	phy_ref_clk = clk_get(NULL, "auxclk4_ck");
	if (IS_ERR(phy_ref_clk)) {
		pr_err("Cannot request auxclk4\n");
		return;
	}
	clk_set_rate(phy_ref_clk, 38400000);
	clk_enable(phy_ref_clk);//the clock is on all the time
	clk_put(phy_ref_clk);
#endif
#if 0
//FREF_CLK5_OUT for CLK_TV_OMAP
	omap_mux_init_signal("fref_clk4_req.fref_clk5_out", OMAP_MUX_MODE1|OMAP_PIN_OUTPUT);

	phy_ref_clk = clk_get(NULL, "auxclk5_ck");
	if (IS_ERR(phy_ref_clk)) {
		pr_err("Cannot request auxclk5\n");
		return;
	}
	clk_set_rate(phy_ref_clk, 38400000);
	clk_enable(phy_ref_clk);//the clock is on all the time
	clk_put(phy_ref_clk);
#endif
}
#endif
static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
				struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request();
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata jet4430_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata jet4430_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata jet4430_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata jet4430_i2c_4_bus_pdata;

static int __init omap4_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &jet4430_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &jet4430_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &jet4430_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &jet4430_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &jet4430_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &jet4430_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &jet4430_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &jet4430_i2c_4_bus_pdata);

	return 0;
}

static int __init jet_power_init(void) {
	omap4_power_init();

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	return 0;
}

static int __init jet_gpio_init(void) {
	/*
	 * Drive MSECURE high for TWL6030/6032 write access.
	 */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	// Finger Navigation
	gpio_request(GPIO_OFM_STANDBY,"ofm_standby");
	gpio_direction_output(GPIO_OFM_STANDBY, 0);

	/*
	 * MFi reset pins
	 */
	omap_mux_init_signal("gpmc_a20.gpio_44", OMAP_MUX_MODE3|OMAP_PIN_OUTPUT);
	gpio_request(GPIO_MFI_RESET, "mfi_reset");
	gpio_direction_output(GPIO_MFI_RESET, 1);
	gpio_export(GPIO_MFI_RESET, 0);

	/*
	 * Set Wilink idle as init state
	 */
	gpio_request(GPIO_WLAN_EN,NULL);//fixed.c driver will request later, so set NULL label and free later
	gpio_direction_output(GPIO_WLAN_EN,0);

	gpio_request(GPIO_BT_EN,NULL);//st_kim.c driver will request later, so set NULL label and free later
	gpio_direction_output(GPIO_BT_EN,0);

	omap_mux_init_signal("gpmc_a24.gpio_48", OMAP_MUX_MODE3|OMAP_PIN_OUTPUT);//The pin is in safe mode setted by xloader
	gpio_request(GPIO_TCXO_CLK_REQ_IN,"tcxo_clk_req_in");
	if(gpio_direction_output(GPIO_TCXO_CLK_REQ_IN,0))
		printk(KERN_ERR "%s:GPIO_TCXO_CLK_REQ_IN err\n",__func__);
	//gpio_export(GPIO_TCXO_CLK_REQ_IN, 1);

	gpio_free(GPIO_WLAN_EN);
	gpio_free(GPIO_BT_EN);

#if defined(CONFIG_JET_SUN) && defined(CONFIG_JET_V2)
	//EYE_TRACING_INT
	omap_mux_init_signal("gpmc_nadv_ale.gpio_56", OMAP_MUX_MODE3|OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_EYE_TRACING_INT, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);

	//SW_CAM_ON, OFN-KEYPRESS in B3
	omap_mux_init_signal("kpd_row3.gpio_175", OMAP_MUX_MODE3|OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_SW_CAM_ON, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);

	//EVENT_INT, NLOFN0MOTION in B3
	omap_mux_init_signal("kpd_row4.gpio_176", OMAP_MUX_MODE3|OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_EVENT_INT, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);

#if 0//test prupose
	gpio_request(GPIO_EYE_TRACING_INT, "eye_tracing_int");
	gpio_direction_input(GPIO_EYE_TRACING_INT);
	gpio_request(GPIO_SW_CAM_ON, "sw_cam_on");
	gpio_direction_input(GPIO_SW_CAM_ON);
	gpio_request(GPIO_EVENT_INT, "event_int");
	gpio_direction_input(GPIO_EVENT_INT);
	gpio_export(GPIO_SW_CAM_ON, 1);
	gpio_export(GPIO_EVENT_INT, 1);
#endif
#endif
//test gpio pin
	//gpio_request(LED_FLEX, "LED_FLEX_GPIO");
	//gpio_direction_output(LED_FLEX,0);
//Kopin setting pins--------------------------------
	omap_mux_init_signal("gpmc_ncs2.gpio_52", OMAP_PIN_OUTPUT|OMAP_MUX_MODE3);
	gpio_request(RFBI_A16_GPIO, "RFBI_A16_GPIO");
	gpio_request(A230_POWER_ENABLE_GPIO, "A230_POWER_ENABLE_GPIO");
	gpio_direction_output(A230_POWER_ENABLE_GPIO, 1);//set high to work around for power surge
	gpio_direction_output(RFBI_A16_GPIO, 0);
#ifdef CONFIG_JET_V2
	//omap_mux_init_signal("gpmc_a17.gpio_41", OMAP_PIN_OUTPUT|OMAP_MUX_MODE3);
	gpio_request(A230_CLK_TV_EN_GPIO, "A230_CLK_TV_EN_GPIO");
	gpio_request(A230_DISP_RES_GPIO, "A230_DISP_RES_GPIO");
	gpio_direction_output(A230_CLK_TV_EN_GPIO, 0);
	gpio_direction_output(A230_DISP_RES_GPIO, 0);
#endif

	return 0;
}

static bool enable_suspend_off = true;
module_param(enable_suspend_off, bool, S_IRUSR | S_IRGRP | S_IROTH);

static int jet_enable_a230(struct omap_dss_device *dssdev)
{
		return 0;
}

static void jet_disable_a230(struct omap_dss_device *dssdev)
{

}
static struct omap_dss_device kopin_a230_lcd_device = {
		.type = OMAP_DISPLAY_TYPE_DBI,
		.name = "kopin_a230_panel",
		.driver_name = "kopin_a230_panel",
		.phy.rfbi.data_lines = 16,
		.phy.rfbi.channel = OMAP_DSS_CHANNEL_LCD,
		.platform_enable = jet_enable_a230,
		.platform_disable = jet_disable_a230,
};

static struct omap_dss_device *jet4430_dss_devices[] = {
		&kopin_a230_lcd_device,
};

static struct omap_dss_board_info jet4430_dss_data = {
	.num_devices    = ARRAY_SIZE(jet4430_dss_devices),
	.devices        = jet4430_dss_devices,
	.default_device = &kopin_a230_lcd_device,
};

#define JET_FB_RAM_SIZE                SZ_2M // 428*240*4*3FB
static struct omapfb_platform_data jet_fb_pdata = {
	.mem_desc = {
		.region_cnt = 2,
		.region = {
			[0] = {
				.size = JET_FB_RAM_SIZE,
			},
			[1] = {
				.size = JET_FB_RAM_SIZE,
			},
		},
	},
};

static void omap_4430jet_display_init(void)
{
	omapfb_set_platform_data(&jet_fb_pdata);
	omap_display_init(&jet4430_dss_data);
}

/*
 * LPDDR2 Configuration Data for 4430/4460 SOMs:
 * The memory organization is as below :
 *  EMIF1 - CS0 -   4 Gb
 *  EMIF2 - CS0 -   4 Gb
 *  --------------------
 *  TOTAL -     8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
static __initdata struct emif_device_details emif_devices = {
	.cs0_device = &lpddr2_micron_4G_S4_dev
};

static void omap4_jet4430_wifi_bt_mux_init(void)
{
	omap_mux_init_gpio(GPIO_WIFI_IRQ, OMAP_PIN_INPUT |
				OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(GPIO_WIFI_PMENA, OMAP_PIN_OUTPUT);

	omap_mux_init_signal("sdmmc5_cmd.sdmmc5_cmd",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_clk.sdmmc5_clk",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat0.sdmmc5_dat0",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat1.sdmmc5_dat1",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat2.sdmmc5_dat2",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat3.sdmmc5_dat3",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);

	//omap_mux_init_gpio(GPIO_BT_EN, OMAP_PIN_OUTPUT);
	//gpio_request(GPIO_WLAN_EN,"WLAN_EN");
	//gpio_direction_output(GPIO_WLAN_EN, 0);
	//gpio_export(GPIO_WLAN_EN, 0);
	//gpio_request(GPIO_BT_EN,"BT_EN");
	//gpio_direction_output(GPIO_BT_EN, 0);
	//gpio_export(GPIO_BT_EN, 0);
}

static struct wl12xx_platform_data omap4_jet4430_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = WL12XX_TCXOCLOCK_26, // TODO - we dont have tcxo
};

static void omap4_jet4430_wifi_bt_init(void)
{
	omap4_jet4430_wifi_bt_mux_init();
	if (wl12xx_set_platform_data(&omap4_jet4430_wlan_data))
		pr_err("Error setting wl12xx data\n");
	platform_device_register(&omap_vwlan_device);
}

static void jet_set_osc_timings(void)
{
	/* Device Oscilator
	 * tstart = 2ms + 2ms = 4ms.
	 * tshut = Not defined in oscillator data sheet so setting to 1us
	 */
	omap_pm_set_osc_lp_time(4000, 1);
}

static struct omap_board_mux board_mux[] __initdata = {
		OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
		{ .reg_offset = OMAP_MUX_TERMINATOR },
};

#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
struct usbhs_omap_board_data usbhs_bdata __initdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};
#endif

static void __init omap_4430jet_init(void)
{
	int package = OMAP_PACKAGE_CBS;
	omap4_mux_init(board_mux, NULL, package);
	jet_gpio_init();
	omap_emif_setup_device_details(&emif_devices, &emif_devices);

	omap_board_config = jet4430_config;
	omap_board_config_size = ARRAY_SIZE(jet4430_config);

	omap_init_board_version(0);
	omap4_audio_conf();
	omap4_create_board_props();
	jet_pmic_mux_init();
#if defined(CONFIG_JET_SUN) && defined(CONFIG_JET_V2)
	jet_camera_init();
#endif
	jet_set_osc_timings();
	omap4_i2c_init();
	jet_power_init();

	jet_sensor_init();
	jet_pwbutton_init();
	omap4_register_ion();
	platform_add_devices(jet4430_devices, ARRAY_SIZE(jet4430_devices));
	wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");
	omap4_jet4430_wifi_bt_init();
	omap4_twl6030_hsmmc_init(mmc);

	board_serial_init();

#ifdef CONFIG_MFD_OMAP_USB_HOST
	usbhs_init(&usbhs_bdata);
#endif
	usb_musb_init(&musb_board_data);

	omap_dmm_init();

	omap_4430jet_display_init();
	init_duty_governor();

	omap_enable_smartreflex_on_init();
	if (enable_suspend_off)
		omap_pm_enable_off_mode();
}

static void __init omap_4430jet_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}
static void __init omap_4430jet_reserve(void)
{
	omap_init_ram_size();

	omap_android_display_setup(&jet4430_dss_data,
				   NULL,
				   NULL,
				   &jet_fb_pdata,
#ifdef CONFIG_ION_OMAP
				   get_omap_ion_platform_data());
	omap_ion_init();
#else
				   NULL);
#endif

	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			OMAP_RAM_CONSOLE_SIZE_DEFAULT);

	/* do the static reservations first */
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM,
					PHYS_ADDR_DUCATI_SIZE +
					OMAP4_ION_HEAP_SECURE_INPUT_SIZE +
					OMAP4_ION_HEAP_SECURE_OUTPUT_WFDHDCP_SIZE);

	omap_reserve();
}

MACHINE_START(OMAP4_JET, "OMAP4 jet board")
	.boot_params    = 0x80000100,
	.reserve        = omap_4430jet_reserve,
	.map_io         = omap_4430jet_map_io,
	.init_early     = omap_4430jet_init_early,
	.init_irq       = gic_init_irq,
	.init_machine   = omap_4430jet_init,
	.timer          = &omap_timer,
MACHINE_END
