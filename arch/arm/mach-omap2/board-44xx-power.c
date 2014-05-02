/* OMAP Identity file for OMAP4 boards.
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * Based on
 * mach-omap2/board-44xx-tablet.c
 * mach-omap2/board-4430sdp.c
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#if defined(CONFIG_JET_SUN) && defined(CONFIG_JET_V2)
//TODO: remove the tow defines below after cammera module drivers registered
#define CONFIG_PUT_A_CAMERA_HERE
#define CAMERA_ALWAYS_ON
#endif
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/tps6130x.h>
#include "mux.h"
#include <linux/i2c/twl.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/omap-serial.h>
#include <linux/mfd/twl6040-codec.h>
#include "common-board-devices.h"
#if defined(CONFIG_JET_V2)
//emmc use mmc1 interface
static struct regulator_consumer_supply vmmc_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1"),
};
#else
static struct regulator_consumer_supply vmmc_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0"),
};
#endif

static struct regulator_init_data vmmc = {
	.constraints = {
#if defined(CONFIG_JET_V2)
		.min_uV			= 2800000,
#else
		.min_uV			= 1200000,
#endif
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		}
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = vmmc_supply,
};

static struct regulator_init_data vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data vusim = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_consumer_supply vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

static struct regulator_init_data vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies	= ARRAY_SIZE(vcxio_supply),
	.consumer_supplies	= vcxio_supply,
};

static struct regulator_consumer_supply vdac_supply[] = {
	{
		.supply = "hdmi_vref",
	},
};

static struct regulator_init_data vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = ARRAY_SIZE(vdac_supply),
	.consumer_supplies      = vdac_supply,
};

static struct regulator_consumer_supply vusb_supply[] = {
	REGULATOR_SUPPLY("vusb", "twl6030_usb"),
};

static struct regulator_init_data vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = ARRAY_SIZE(vusb_supply),
	.consumer_supplies      = vusb_supply,
};

//externel sd card use mmc0 interface
static struct regulator_consumer_supply vaux1_supply[] = {
#if !defined(CONFIG_JET_SUN) || !defined(CONFIG_JET_V2)
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0"),
#endif
#ifdef CONFIG_MOUSE_OFM_PARTRON
	REGULATOR_SUPPLY("ofm_switch", "ofm_driver"),
#endif
};

static struct regulator_init_data vaux1 = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
	},
#if defined(CONFIG_JET_SUN) && defined(CONFIG_JET_V2)
#ifdef CONFIG_MOUSE_OFM_PARTRON
	.num_consumer_supplies  = 1,
	.consumer_supplies      = vaux1_supply,
#endif
#else
#ifdef CONFIG_MOUSE_OFM_PARTRON
	.num_consumer_supplies  = 2,
#else
	.num_consumer_supplies  = 1,
#endif
	.consumer_supplies      = vaux1_supply,
#endif
	
};

#ifdef CONFIG_MACH_OMAP4_JET
static struct regulator_consumer_supply vaux2_supply[] = {
	REGULATOR_SUPPLY("switch", "jet_sensors"),
};
#else
static struct regulator_consumer_supply vaux2_supply[] = {
	REGULATOR_SUPPLY("av-switch", "soc-audio"),
};
#endif
static struct regulator_init_data vaux2 = {
	.constraints = {
		.min_uV			= 2500000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		}
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= vaux2_supply,
};

#ifdef CONFIG_PUT_A_CAMERA_HERE
static struct regulator_consumer_supply vaux3_supply[] = {
};
#endif

static struct regulator_init_data vaux3 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
#ifdef CAMERA_ALWAYS_ON
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.always_on		= true,
	},
#else

		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
#ifdef CONFIG_PUT_A_CAMERA_HERE
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = vaux3_supply,
#else
		.initial_state          = PM_SUSPEND_MEM,
	},
#endif
#endif
};

static struct regulator_init_data clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct regulator_init_data clk32kaudio = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
#ifdef CONFIG_MACH_OMAP4_JET
		/*Disable clk32kaudio*/
		.always_on		= false,
#else
		.always_on		= true,
#endif
	},
};

static struct regulator_init_data v2v1 = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data sysen = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data regen1 = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data vcore1	= {
	.constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
		.always_on              = true,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data vcore2	= {
	.constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
		.always_on              = true,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data vcore3 = {
	.constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data vmem = {
	.constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};
#if defined(CONFIG_JET_V2)
static int batt_table[] = {
	/* adc threshold,correponding temperature(0.01C) and slope */
	896, -2888, 16,
	882, -2678, 15,
	866, -2453, 14,
	846, -2193, 13,
	819, -1868, 12,
	780, -1439, 11,
	715, -792, 10,
	468, 1458, 9,
	393, 2203, 10,
	346, 2718, 11,
	311, 3136, 12,
	284, 3485, 13,
	262, 3792, 14,
	244, 4061, 15,
	228, 4315, 16,
	215, 4535, 17,
	203, 4750, 18,
	192, 4958, 19,
	183, 5137, 20,
	175, 5304, 21,
	167, 5479, 22,
	160, 5639, 23,
	154, 5782, 24,
	149, 5906, 25,
	143, 6061, 26,
	138, 6196, 27,
	134, 6307, 28,
	130, 6422, 29,
	122, 6663, 31,
	116, 6856, 33,
	110, 7061, 35,
};
#else
static int batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925, /* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
	511, 504, 496 /* 60 - 62 */
};
#endif
#ifdef CONFIG_FUEL_GAUGE
/* Fuel Gauge EDV Configuration */
static struct edv_config edv_cfg = {
	.averaging = true,
	.seq_edv = 5,
	.filter_light = 155,
	.filter_heavy = 199,
	.overload_current = 1000,
	.edv = {
		{SHUTDOWN_VOLTAGE, 0},
		{3615, 5},
		{3645, EDV_FIRST_CHECK_POINT},
	},
};
/* Fuel Gauge OCV Configuration */
static struct ocv_config ocv_cfg = {
	.voltage_diff = 75,
	.current_diff = 30,

	.sleep_enter_current = 60,
	.sleep_enter_samples = 3,

	.sleep_exit_current = 100,
	.sleep_exit_samples = 3,

	.long_sleep_current = 500,
	.ocv_period = 300,
	.relax_period = 600,

	.flat_zone_low = 35,
	.flat_zone_high = 65,

#ifdef CONFIG_JET_SUN
	.max_ocv_discharge = 530,
#else
	.max_ocv_discharge = 1300,
#endif
#ifndef CONFIG_JET_V2
	.table = {
		3450, 3552, 3576, 3598, 3618,
		3637, 3655, 3673, 3690, 3708,
		3727, 3748, 3769, 3793, 3819,
		3848, 3881, 3917, 3957, 4002,
		4051
	},
#endif
};
/* General OMAP4 Battery Cell Configuration */
static struct cell_config cell_cfg =  {
	.cc_voltage = 4175,
	.cc_current =100,//250,
	.cc_capacity = 15,
	.seq_cc = 5,

	.cc_polarity = true,
	.cc_out = true,
	.ocv_below_edv1 = false,
#ifdef CONFIG_JET_SUN
	.design_capacity = 470,
	.design_qmax = 530,
#else
	.design_capacity = 1183,
	.design_qmax = 1250,//4100,
#endif
	.r_sense = 20,//10,

	.qmax_adjust = 1,
	.fcc_adjust = 2,

	.max_overcharge = 100,
	.electronics_load = 200, /* *10 uAh */

#ifdef CONFIG_JET_SUN
	.max_increment = 50,
	.max_decrement = 50,
#else
	.max_increment = 150,
	.max_decrement = 150,
#endif
	.deep_dsg_voltage = 30,
	.max_dsg_estimate = 300,
	.light_load = 100,
	.near_full = 500,
	.cycle_threshold = 3500,
	.recharge = 60,

	.mode_switch_capacity = 5,

	.call_period = 2,

	.ocv = &ocv_cfg,
	.edv = &edv_cfg,
};
#endif
static struct twl4030_bci_platform_data bci_data = {
	.monitoring_interval		= 10,
#ifdef CONFIG_JET_SUN
	.max_charger_currentmA		= 400,
#else
	.max_charger_currentmA		= 750,
#endif
	.max_charger_voltagemV		= 4200,
	//.termination_currentmA		=
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= batt_table,
	.tblsize			= ARRAY_SIZE(batt_table),
#ifdef CONFIG_FUEL_GAUGE
	.cell_cfg			= &cell_cfg,
#else
#ifdef CONFIG_JET_SUN
#ifdef CONFIG_JET_V1
	.max_battery_capacity		= 400,
#else // CONFIG_JET_V2
    .max_battery_capacity       = 500,
#endif
#else
	.max_battery_capacity		= 1200,
#endif
	.sense_resistor_mohm		= 20,//10
#endif
};

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_suspend	= omap4430_phy_suspend,
};
#ifdef CONFIG_TWL6040_CODEC
static struct twl4030_codec_audio_data twl6040_audio = {
	/* single-step ramp for headset and handsfree */
	.hs_left_step   = 0x0f,
	.hs_right_step  = 0x0f,
	.hf_left_step   = 0x1d,
	.hf_right_step  = 0x1d,
	.vddhf_uV	= 4075000,
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
	.max_timeout	= 15000,
	.initial_vibrate = 0,
	.voltage_raise_speed = 0x26,
};

static int twl6040_init(void)
{
	u8 rev = 0;
	int ret;

	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE,
				&rev, TWL6040_REG_ASICREV);
	if (ret)
		return ret;

	/*
	 * ERRATA: Reset value of PDM_UL buffer logic is 1 (VDDVIO)
	 * when AUDPWRON = 0, which causes current drain on this pin's
	 * pull-down on OMAP side. The workaround consists of disabling
	 * pull-down resistor of ABE_PDM_UL_DATA pin
	 * Impacted revisions: ES1.1, ES1.2 (both share same ASICREV value),
	 * ES1.3, ES2.0 and ES2.2
	 */
	if ((rev == TWL6040_REV_1_1) ||
	    (rev == TWL6040_REV_1_3) ||
	    (rev == TWL6041_REV_2_0) ||
	    (rev == TWL6041_REV_2_2)) {
		omap_mux_init_signal("abe_pdm_ul_data.abe_pdm_ul_data",
			OMAP_PIN_INPUT);
	}

	return 0;
}

static struct twl4030_codec_data twl6040_codec = {
	.audio          = &twl6040_audio,
	.vibra          = &twl6040_vibra,
	.audpwron_gpio  = 127,
	.naudint_irq    = OMAP44XX_IRQ_SYS_2N,
	.irq_base       = TWL6040_CODEC_IRQ_BASE,
	.init		= twl6040_init,
};
#endif
static struct twl4030_madc_platform_data twl6030_gpadc = {
	.irq_line = -1,
};

static struct twl4030_platform_data twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* TWL6030 regulators at OMAP443X/4460 based SOMs */
	.vmmc		= &vmmc,
	.vpp		= &vpp,
	.vusim		= &vusim,
	.vana		= &vana,
	.vcxio		= &vcxio,
	.vdac		= &vdac,
	.vusb		= &vusb,
	.vaux1		= &vaux1,
	.vaux2		= &vaux2,
	.vaux3		= &vaux3,

	/* TWL6032 regulators at OMAP447X based SOMs */
	.ldo1		= &vpp,
	.ldo2		= &vaux1,
	.ldo3		= &vaux3,
	.ldo4		= &vaux2,
	.ldo5		= &vmmc,
	.ldo6		= &vcxio,
	.ldo7		= &vusim,
	.ldoln		= &vdac,
	.ldousb		= &vusb,

	/* TWL6030/6032 common resources */
	.clk32kg	= &clk32kg,
	.clk32kaudio	= &clk32kaudio,

	/* SMPS */
	.vdd1		= &vcore1,
	.vdd2		= &vcore2,
	.v2v1		= &v2v1,

	/* children */
	.bci		= &bci_data,
	.usb		= &omap4_usbphy_data,
#ifdef CONFIG_TWL6040_CODEC
	.codec		= &twl6040_codec,
#endif
	.madc		= &twl6030_gpadc,

	/* External control pins */
	.sysen		= &sysen,
	.regen1		= &regen1,
};

void __init omap4_power_init(void)
{
	/*
	 * VCORE3 & VMEM are not used in 4460. By register it to regulator
	 * framework will ensures that resources are disabled.
	 */
	if (cpu_is_omap446x()) {
		twldata.vdd3 = &vcore3;
		twldata.vmem = &vmem;
	}

	omap4_pmic_init("twl6030", &twldata);
}

