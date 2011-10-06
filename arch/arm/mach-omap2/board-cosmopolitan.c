/*
 * Board support file for LGE Cosmopolitan Board.
 *
 * Copyright (C) 2010 LG Electronic Inc.
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/input/sfh7741.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/leds-omap4430sdp-display.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/twl6040-vib.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/vibrator.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/timer-gp.h>
#include <plat/display.h>
#include <plat/usb.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif
#include <plat/mmc.h>
#include <plat/mcspi.h>
#include <plat/omap4-keypad.h>
#include <plat/hwspinlock.h>
#include <plat/nokia-dsi-panel.h>
#include <plat/keypad.h>
#include <plat/opp_twl_tps.h>

#include <linux/synaptics_i2c_rmi.h>
#ifdef CONFIG_SPI_IFX
#include <linux/spi/ifx_n721_spi.h>
#endif /* CONFIG_SPI_IFX */
#include <linux/cosmo/fuel_gauge_max17043.h>

#include "mux.h"

#include "hsmmc.h"
#include "smartreflex-class3.h"
#include "board-4430sdp-wifi.h"	/* JamesLee :: FEB17 */
#include "board-connectivity.h"

#define OMAP4_KBDOCP_BASE               0x4A31C000
#define ETH_KS8851_IRQ			34
#define ETH_KS8851_POWER_ON		48
#define ETH_KS8851_QUART		138
#define OMAP4SDP_MDM_PWR_EN_GPIO	157

#define LED_SEC_DISP_GPIO 27
#define DSI2_GPIO_59	59

#define LED_PWM2ON		0x03
#define LED_PWM2OFF		0x04
#define LED_TOGGLE3		0x92

#ifdef CONFIG_MACH_LGE_COSMO_DOMASTIC
#define LGE_FW_TDMB
#endif 

#ifdef LGE_FW_TDMB
#define COSMO_TDMB_IRQ_GPIO 		44
#endif

#ifdef CONFIG_SND_OMAP_SOC_HDMI
static struct platform_device cosmopolitan_hdmi_audio_device = {
	.name		= "hdmi-dai",
	.id		= -1,
};
#endif

static int omap_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_VOLUMEDOWN),
	
	KEY(1, 0, KEY_HOME),
	KEY(1, 1, KEY_3D),
#ifdef CONFIG_MACH_LGE_COSMO_DOMASTIC
	KEY(2, 1, KEY_TESTMODE_UNLOCK),	//  Pattern_Unlock_ATCommand 
#endif
	0,
};


static struct resource sdp4430_kp_resources[] = {
	{
		.start  = OMAP4_KBDOCP_BASE,
		.end    = OMAP4_KBDOCP_BASE,
		.flags  = IORESOURCE_MEM,
	},
};

static struct omap_kp_platform_data omap_kp_data = {
	.rows		= 4,
	.cols		= 4,
	.keymap		= omap_keymap,
	.keymapsize	= ARRAY_SIZE(omap_keymap),
	.delay		= 4,
	.rep		= 0,
};

static struct platform_device omap_kp_device = {
	.name		= "omap-keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &omap_kp_data,
	},
	.num_resources	= ARRAY_SIZE(sdp4430_kp_resources),
	.resource	= sdp4430_kp_resources,
};

#ifdef CONFIG_SPI_IFX
static void ifx_n721_dev_init(void)
{
	printk("[e]Board-4430: IFX_n721_DEV_INIT\n");

	if(gpio_request(IFX_SRDY_GPIO, "ifx srdy") < 0) {
		printk(KERN_ERR "Can't get SRDY GPIO\n");
		return;
	}
	if(gpio_request(IFX_MRDY_GPIO, "ifx mrdy") < 0) {
		printk(KERN_ERR "Can't get MRDY GPIO\n");
		return;
	}
	if (gpio_request(MODEM_GPIO_PWRON, "ifx pwron") < 0) {
		printk(KERN_ERR "Can't get MODEM_PWRON GPIO\n");
		return;
	}

	gpio_direction_input(IFX_SRDY_GPIO);  //gpio_119
	gpio_direction_output(IFX_MRDY_GPIO, 0);
}

static struct omap2_mcspi_device_config ifxn721_mcspi_config = {
	.turbo_mode = 0,
	.single_channel = 1,	/* 0: slave, 1: master */
};
#endif /* CONFIG_SPI_IFX */

static struct spi_board_info sdp4430_spi_board_info[] __initdata = {
	{
		.modalias               = "ks8851",

#ifdef LGE_FW_TDMB
		.bus_num                = 2,
#else
		.bus_num                = 1,
#endif

		.chip_select            = 0,
		.max_speed_hz           = 24000000,
		.irq                    = ETH_KS8851_IRQ,
	},
#ifdef CONFIG_SPI_IFX
	{
		.modalias		= "ifxn721",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 24000000,
		.controller_data	= &ifxn721_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(IFX_SRDY_GPIO),
	},
#endif
};


#ifdef CONFIG_LGE_MTC_ETA
struct platform_device cosmo_mtc_eta_log_device = {
	.name = "lge_mtc_eta_logger",
};
#endif



static struct pwm_vib_platform_data vib_data = {
        .max_timeout            =       15000,
        .active_low                     =       0,
        .initial_vibrate        =       0,
};

static struct platform_device vib = {
	.name   =	VIB_PWM_NAME,
	.id		=	-1,
	.dev    =	{
		.platform_data  = &vib_data,
	},
};

#if defined(CONFIG_TOUCHSCREEN_HUB_SYNAPTICS) || defined(CONFIG_TOUCHSCREEN_COSMO_SYNAPTICS) || defined(CONFIG_TOUCHSCREEN_COSMO_TM1709)
#define HUB_TS_I2C_INT_GPIO   52

static struct synaptics_i2c_rmi_platform_data heaven_ts_synaptics_platform_data[] = {
        {
		.version	= 0x0,
        .irqflags	= IRQF_TRIGGER_FALLING,
       }
};
#endif

static struct nokia_dsi_panel_data dsi_panel = {
		.name	= "hub_panel",
		.reset_gpio	= 30,
		.use_esd_check	= false,
		.set_backlight	= NULL,
};

static struct omap_dss_device sdp4430_lcd_device = {
	.name			= "lcd",
	.driver_name		= "taal",
	.type			= OMAP_DISPLAY_TYPE_DSI,
	.data			= &dsi_panel,
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 2,
		.data1_pol	= 0,
		.data2_lane	= 3,
		.data2_pol	= 0,
		.div		= {
			.lck_div	= 1,
			.pck_div	= 5,
			.regm		= 150,
			.regn		= 17,
			.lp_clk_div	= 8,
		},
	},
	.channel 		= OMAP_DSS_CHANNEL_LCD,
};

/* Display */

#define GPIO_144	144
#define GPIO_190	190
#define GPIO_27		27

static int sdp4430_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2) {

                gpio_request(GPIO_190, "3d_lcd_en");
                gpio_direction_output(GPIO_144, 0);
                gpio_request(GPIO_144, "dsi2_lcd_en");
                gpio_request(GPIO_27, "3d_bank_sel");
                gpio_direction_output(GPIO_190, 1);
		mdelay(10);

                twl_i2c_write_u8(TWL_MODULE_PWM, 0xFF, PWM2ON); /*0xBD = 0xFF*/
                twl_i2c_write_u8(TWL_MODULE_PWM, 0x7F, PWM2OFF); /*0xBE = 0x7F*/
                twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x30, TOGGLE3);
	} else {

	}

	return 0;
}

static int sdp4430_panel_disable_lcd(struct omap_dss_device *dssdev)
{

	if (dssdev->channel == OMAP_DSS_CHANNEL_LCD2) {

		gpio_set_value(GPIO_190, 0);
		gpio_set_value(GPIO_144, 0);
		

	} else {
	}
	return 0;
}

#if defined(CONFIG_PANEL_HUB) || defined(CONFIG_PANEL_COSMO)
#define DSI_CLOCK_POLARITY  0   /* +/- pin order */
#define DSI_DATA0_POLARITY  0   /* +/- pin order */
#define DSI_DATA1_POLARITY  0   /* +/- pin order */
#define DSI_CLOCK_LANE      3   /* Clock lane position: 1 */
#define DSI_DATA0_LANE      1   /* Data0 lane position: 2 */
#define DSI_DATA1_LANE      2   /* Data1 lane position: 3 */
#endif

static struct omap_dss_device sdp4430_lcd2_device = {
	.name			= "2lcd",
	.driver_name		="cosmo_panel",
	.type = OMAP_DISPLAY_TYPE_DSI,
	.phy.dpi.data_lines = 24,	 /* if use bpp32, set as 24 */

	.panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_ONOFF | OMAP_DSS_LCD_RF,
	.panel.timings.x_res = 480,
	.panel.timings.y_res = 800,

	.panel.timings.vfp = 0,
	.panel.timings.vsw = 8,	
	.panel.timings.vbp = 22,
	.panel.timings.hfp = 14,
	.panel.timings.hsw = 51,	 
	.panel.timings.hbp = 19,

	.phy.dsi.clk_lane = 1,
	.phy.dsi.data1_lane = 3,
	.phy.dsi.data2_lane = 2,
	.phy.dsi.num_data_lanes = 2,
	.phy.dsi.clk_pol = DSI_CLOCK_POLARITY,
	.phy.dsi.data1_pol = DSI_DATA0_POLARITY,
	.phy.dsi.data2_pol = DSI_DATA1_POLARITY,

	.phy.dsi.div.regn = 20,
	.phy.dsi.div.regm = 177,  
	.phy.dsi.div.regm_dispc = 5,
	.phy.dsi.div.regm_dsi = 7,

	.phy.dsi.div.lck_div = 1,
	.phy.dsi.div.pck_div = 5,

	.ctrl.pixel_size = 24,

	.phy.dsi.div.lp_clk_div = 12,
	.phy.dsi.mode = OMAP_DSI_MODE_VIDEO,

	.phy.dsi.timings.vfp = 1,
	.phy.dsi.timings.vsa = 8,
	.phy.dsi.timings.vbp = 22,		 
	.phy.dsi.timings.hfp = 20,
	.phy.dsi.timings.hsa = 58,
	.phy.dsi.timings.hbp = 101,
	.platform_enable	=	sdp4430_panel_enable_lcd,
	.platform_disable	=	sdp4430_panel_disable_lcd,
	.channel			=	OMAP_DSS_CHANNEL_LCD2,
};

static int sdp4430_panel_enable_hdmi(struct omap_dss_device *dssdev);
static void sdp4430_panel_disable_hdmi(struct omap_dss_device *dssdev);

#ifdef CONFIG_OMAP2_DSS_HDMI
static int sdp4430_panel_enable_hdmi(struct omap_dss_device *dssdev)
{

	gpio_set_value(HDMI_GPIO_60, 1);
	gpio_set_value(HDMI_GPIO_41, 1);
	return 0;
}

static void sdp4430_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
	gpio_set_value(HDMI_GPIO_60, 1); 
	gpio_set_value(HDMI_GPIO_41, 0);
}

static __attribute__ ((unused)) void __init sdp4430_hdmi_init(void)
{
	return;
}
#endif /* CONFIG_OMAP2_DSS_HDMI */

static struct omap_dss_device sdp4430_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.phy.dpi.data_lines = 24,
	.platform_enable = sdp4430_panel_enable_hdmi,
	.platform_disable = sdp4430_panel_disable_hdmi,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

static struct omap_dss_device *sdp4430_dss_devices[] = {
	&sdp4430_lcd2_device,
	&sdp4430_hdmi_device,
};

static struct omap_dss_board_info sdp4430_dss_data = {
	.num_devices	=	ARRAY_SIZE(sdp4430_dss_devices),
	.devices		=	sdp4430_dss_devices,
	.default_device	=	&sdp4430_lcd2_device,
};

/* wl128x BT, FM, GPS connectivity chip */
static int gpios[] = {166, -1, -1};
static struct platform_device wl128x_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &gpios,
};


static struct platform_device cosmo_gps_gpio =
{
    .name = "cosmo_gps_gpio",
    .id   = -1,
};


static struct platform_device omap_kp_leds_device = {
	.name	=	"keypad_led",
	.id		=	-1,
};

static struct platform_device cosmo_charger_device= {
	.name	=	"cosmo_charger",
	.id		=	-1,
};

#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE_GPIO)

static struct max17043_platform_data max17043_pdata = {
	.slave_addr = FUEL_GAUGE_MAX17043_SLAVE_ID,
	.gpio_alert	= FUEL_GAUGE_MAX17043_GPIO_ALERT,	/* active low */
	.gpio_scl	= FUEL_GAUGE_MAX17043_GPIO_SCL,
	.gpio_sda	= FUEL_GAUGE_MAX17043_GPIO_SDA,
	.udelay		= FUEL_GAUGE_MAX17043_UDELAY,
	.timeout	= FUEL_GAUGE_MAX17043_TIMEOUT,
};


static struct platform_device cosmo_fuel_gauge_device= {
	.name		= "max17043_gpio",
	.id		= -1,
	.dev.platform_data = &max17043_pdata,
};
#endif
static struct platform_device *sdp4430_devices[] __initdata = {

#ifdef CONFIG_SND_OMAP_SOC_HDMI
	&cosmopolitan_hdmi_audio_device,
#endif

    &cosmo_gps_gpio,

	&vib,
	&omap_kp_leds_device,
	&omap_kp_device,

#ifdef CONFIG_LGE_MTC_ETA
      &cosmo_mtc_eta_log_device,
#endif

	&cosmo_charger_device,
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE_GPIO)
	&cosmo_fuel_gauge_device,
#endif
};

static void __init lge_cosmopolitan_init_irq(void)
{
	omap2_init_common_hw(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
	gic_init_irq();
	sr_class3_init();
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode			= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 100,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		=	2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
		.gpio_cd	=	-EINVAL,
		.gpio_wp	=	-EINVAL,
		.ocr_mask	=	MMC_VDD_165_195,
		.nonremovable	=	true,
#if defined(CONFIG_PM_RUNTIME)
		.power_saving	=	true,
#endif
	},
	{
		.mmc		=	1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
//		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_1_8V_DDR,
		.gpio_cd	=	40,
		.gpio_wp	=	-EINVAL,
#ifdef CONFIG_MACH_LGE_MMC_COVER
		.sd_cover   =   42,		
#endif		
		.nonremovable	=	false,
		.ocr_mask	=	MMC_VDD_32_33,
#if defined(CONFIG_MACH_LGE_VMMC_ALWAYSON_FORCED)||defined(CONFIG_MACH_LGE_MMC_ALWAYSON)	
		.power_saving	=	false,

#else
  #if defined(CONFIG_PM_RUNTIME) 
		.power_saving	=	true,
  #endif
#endif	
	},
	{
		.mmc		=	5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	=	-EINVAL,
		.gpio_wp	=	-EINVAL,
		.ocr_mask	=	MMC_VDD_165_195,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply sdp4430_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "mmci-omap-hs.0",
	},
};

static struct regulator_consumer_supply sdp4430_cam2_supply[] = {
	{
		.supply = "cam2pwr",
	},
};
static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
#ifndef CONFIG_MACH_LGE_MMC_COVER
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE + MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;

#else
  #if defined(CONFIG_MACH_LGE_MMC_ENHANCED_COVER)
		pdata->slots[0].card_detect_irq_by_data3pin = TWL6030_IRQ_BASE + MMCDETECT_INTR_OFFSET;		
  #endif

#endif		
	}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev)
		return;

	pdata = dev->platform_data;
	pdata->init = omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static struct regulator_init_data sdp4430_vaux1 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vaux2 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vaux3 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = sdp4430_cam2_supply,
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data sdp4430_vmmc = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = sdp4430_vmmc_supply,
};

static struct regulator_init_data sdp4430_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vusim = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct twl4030_madc_platform_data sdp4430_gpadc_data = {
	.irq_line	= 1,
};

static int sdp4430_batt_table[] = {
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

static struct twl4030_bci_platform_data sdp4430_bci_data = {
	.monitoring_interval		= 10,
	.max_charger_currentmA		= 1500,
	.max_charger_voltagemV		= 4560,
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= sdp4430_batt_table,
	.tblsize			= ARRAY_SIZE(sdp4430_batt_table),
};

#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
#define EAR_SENSE_GPIO	21
#endif


static struct twl4030_codec_audio_data twl6040_audio = {
	.audio_mclk	= 38400000,
	.audpwron_gpio  = 127,
	.naudint_irq    = OMAP44XX_IRQ_SYS_2N,

#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
	.hsjack_gpio	= EAR_SENSE_GPIO,
	.hsjack_irq		= OMAP_GPIO_IRQ(EAR_SENSE_GPIO),
#endif

};

static struct twl4030_codec_vibra_data twl6040_vibra = {
	.audio_mclk	= 38400000,
};

static struct twl4030_codec_data twl6040_codec = {
	.audio_mclk	= 38400000,
	.audio = &twl6040_audio,
	.vibra = &twl6040_vibra,
};

static struct twl4030_platform_data sdp4430_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &sdp4430_vmmc,
	.vpp		= &sdp4430_vpp,
	.vusim		= &sdp4430_vusim,
	.vana		= &sdp4430_vana,
	.vcxio		= &sdp4430_vcxio,
	.vdac		= &sdp4430_vdac,
	.vusb		= &sdp4430_vusb,
	.vaux1		= &sdp4430_vaux1,
	.vaux2		= &sdp4430_vaux2,
	.vaux3		= &sdp4430_vaux3,
	.madc           = &sdp4430_gpadc_data,
	.bci            = &sdp4430_bci_data,

	/* children */
	.codec          = &twl6040_codec,
};

#if defined(CONFIG_SUBPMIC_LP8720)
#include <mach/lp8720.h>
static struct lp8720_platform_data lp8720_pdata = {
	.en_gpio_num	=	99,
};
#endif

#if defined(CONFIG_SUBPMIC_RT8053)
#include <mach/rt8053.h>
static struct rt8053_platform_data rt8053_pdata = {
	.en_gpio_num	=	99,
};
#endif

#if defined(CONFIG_BACKLIGHT_LM3528)
#include <mach/lm3528.h>
static struct lm3528_platform_data	lm3528_pdata = {
	.gpio_hwen	=	98,
};
#endif

#if defined(CONFIG_COSMO_CAMERAFLASH_LM3559)
#include <mach/lm3559.h>
static struct lm3559_platform_data	lm3559_pdata = {
	.gpio_hwen	=	191,
};
#endif

#if defined(CONFIG_SENSORS_MPU3050) || defined(CONFIG_SENSORS_MPU3050_MODULE)
#include <linux/mpu.h>

static struct mpu3050_platform_data mpu3050_data = {
    .int_config  = 0x10,
    .orientation = {  1,  0,  0, 
                      0,  1,  0, 
                      0,  0,  1 },
    .level_shifter = 0,
    .accel = {
#if !defined(CONFIG_SENSORS_MPU3050_MODULE)
        .get_slave_descr = kxtf9_get_slave_descr,
#endif
        .adapt_num   = 4,
        .bus         = EXT_SLAVE_BUS_SECONDARY,
        .address     = 0x0F,
        .orientation = {  1,  0,  0, 
                          0,  1,  0, 
                          0,  0,  1 },
    },
    .compass = {
#if !defined(CONFIG_SENSORS_MPU3050_MODULE)
        .get_slave_descr = ami30x_get_slave_descr,
#endif
#if defined(CONFIG_MACH_LGE_COSMO_EVB_C) || defined(CONFIG_MACH_LGE_COSMO_REV_A) || defined(CONFIG_MACH_LGE_COSMO_REV_B) || defined(CONFIG_MACH_LGE_COSMO_REV_C)
		.adapt_num	= 2,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
        .address     = 0x0E,
        .orientation = {  1,  0,  0,
                          0, -1,  0,
               	          0,  0, -1 },
#else 
		.adapt_num	= 4,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
        .address     = 0x0E,
        .orientation = {  0,  1,  0,
               	          1,  0,  0,
                       	  0,  0, -1 },
#endif
    },
};
#endif

static struct i2c_board_info __initdata sdp4430_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &sdp4430_twldata,
	},
	{
		I2C_BOARD_INFO("cdc_tcxo_driver", 0x6c),
	},
};

static struct i2c_board_info __initdata sdp4430_i2c_2_boardinfo[] = {
#if defined(CONFIG_TOUCHSCREEN_HUB_SYNAPTICS) || defined(CONFIG_TOUCHSCREEN_COSMO_SYNAPTICS) || defined(CONFIG_TOUCHSCREEN_COSMO_TM1709)
	{	
		I2C_BOARD_INFO("heaven_synaptics_ts", 0x20),
		//.platform_data = &heaven_ts_synaptics_i2c_bdinfo,
	       .platform_data = &heaven_ts_synaptics_platform_data,
	       .irq = HUB_TS_I2C_INT_GPIO,
	},
#endif 
#if defined(CONFIG_MACH_LGE_COSMO_EVB_B) && defined(CONFIG_SUBPMIC_LP8720)
	{
		I2C_BOARD_INFO(LP8720_I2C_NAME,  LP8720_I2C_ADDR),
		.platform_data =&lp8720_pdata,
	},
#endif
#if defined(CONFIG_BACKLIGHT_LM3528)
	{
		I2C_BOARD_INFO(LM3528_I2C_NAME,  LM3528_I2C_ADDR),
		.platform_data	=	&lm3528_pdata,
	},
#endif
	{
		I2C_BOARD_INFO("lgdp4512_barrierA", 0x74),		
	},
	{
		I2C_BOARD_INFO("lgdp4512_barrierB", 0x75),
	},
#if defined(CONFIG_COSMO_CAMERAFLASH_LM3559)
	{
		I2C_BOARD_INFO(LM3559_I2C_NAME,	LM3559_I2C_ADDR),
		.platform_data	=	&lm3559_pdata,
	},
#endif	
};

static struct i2c_board_info __initdata sdp4430_i2c_3_boardinfo[] = {
#if !defined( CONFIG_MACH_LGE_COSMO_EVB_B) \
	&& !defined(CONFIG_MACH_LGE_COSMO_EVB_C)
	{
		I2C_BOARD_INFO("max14526", 0x44),
	},
#endif // CONFIG_MACH_LGE_COSMO_EVB_B	
#if defined(CONFIG_MACH_LGE_COSMO_EVB_C) || defined(CONFIG_MACH_LGE_COSMO_REV_A) || defined(CONFIG_MACH_LGE_COSMO_REV_B)
#if defined(CONFIG_SUBPMIC_LP8720)
	{
		I2C_BOARD_INFO(LP8720_I2C_NAME,  LP8720_I2C_ADDR),
		.platform_data =&lp8720_pdata,	
	},
#endif
#endif
#if defined(CONFIG_SUBPMIC_RT8053)
	{
		I2C_BOARD_INFO(RT8053_I2C_NAME, RT8053_I2C_ADDR),
		.platform_data	=	&rt8053_pdata,
	},
#endif
#if !defined(CONFIG_MACH_LGE_COSMO_EVB_C) && !defined(CONFIG_MACH_LGE_COSMO_REV_A) && !defined(CONFIG_MACH_LGE_COSMO_REV_B) && !defined(CONFIG_MACH_LGE_COSMO_REV_C)
#if defined( CONFIG_SENSORS_APDS9900 )
	{
		I2C_BOARD_INFO("apds9900", 0x39),
	},
#endif
#endif
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE_I2C)
	{
		I2C_BOARD_INFO("max17043_i2c", 0x36),
	},
#endif
};

static struct i2c_board_info __initdata sdp4430_i2c_4_boardinfo[] = {
#if defined( CONFIG_MACH_LGE_COSMO_EVB_B) || defined(CONFIG_MACH_LGE_COSMO_EVB_C)
	{
		I2C_BOARD_INFO("max14526", 0x44),
	},
#endif
#if defined(CONFIG_MACH_LGE_COSMO_EVB_C) || defined(CONFIG_MACH_LGE_COSMO_REV_A) || defined(CONFIG_MACH_LGE_COSMO_REV_B) || defined(CONFIG_MACH_LGE_COSMO_REV_C)
#if defined( CONFIG_SENSORS_APDS9900 )
	{
		I2C_BOARD_INFO("apds9900", 0x39),
	},
#endif
#endif

#if defined(CONFIG_SENSORS_MPU3050) || defined(CONFIG_SENSORS_MPU3050_MODULE)
	{
		I2C_BOARD_INFO("mpu3050",0x68),
		//.irq = OMAP_GPIO_IRQ(27),
		.platform_data = &mpu3050_data,
	},
#endif
};

static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_4_bus_pdata;

static __initdata struct emif_device_details emif_devices = {
	.cs0_device = &elpida_2G_S2,
	.cs1_device = NULL
};

static int __init omap_i2c_hwspinlock_init(int bus_id, unsigned int
			spinlock_id, struct omap_i2c_bus_board_data *pdata)
{
	pdata->handle = hwspinlock_request_specific(spinlock_id);
	if (pdata->handle != NULL) {
		pdata->hwspinlock_lock = hwspinlock_lock;
		pdata->hwspinlock_unlock = hwspinlock_unlock;
		return 0;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d, ignore register i2c bus\n", bus_id);
		return 1;
	}
}

static int __init omap4_i2c_init(void)
{
	int ret;
	
	/*
	 * Phoenix Audio IC needs I2C1 to
	 * start with 400 KHz or less
	 */
	ret = omap_i2c_hwspinlock_init(1, 0, &sdp4430_i2c_bus_pdata);
	if (!ret)
	omap_register_i2c_bus(1, 400, &sdp4430_i2c_bus_pdata,
		sdp4430_i2c_boardinfo, ARRAY_SIZE(sdp4430_i2c_boardinfo));
	
	ret = omap_i2c_hwspinlock_init(2, 1, &sdp4430_i2c_2_bus_pdata);
	if (!ret)
	omap_register_i2c_bus(2, 400, &sdp4430_i2c_2_bus_pdata,
		sdp4430_i2c_2_boardinfo, ARRAY_SIZE(sdp4430_i2c_2_boardinfo));
	
	ret = omap_i2c_hwspinlock_init(3, 2, &sdp4430_i2c_3_bus_pdata);
	if (!ret)
	omap_register_i2c_bus(3, 400, &sdp4430_i2c_3_bus_pdata,
		sdp4430_i2c_3_boardinfo, ARRAY_SIZE(sdp4430_i2c_3_boardinfo));
	
	ret = omap_i2c_hwspinlock_init(4, 3, &sdp4430_i2c_4_bus_pdata);
	if (!ret)
	omap_register_i2c_bus(4, 400, &sdp4430_i2c_4_bus_pdata,
		sdp4430_i2c_4_boardinfo, ARRAY_SIZE(sdp4430_i2c_4_boardinfo));
	return 0;
}


#ifdef CONFIG_TIWLAN_SDIO
static void pad_config(unsigned long pad_addr, u32 andmask, u32 ormask)
{
	int val;
	 u32 *addr;

	addr = (u32 *) ioremap(pad_addr, 4);
	if (!addr) {
		printk(KERN_ERR"OMAP_pad_config: ioremap failed with addr %lx\n",
		pad_addr);
	return;
	}

	val =  __raw_readl(addr);
	val &= andmask;
	val |= ormask;
	__raw_writel(val, addr);

	iounmap(addr);
}

void wlan_1283_config(void)
{
	return ;
}
#endif

#ifdef LGE_FW_TDMB
static struct omap2_mcspi_device_config fc8050_mcspi_config = 
{
	.turbo_mode = 0,
	.single_channel = 1,	/* 0: slave, 1: master */
};

static struct spi_board_info hub_tdmb_spi_board_info[] __initdata = 
{
	[0] = {
	       .modalias 		= "tdmb_fc8050",
	       .bus_num 		= 1, /* MCSPI NUM */
	       .chip_select 		= 0,
	       .max_speed_hz 	= 24000*1000,
	       .controller_data 	= &fc8050_mcspi_config,
	       .irq = OMAP_GPIO_IRQ(COSMO_TDMB_IRQ_GPIO),
	       },
};

static void __init cosmo_tdmb_spi_init(void)
{
	spi_register_board_info(hub_tdmb_spi_board_info, ARRAY_SIZE(hub_tdmb_spi_board_info));
}
#endif

static void __init omap4_display_init(void)
{
	void __iomem *phymux_base = NULL;
	unsigned int dsimux = 0xFFFFFFFF;
	phymux_base = ioremap(0x4A100000, 0x1000);
	/* Turning on DSI PHY Mux*/
	__raw_writel(dsimux, phymux_base+0x618);
	dsimux = __raw_readl(phymux_base+0x618);

	gpio_request(HDMI_GPIO_60 , "hdmi_gpio_60");
        gpio_request(HDMI_GPIO_41 , "hdmi_gpio_41");
        gpio_direction_output(HDMI_GPIO_60, 1);
        gpio_direction_output(HDMI_GPIO_41, 0);
}



/* OMAP4: HSI: PM: Enable Modem CAWAKE to issue wakeup events */

/*
 * As OMAP4430 mux HSI and USB signals, when HSI is used (for instance HSI
 * modem is plugged) we should configure HSI pad conf and disable some USB
 * configurations.
 * HSI usage is declared using bootargs variable:
 * Variable modem_ipc is used to catch bootargs parameter value.
 */
static void omap_4430hsi_pad_conf(void)
{
	/*
	 * HSI pad conf: hsi1_ca/ac_wake/flag/data/ready
	 */

	/* hsi1_cawake */
	omap_mux_init_signal("usbb1_ulpitll_clk.hsi1_cawake", \
		OMAP_PIN_INPUT_PULLDOWN | \
		OMAP_PIN_OFF_NONE | \
		OMAP_PIN_OFF_WAKEUPENABLE);
	/* hsi1_caflag */
	omap_mux_init_signal("usbb1_ulpitll_dir.hsi1_caflag", \
		OMAP_PIN_INPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_cadata */
	omap_mux_init_signal("usbb1_ulpitll_stp.hsi1_cadata", \
		OMAP_PIN_INPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_acready */
	omap_mux_init_signal("usbb1_ulpitll_nxt.hsi1_acready", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_OUTPUT_LOW);
	/* hsi1_acwake */
	omap_mux_init_signal("usbb1_ulpitll_dat0.hsi1_acwake", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_acdata */
	omap_mux_init_signal("usbb1_ulpitll_dat1.hsi1_acdata", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_acflag */
	omap_mux_init_signal("usbb1_ulpitll_dat2.hsi1_acflag", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_caready */
	omap_mux_init_signal("usbb1_ulpitll_dat3.hsi1_caready", \
		OMAP_PIN_INPUT | \
		OMAP_PIN_OFF_NONE);
}


static void enable_board_wakeup_source(void)
{
	/* Android does not have touchscreen as wakeup source */
#if !defined(CONFIG_ANDROID)
	u16 padconf;
	/* NOTE: Use mx framework when available */
	/* Enable IO wakeup for the gpio used for primary touchscreen */
	padconf = omap_readw(CONTROL_CORE_PAD1_GPMC_AD11);
	padconf |= OMAP44XX_PADCONF_WAKEUPENABLE0;
	omap_writew(padconf, CONTROL_CORE_PAD1_GPMC_AD11);
#endif

	/*
	 * Enable IO daisy for sys_nirq1/2, to be able to
	 * wakeup from interrupts from PMIC/Audio IC.
	 * Needed only in Device OFF mode.
	 */
	omap_mux_enable_wakeup("sys_nirq1");
	omap_mux_enable_wakeup("sys_nirq2");


#if defined(CONFIG_OMAP_HSI)	
	/*
	 * Enable IO daisy for HSI CAWAKE line, to be able to
	 * wakeup from interrupts from Modem.
	 * Needed only in Device OFF mode.
	 */
	omap_mux_enable_wakeup("usbb1_ulpitll_clk.hsi1_cawake");
#endif


}

static struct omap_volt_pmic_info omap_pmic_core = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x61,
	.i2c_cmdreg = 0x62,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0xA,
	.vp_vlimitto_vddmax = 0x28,
};

static struct omap_volt_pmic_info omap_pmic_mpu = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x55,
	.i2c_cmdreg = 0x56,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0xA,
	.vp_vlimitto_vddmax = 0x39,
};

static struct omap_volt_pmic_info omap_pmic_iva = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x5b,
	.i2c_cmdreg = 0x5c,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0xA,
	.vp_vlimitto_vddmax = 0x2D,
};

static struct omap_volt_vc_data vc_config = {
	.vdd0_on = 1350000,        /* 1.35v */
	.vdd0_onlp = 1350000,      /* 1.35v */
	.vdd0_ret = 837500,       /* 0.8375v */
	.vdd0_off = 0,		/* 0 v */
	.vdd1_on = 1100000,        /* 1.1v */
	.vdd1_onlp = 1100000,      /* 1.1v */
	.vdd1_ret = 837500,       /* 0.8375v */
	.vdd1_off = 0,		/* 0 v */
	.vdd2_on = 1100000,        /* 1.1v */
	.vdd2_onlp = 1100000,      /* 1.1v */
	.vdd2_ret = 837500,       /* .8375v */
	.vdd2_off = 0,		/* 0 v */
};
#define CONFIG_SERIAL_OMAP_UART2_DMA 1

static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
#if defined(CONFIG_SERIAL_OMAP_UART1_DMA)
		.use_dma	= CONFIG_SERIAL_OMAP_UART1_DMA,
		.dma_rx_buf_size = CONFIG_SERIAL_OMAP_UART1_RXDMA_BUFSIZE,
		.dma_rx_timeout	= CONFIG_SERIAL_OMAP_UART1_RXDMA_TIMEOUT,
#else
		.use_dma	= 0,
		.dma_rx_buf_size = 0,
		.dma_rx_timeout	= 0,
#endif /* CONFIG_SERIAL_OMAP_UART1_DMA */
		.idle_timeout	= CONFIG_SERIAL_OMAP_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
#if defined(CONFIG_SERIAL_OMAP_UART2_DMA)
		.use_dma	= CONFIG_SERIAL_OMAP_UART2_DMA,
		.dma_rx_buf_size = 4096,
		.dma_rx_timeout	= 1,
#else
		.use_dma	= 0,
		.dma_rx_buf_size = 0,
		.dma_rx_timeout	= 0,
#endif /* CONFIG_SERIAL_OMAP_UART2_DMA */
		.idle_timeout	= CONFIG_SERIAL_OMAP_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
#if defined(CONFIG_SERIAL_OMAP_UART3_DMA)
		.use_dma	= CONFIG_SERIAL_OMAP_UART3_DMA,
		.dma_rx_buf_size = CONFIG_SERIAL_OMAP_UART3_RXDMA_BUFSIZE,
		.dma_rx_timeout	= CONFIG_SERIAL_OMAP_UART3_RXDMA_TIMEOUT,
#else
		.use_dma	= 0,
		.dma_rx_buf_size = 0,
		.dma_rx_timeout	= 0,
#endif /* CONFIG_SERIAL_OMAP_UART3_DMA */
		.idle_timeout	= CONFIG_SERIAL_OMAP_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
#if defined(CONFIG_SERIAL_OMAP_UART4_DMA)
		.use_dma	= CONFIG_SERIAL_OMAP_UART4_DMA,
		.dma_rx_buf_size = CONFIG_SERIAL_OMAP_UART4_RXDMA_BUFSIZE,
		.dma_rx_timeout	= CONFIG_SERIAL_OMAP_UART4_RXDMA_TIMEOUT,
#else
		.use_dma	= 0,
		.dma_rx_buf_size = 0,
		.dma_rx_timeout	= 0,
#endif /* CONFIG_SERIAL_OMAP_UART3_DMA */
		.idle_timeout	= CONFIG_SERIAL_OMAP_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.flags		= 0
	}
};
#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif
static void __init lge_cosmopolitan_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, package);
	omap_emif_setup_device_details(&emif_devices, &emif_devices);
	omap_init_emif_timings();

	omap4_i2c_init();
	omap4_display_init();
	conn_board_init(); /* Added for FlexST */
	omap_display_init(&sdp4430_dss_data);
	platform_add_devices(sdp4430_devices, ARRAY_SIZE(sdp4430_devices));
	conn_add_plat_device(); /* Added for FlexST */
	omap_serial_init(omap_serial_platform_data);
	omap4_twl6030_hsmmc_init(mmc);

#ifdef CONFIG_TIWLAN_SDIO
	config_wlan_mux();
#endif

/* OMAP4: HSI: PM: Enable Modem CAWAKE to issue wakeup events */
#if defined(CONFIG_OMAP_HSI)
	pr_info("Modem MIPI-HSI detected");
	omap_4430hsi_pad_conf();
#endif
	usb_musb_init(&musb_board_data);

	spi_register_board_info(sdp4430_spi_board_info, ARRAY_SIZE(sdp4430_spi_board_info));
	
#ifdef LGE_FW_TDMB
	cosmo_tdmb_spi_init();
#endif

#ifdef CONFIG_SPI_IFX
	ifx_n721_dev_init();
#endif /* CONFIG_SPI_IFX */

	/* Added for FlexST */
	conn_config_gpios();	
	enable_board_wakeup_source();
	omap_voltage_register_pmic(&omap_pmic_core, "core");
	omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
	omap_voltage_register_pmic(&omap_pmic_iva, "iva");
	omap_voltage_init_vc(&vc_config);
	printk("2army\n");
}

static void __init lge_cosmopolitan_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

MACHINE_START(LGE_COSMOPOLITAN, "OMAP4430 LGE Cosmopolitan board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= lge_cosmopolitan_map_io,
	.init_irq	= lge_cosmopolitan_init_irq,
	.init_machine	= lge_cosmopolitan_init,
	.timer		= &omap_timer,
MACHINE_END
