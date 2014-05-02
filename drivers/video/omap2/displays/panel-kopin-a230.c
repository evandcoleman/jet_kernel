/********************* (C) COPYRIGHT 2012 Recon Instruments ********************
*
* File Name          : panel-kopin-a230.c
* Author             : Gil Zhaiek
* Version            : V 1.0.0
* Date               : 03/30/2012
* Description        : Kopin A230 Low Level Display Driver for OMAP RFBI
*
********************************************************************************
*                  COPYRIGHT (C) 2012 Recon Instruments LLC                    *
*                                                                              *
* This file is subject to the terms and conditions of the GNU General Public   *
* License.  See the file COPYING in the main directory of this archive for     *
* more details.                                                                *
*******************************************************************************/
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/leds-omap4430jet-display.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <video/omapdss.h>

static const u8 boot_image[] = {
#include "boot_image.h"
};
//#define ENABLE_FPS_LOG

#define KOPIN_WIDTH		428
#define KOPIN_HEIGHT	240

#define MCLK_PS			37037

#define A230_DEFAULT_BRIGHTNESS	0x198  // 0x66*4
#define A230_PWM0_H_ADDR		0x1D
#define A230_PWM1_H_ADDR		0x1E
#define A230_PWM0_1_L_ADDR		0x1F

#define A230_PWR_REG			0x12
#define A230_PWR_PAD_DWN_BIT	(1 << 0)
#define A230_PWR_DIG_DWN_BIT	(1 << 1)

#define RFBI_A16_GPIO			163
#define A230_POWER_ENABLE_GPIO 52

#ifdef CONFIG_JET_V2
#define A230_CLK_TV_EN_GPIO		41
#define A230_DISP_RES_GPIO	42
#endif


#define KOPIN_REG_BASE_ADDR		0x1ff00

#define RGB565_RED				0xF800
#define RGB565_GREEN			0x07E0
#define RGB565_BLUE				0x001F

typedef struct
{
	u8 RegAddr;
	u8 RegData;
} REG8_t;

typedef struct __attribute__ ((packed))
{
	u16 pixels[KOPIN_WIDTH];
} s_fb_line;

typedef struct __attribute__ ((packed))
{
	s_fb_line fb_lines[KOPIN_HEIGHT];
} s_fb_frame;

#define BOOTMODE_POWEROFF	0
#define BOOTMODE_PREWARNING	1
#define BOOTMODE_ANDROID	2
static int bootmode = 0;

bool static kopin_init_done = 0;
bool static kopin_first_image = 1;
u16 static kopin_current_pwm = A230_DEFAULT_BRIGHTNESS;

static REG8_t kopin_init_data[] =
{
	{0x00,0xC2},{0x01,0x00},{0x02,0x04},{0x03,0x01},{0x04,0x01},{0x05,0x00},{0x06,0x1E},{0x07,0x0B},{0x08,0x0B},{0x09,0x80},{0x0F,0x00},
#ifdef CONFIG_JET_V2
	{0x10,0x09}, /*Flip Display Software*/
#else
	{0x10,0x01},
#endif
	{0x11,0x04},{0x12,0x00},{0x13,0x00},{0x14,0x80},{0x15,0x00},{0x16,0x00},{0x17,0xFF},{0x18,0x80},{0x19,0x00},{0x1A,0x00},{0x1B,0x80},{0x1C,0x80},
	{A230_PWM0_H_ADDR,A230_DEFAULT_BRIGHTNESS>>4},{A230_PWM1_H_ADDR,A230_DEFAULT_BRIGHTNESS>>4},{A230_PWM0_1_L_ADDR,((A230_DEFAULT_BRIGHTNESS&0xf)<<4)|(A230_DEFAULT_BRIGHTNESS&0xf)},
	{0x20,0x01},{0x22,0x10},{0x23,0xEB},{0x24,0x10},{0x25,0xF0},{0x26,0x2A},{0x27,0x01},{0x28,0x98},{0x29,0x01},{0x2A,0xD0},{0x2B,0x00},{0x2C,0x64},{0x2D,0x00},{0x2E,0x04},{0x2F,0x02},
	{0x30,0x40},{0x31,0x00},{0x32,0x80},{0x33,0x00},{0x34,0x80},{0x36,0xAB},{0x37,0x01},{0x38,0x54},{0x39,0x00},{0x3F,0x88},
	{0x40,0x0B},{0x41,0x2A},{0x42,0x30},{0x44,0xB7},{0x45,0x01},{0x46,0x06},{0x47,0x00},{0x48,0xB5},{0x49,0x01},{0x4A,0x00},{0x4C,0x01},{0x4D,0xF1},{0x4F,0x00},
	{0x50,0xFF},{0x51,0xEF},{0x52,0xDF},{0x53,0xCF},{0x54,0xBF},{0x55,0xAF},{0x56,0x9F},{0x57,0x8F},{0x58,0x7F},{0x59,0x6F},{0x5A,0x5F},{0x5B,0x4F},{0x5C,0x3F},{0x5D,0x2F},{0x5E,0x1F},{0x5F,0x0F},
	{0x60,0x00},{0x62,0x00},{0x63,0x0F},{0x64,0x1F},{0x65,0x2F},{0x66,0x3F},{0x67,0x4F},{0x68,0x5F},{0x69,0x6F},{0x6A,0x7F},{0x6B,0x8F},{0x6C,0x9F},{0x6D,0xAF},{0x6E,0xBF},{0x6F,0xCF},
	{0x70,0xDF},{0x71,0xEF},{0x72,0xFF},
	{0x80,0x8F},{0x81,0x00},{0x82,0x00},{0x83,0x00},{0x84,0xAC},{0x85,0x01},{0x86,0xF0},{0x88,0x0F},{0x89,0x00},{0x8A,0x00},{0x8B,0x00},{0x8C,0x00},{0x8D,0x00},{0x8E,0x00},
	{0x90,0x0F},{0x91,0x00},{0x92,0x00},{0x93,0x00},{0x94,0x00},{0x95,0x00},{0x96,0x00},{0x98,0x0F},{0x99,0x00},{0x9A,0x00},{0x9B,0x00},{0x9C,0x00},{0x9D,0x00},{0x9E,0x00},
	{0xA0,0x00},{0xA1,0x00},{0xA2,0x00},{0xA4,0xFF},{0xA5,0xFE},{0xA6,0x01},{0xA8,0xFF},{0xA9,0xFE},{0xAA,0x01},{0xAC,0xFF},{0xAD,0xFE},{0xAE,0x01},
	{0xB0,0x00},{0xB1,0x00},{0xB2,0x00},{0xB3,0x03},{0x00,0x0},
};

struct panel_kopin_a230_data {
	int	(*platform_enable)(struct omap_dss_device *dssdev);
	void	(*platform_disable)(struct omap_dss_device *dssdev);
	int	panel_reset;
	int	ctrl_pwrdown;
};

struct panel_drv_data {
	struct	mutex lock;
	struct	omap_dss_device *dssdev;
	s_fb_frame fb;
};

static struct omap_video_timings kopin_a230_timings = {
	.x_res = 428,
	.y_res = 240,

	.pixel_clock	= 10000,
};

#define KOPIN_CMD_MODE		omap_rfbi_configure(dssdev, 16, 8)
#define KOPIN_RAW_MODE		omap_rfbi_configure(dssdev, 16, 16)
#define KOPIN_DISABLE_TE	omap_rfbi_enable_te(false, 0);

static bool is_direct_mode = 0;

#ifdef ENABLE_FPS_LOG
static int frame_cnt = 0;
static unsigned long start_of_frame = 0;
static unsigned long time_sec_helper = 0;

static unsigned long get_curr_time()
{
	struct timespec n;
	getnstimeofday(&n);
	return timespec_to_ns(&n);
}
#endif

// Inlines
static inline struct panel_kopin_a230_data	*get_board_data(const struct omap_dss_device *dssdev) { return dssdev->data;}

static void kopin_a230_write_addr(u16 address)
{
	omap_rfbi_write_command(&address, 2);
}

static void kopin_a230_write_reg_8bit(u8 addr, u8 val) {
	u8 tmp_addr = addr;
	u8 tmp_val = val;
	omap_rfbi_write_command(&tmp_addr, 1);
	omap_rfbi_write_data(&tmp_val, 1);
}

static void kopin_a230_write_reg_16bit(u32 addr, u16 val) {
	u16 tmp_addr = addr & 0xffff;
	u16 tmp_val = val;

	if((addr >> 16) & 0x1) gpio_set_value(RFBI_A16_GPIO, 1);
	omap_rfbi_write_command(&tmp_addr, 2);
	if((addr >> 16) & 0x1) gpio_set_value(RFBI_A16_GPIO, 0);

	omap_rfbi_write_data(&tmp_val, 2);
}

static u8 kopin_a230_read_reg_8bit(u8 addr) {
	u8 tmp_addr = addr;
	u8 ret_val = 0;

	omap_rfbi_write_command(&tmp_addr, 1);
	omap_rfbi_read_data(&ret_val, 1);

	return ret_val;
}

static u16 kopin_a230_read_reg_16bit(u32 addr) {
	u16 tmp_addr = addr & 0xffff;
	u16 ret_val = 0;

	if((addr >> 16) & 0x1) gpio_set_value(RFBI_A16_GPIO, 1);
	omap_rfbi_write_command(&tmp_addr, 2);
	if((addr >> 16) & 0x1) gpio_set_value(RFBI_A16_GPIO, 0);

	omap_rfbi_read_data(&ret_val, 2);

	return ret_val;
}

static void kopin_a230_write_reg(u32 addr, const u16 val, bool direct_mode)
{
	if(direct_mode) {
		kopin_a230_write_reg_16bit(addr | KOPIN_REG_BASE_ADDR, val);
	} else {
		kopin_a230_write_reg_8bit(0xB9, 0x0);
		kopin_a230_write_reg_8bit(0xBA, (addr >> 8) & 0xff);
		kopin_a230_write_reg_8bit(0xBB, addr & 0xff);
		kopin_a230_write_reg_8bit(0xBC, val & 0xff);
		kopin_a230_write_reg_8bit(0xBD, (val >> 8) & 0xff);
	}
}

static u16 kopin_a230_read_reg(u8 addr, bool direct_mode)
{
	u16 ret_val = 0;

	if(direct_mode) {
		ret_val = kopin_a230_read_reg_16bit(addr | KOPIN_REG_BASE_ADDR);
	} else {
		kopin_a230_write_reg_8bit(0xB9, 0x0);
		kopin_a230_write_reg_8bit(0xBA, (addr >> 8) & 0xff);
		kopin_a230_write_reg_8bit(0xBB, addr & 0xff);
		ret_val = kopin_a230_read_reg_8bit(0xBC);
		ret_val |= (kopin_a230_read_reg_8bit(0xBD) << 8);
	}

	return ret_val;
}

// Send any update to the kopin panel here
static void kopin_a230_panel_setup_update(int x, int y, int w, int h)
{
//	printk(KERN_INFO "[%s:%u] \n", __FUNCTION__,__LINE__);

	kopin_a230_write_addr(0x00); // Start of Frame
}

static void a230_set_pwm(u16 val)
{
	rfbi_bus_lock();
	kopin_a230_write_reg(A230_PWM0_H_ADDR, val>>4, is_direct_mode);
	kopin_a230_write_reg(A230_PWM1_H_ADDR, val>>4, is_direct_mode);
	kopin_a230_write_reg(A230_PWM0_1_L_ADDR, ((val&0xf)<<4)|(val&0xf), is_direct_mode);
	rfbi_bus_unlock();
}

static void __exit kopin_a230_panel_remove(struct omap_dss_device *dssdev)
{
	struct kopin_data *sd = dev_get_drvdata(&dssdev->dev);

	kfree(sd);
}

static int kopin_a230_power_on(struct omap_dss_device *dssdev)
{
	u32 i;
	int r = 0;
	printk(KERN_INFO "[%s:%u] \n", __FUNCTION__,__LINE__);
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_rfbi_display_enable(dssdev);
	if (r)
		goto err0;

	dssdev->first_vsync = false;

	msleep(50);

	if(!is_direct_mode) {
		KOPIN_CMD_MODE;
		kopin_a230_write_reg(0xB3, 0x3, is_direct_mode);
		kopin_a230_read_reg(0xB3, is_direct_mode);
		is_direct_mode = 1;
		KOPIN_RAW_MODE;
	}

	if(!kopin_init_done) {
		if(!is_direct_mode) {
			KOPIN_CMD_MODE;
		} else {
			KOPIN_RAW_MODE;
		}
		for(i = 0; i < sizeof(kopin_init_data)/sizeof(REG8_t); i++)
		{
			kopin_a230_write_reg(kopin_init_data[i].RegAddr, kopin_init_data[i].RegData, is_direct_mode);
		}
		kopin_init_done = 1;
		if(!is_direct_mode) {
			KOPIN_RAW_MODE;
			is_direct_mode = 1;
		}
		msleep(1000);
	}

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_rfbi_display_disable(dssdev);
err0:
	return r;
}

static void kopin_a230_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	a230_set_pwm(0);

	rfbi_bus_lock();

	kopin_a230_write_reg(A230_PWR_REG, A230_PWR_PAD_DWN_BIT & 0xFF, is_direct_mode);
#ifdef CONFIG_JET_V2
	gpio_direction_output(A230_DISP_RES_GPIO, 0);
	//gpio_set_value(A230_POWER_ENABLE_GPIO, 0);//set high to work around for power surge
#endif
	
	if (dssdev->platform_disable)
	{
		printk(KERN_INFO "[%s:%u] \n", __FUNCTION__,__LINE__);
		dssdev->platform_disable(dssdev);
	}

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(10);

	omapdss_rfbi_display_disable(dssdev);

	rfbi_bus_unlock();
}

static int kopin_a230_panel_enable(struct omap_dss_device *dssdev)
{
	int r;
	printk(KERN_INFO "[%s:%u] \n", __FUNCTION__,__LINE__);

	if(dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		printk(KERN_WARNING "[%s:%u] Already Enabled\n", __FUNCTION__,__LINE__);
	}

	r = kopin_a230_power_on(dssdev);
	if(r) {
		printk(KERN_INFO "[%s:%u] Failed to Power ON\n", __FUNCTION__,__LINE__);
	}
	else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	}

	KOPIN_RAW_MODE;
	KOPIN_DISABLE_TE;

	return r;
}

static void kopin_a230_panel_disable(struct omap_dss_device *dssdev)
{
	kopin_a230_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int kopin_a230_panel_suspend(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO "[%s:%u] \n", __FUNCTION__,__LINE__);

//#ifdef CONFIG_JET_V2
	//gpio_set_value(A230_POWER_ENABLE_GPIO, 0);
	//dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
//#else
	a230_set_pwm(0);

	msleep(200);

	rfbi_bus_lock();

	kopin_a230_write_reg(A230_PWR_REG, (A230_PWR_DIG_DWN_BIT | A230_PWR_PAD_DWN_BIT )& 0xFF, is_direct_mode);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
#ifdef CONFIG_JET_V2
	gpio_direction_output(A230_CLK_TV_EN_GPIO, 0);//turn off clock as described in datasheet Figure 44.
#endif
	rfbi_bus_unlock();
//#endif


	return 0;
}

static int kopin_a230_panel_resume(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO "[%s:%u] \n", __FUNCTION__,__LINE__);

//#ifdef CONFIG_JET_V2
	//gpio_set_value(A230_POWER_ENABLE_GPIO, 1);
	//dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
//#else
	rfbi_bus_lock();
#ifdef CONFIG_JET_V2
	gpio_direction_output(A230_CLK_TV_EN_GPIO, 1);//turn on clock as described in datasheet Figure 44.
#endif
	kopin_a230_write_reg(A230_PWR_REG, 0, is_direct_mode);
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	rfbi_bus_unlock();

	a230_set_pwm(kopin_current_pwm);
//#endif

	if(bootmode < BOOTMODE_ANDROID) {
		a230_set_pwm(0);
	}

	return 0;
}

static void kopin_a230_panel_get_timings(struct omap_dss_device *dssdev, struct omap_video_timings *timings)
{
//	printk(KERN_INFO "[%s:%u] \n", __FUNCTION__,__LINE__);
	*timings = dssdev->panel.timings;
}

static int kopin_a230_panel_check_timings(struct omap_dss_device *dssdev, struct omap_video_timings *timings)
{
	return 0;
}

static void kopin_a230_panel_get_resolution(struct omap_dss_device *dssdev, u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

static void update_done(void *data)
{
#ifdef ENABLE_FPS_LOG
	unsigned long end_of_frame = get_curr_time();

	printk(KERN_INFO "[%s:%u] Frame #%d From=%lu To=%lu = %lu\n",__FUNCTION__,__LINE__,frame_cnt, start_of_frame, end_of_frame, end_of_frame-start_of_frame);

	if(time_sec_helper < (end_of_frame-1000000000)) {// 1 second passed
		time_sec_helper = end_of_frame;
		printk(KERN_INFO "[%s:%u] %dfps\n", __FUNCTION__,__LINE__,frame_cnt);
		frame_cnt = 0;
	}
#endif

	rfbi_bus_unlock();
}

static int kopin_a230_panel_update(struct omap_dss_device *dssdev, u16 x, u16 y, u16 w, u16 h)
{
	struct panel_drv_data *ddata = dev_get_drvdata(&dssdev->dev);

	// We are in midkernel or shutting down
	if(bootmode < BOOTMODE_ANDROID) {
		kopin_first_image = 0;
		return 0;
	}

	if(dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		kopin_a230_panel_resume(dssdev);
	    a230_set_pwm(kopin_current_pwm);
	}

	rfbi_bus_lock();

	omap_rfbi_prepare_update(dssdev, &x, &y, &w, &h);

	kopin_a230_panel_setup_update(x, y, w, h);

#ifdef ENABLE_FPS_LOG
	frame_cnt++;
	start_of_frame = get_curr_time();
#endif

	if(kopin_first_image) {
		omap_rfbi_write_image(boot_image, w, h);
		kopin_first_image = 0;
		rfbi_bus_unlock();
	} else {
		omap_rfbi_update(dssdev, x, y, w, h, update_done, &ddata->fb);
	}

	return 0;
}

static int kopin_a230_panel_sync(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "sync\n");

	mutex_lock(&ddata->lock);
	rfbi_bus_lock();
	rfbi_bus_unlock();
	mutex_unlock(&ddata->lock);

	return 0;
}

static int kopin_a230_panel_set_update_mode(struct omap_dss_device *dssdev, enum omap_dss_update_mode mode)
{
	if (mode != OMAP_DSS_UPDATE_MANUAL)
		return -EINVAL;

	return 0;
}

static enum omap_dss_update_mode kopin_a230_panel_get_update_mode(struct omap_dss_device *dssdev)
{
	return OMAP_DSS_UPDATE_MANUAL;
}

static ssize_t kopin_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	if(!strncmp(buf, "bootmode=1", 10)) {
		printk(KERN_INFO "[%s:%u] BOOTMODE_PREWARNING\n",__FUNCTION__,__LINE__);
		a230_set_pwm(0);
	}
	else if(!strncmp(buf, "bootmode=2", 10)) {
		printk(KERN_INFO "[%s:%u] BOOTMODE_ANDROID\n",__FUNCTION__,__LINE__);
		bootmode=BOOTMODE_ANDROID;
		a230_set_pwm(kopin_current_pwm);
	}

    *ppos = count;

    return count;
}

static struct file_operations kopin_proc_fops = {
	.write		= kopin_proc_write,
	.llseek		= seq_lseek,
};

static int kopin_a230_panel_probe(struct omap_dss_device *dssdev)
{
	int r = 0;
	struct panel_drv_data *ddata;
	struct rfbi_timings rfbi_t;
	const char *bootmode_str = NULL;

	printk(KERN_INFO "[%s:%u] \n", __FUNCTION__,__LINE__);

	bootmode_str = strstr(boot_command_line, "bootmode=");
	if(bootmode_str != NULL) {
		bootmode = bootmode_str[9]-'0';
		printk(KERN_INFO "[%s:%u] bootmode=[%d]\n", __FUNCTION__,__LINE__,bootmode);
	}

	ddata = kzalloc(sizeof(*ddata), GFP_KERNEL);
	if (!ddata) {
	printk(KERN_ERR "[%s:%u] kzalloc(sizeof(*ddata) failed\n", __FUNCTION__,__LINE__);
		r = -ENOMEM;
		goto err_kzalloc;
	}

	ddata->dssdev = dssdev;

	mutex_init(&ddata->lock);

/* 
 * Kopin init begins. Note: 
 * All related pins below have been initialized in the kernel init code board-4430jet.c
*/
	//mdelay(100);
	//gpio_set_value(A230_POWER_ENABLE_GPIO, 1); //Enable power for Kopin

	//mdelay(100);
#ifdef CONFIG_JET_V2
	gpio_direction_output(A230_CLK_TV_EN_GPIO, 1);//Enable 27MHz clock for Kopin
	mdelay(1);
	gpio_direction_output(A230_DISP_RES_GPIO, 1);
#endif
	gpio_direction_output(RFBI_A16_GPIO, 0);
	/*Kopin init ends*/

	dssdev->type				= OMAP_DISPLAY_TYPE_DBI; // Display Bus Interface
	dssdev->ctrl.pixel_size		= 16;
	dssdev->phy.rfbi.data_lines	= 16;
	dssdev->phy.rfbi.channel	= OMAP_DSS_CHANNEL_LCD;
	dssdev->panel.config		= OMAP_DSS_LCD_TFT;
	dssdev->panel.timings		= kopin_a230_timings;

	rfbi_t.cs_on_time	= 0;
	rfbi_t.we_on_time	= rfbi_t.cs_on_time + MCLK_PS;
	rfbi_t.re_on_time	= rfbi_t.cs_on_time + 75000;
	rfbi_t.we_off_time	= rfbi_t.we_on_time + 2*MCLK_PS;
	rfbi_t.re_off_time	= rfbi_t.re_on_time + 150000;
	rfbi_t.access_time	= 0;
	rfbi_t.cs_off_time	= rfbi_t.we_off_time;
	rfbi_t.we_cycle_time	= rfbi_t.cs_off_time + MCLK_PS;
	rfbi_t.re_cycle_time	= rfbi_t.re_off_time + 40000;
	rfbi_t.cs_pulse_width	= 80000;
	rfbi_t.converted	= 0;
	rfbi_t.clk_div 		= 1;
	dssdev->ctrl.rfbi_timings = rfbi_t;

	dev_set_drvdata(&dssdev->dev, ddata);

	if (dssdev->platform_enable)
	{
		printk(KERN_INFO "[%s:%u] \n", __FUNCTION__,__LINE__);
		dssdev->platform_enable(dssdev);
	}

	proc_create_data("kopinctrl", S_IWUSR, NULL, &kopin_proc_fops, NULL);

	return 0;

err_kzalloc:
	return r;
}

static struct omap_dss_driver kopin_a230_driver = {
	.probe		= kopin_a230_panel_probe,
	.remove		= __exit_p(kopin_a230_panel_remove),

	.enable		= kopin_a230_panel_enable,
	.disable	= kopin_a230_panel_disable,
	.suspend	= kopin_a230_panel_suspend,
	.resume		= kopin_a230_panel_resume,

	.set_update_mode = kopin_a230_panel_set_update_mode,
	.get_update_mode = kopin_a230_panel_get_update_mode,

	.update		= kopin_a230_panel_update,
	.sync		= kopin_a230_panel_sync,

	.get_resolution = kopin_a230_panel_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,


	.get_timings	= kopin_a230_panel_get_timings,
	.check_timings	= kopin_a230_panel_check_timings,

	.driver			= {
		.name	= "kopin_a230_panel",
		.owner	= THIS_MODULE,
	},
};

static void a230_init_display_bl(void)
{
}

// A230 brightness is ranged 0x000-0x800 : 0-2048, but we cut off at 1024
// Android input brightness is ranged 0x14-0xFF : 20-255
// so by mult by 4 will cause 0x50-0x3FF:80-1023 which is a good range
static void a230_set_primary_brightness(u8 brightness)
{
	//printk(KERN_INFO "[%s:%u] brightness=0x%x\n",__FUNCTION__,__LINE__,brightness);
#ifdef CONFIG_JET_SUN
	kopin_current_pwm = brightness*16;
#else
	kopin_current_pwm = brightness*4;
#endif
	if(bootmode < BOOTMODE_ANDROID) {
		return;
	}

	a230_set_pwm(kopin_current_pwm);
}

static struct omap4430_jet_disp_led_platform_data a230_disp_bl_data = {
	.display_led_init = a230_init_display_bl,
	.primary_display_set = a230_set_primary_brightness,
};

static struct platform_device a230_disp_bl = {
	.name	= "display_led",
	.id		= -1,
	.dev	= {
		.platform_data = &a230_disp_bl_data,
	},
};

static int __init kopin_a230_panel_drv_init(void)
{
	int ret = 0;

	printk(KERN_INFO "[%s:%u] \n", __FUNCTION__,__LINE__);

	ret = platform_device_register(&a230_disp_bl);
	if(ret)
		goto led_register_failed;

	ret =  omap_dss_register_driver(&kopin_a230_driver);
	if(ret)
		goto dss_register_failed;

	return 0;

dss_register_failed:
	platform_device_unregister(&a230_disp_bl);

led_register_failed:
	return ret;
}

static void __exit kopin_a230_panel_drv_exit(void)
{
	printk(KERN_INFO "[%s:%u] \n", __FUNCTION__,__LINE__);
	omap_dss_unregister_driver(&kopin_a230_driver);
}

module_init(kopin_a230_panel_drv_init);
module_exit(kopin_a230_panel_drv_exit);

MODULE_LICENSE("GPL");
