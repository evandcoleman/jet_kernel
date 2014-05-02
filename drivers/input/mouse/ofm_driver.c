/********************************************************************************
* (C) COPYRIGHT 2011PARTRON
* File Name : ofm_driver.c
* Author		:	Young-Bong Choi
*
* Version   : V1.5
* Date      : 01/September/2011
*
* Target CPU : Samsung S3C_SPV210 Processor
* Linux Kernel : 2.6.35 -> 3.0.8
* Android Ver : 2.3.1 Gingerbread
* Version info
* v1.6 : 03/JAN/2013	- modify mouse mode.
			  (add irq_enable / modify key_code)
			- modify left_event for mouse mode
* v1.5 : 15/DEC/2011	- add motion interrupt mode
*			- add sysfs
* v1.4 : 10/JUN/2010	-  movements improved (I2C / read_word_data)
*			-  ofm i2c address 0xA6
* v1.2 : worqueue debugging
			- left button debugging
********************************************************************************/
//FOR Mango210

#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#include <linux/regulator/consumer.h>

#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/freezer.h>

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include "../../misc/recon/i2c_transfer.h"

#define DRIVER_NAME   "ofm_driver"

//#define OFM_ROTATED

#define OFM_WAKEUP_BY_DOMEKEY

#define ABS(x) (x>0)?x:-x //add by ybchoi 2011.06.22

#define OFM_ON 1
#define OFM_OFF 0

#define DCLICK_TIMER_ID 0
#define LCLICK_TIMER_ID 1
#define CLEAR_TIMER_ID 2

#define KEY_PRESSED	1
#define KEY_RELEASED 0
#define I2C_READ_WORD_DATA 1		//for I2C Multi read function

#define DCLICK_TIME (200*NSEC_PER_MSEC) //200ms
#define LCLICK_TIME (1000*NSEC_PER_MSEC) //1000ms
#define CLEAR_TIME (200*NSEC_PER_MSEC) //200ms

#define OFM_MOTION_GPIO	176
#define OFM_MOTION_IRQ	176	//OMAP pin[J25] GPI_176 connected to Partron M33C01A/MS37C01A pin.4 MOTION

#define OFM_KEY_GPIO	175
#define OFM_KEY_IRQ		175	//OMAP pin[J26] GPI_175 connected to Partron M33C01A/MS37C01A pin.5 DOME_CENT

#define OFM_STANDBY_GPIO 49 //OMAP pin[D20] GPIO_49 connected to Partron M33C01A/MS37C01A pin.3 STANDBY

#define OFM_PWRDWN_GPIO	37  //OMAP pin[D18] GPIO_37 connected to Partron MS37C01A pin.11 POWERDOWN

#define MAX_KEYPAD_CNT	7

#define OFM_SENSITIVITY_MAX_LEVEL  9
#define OFM_SENSITIVITY_X_DEFAULT  0
#define OFM_SENSITIVITY_Y_DEFAULT  0

#define OFM_DEV_VD5376	0	// Partron M33C01A_Rev_1.2
#define OFM_DEV_VD5377	1	// Partron MS37C01A_Rev_1.5

//#define OFM_DEBUG_EN		//OFM debug data logging enable

#define OFM_TEST_DATA_SIZE	(2*32768) //32 Kbytes for expotime, feat, state, etc

#define OFM_IMAGE_DUMP_SIZE	400 //400 bytes
#define OFM_IMAGE_INFO_SIZE	20 //20 bytes image info feat, expotime etc

//#define OFM_9AXIS_EN	//OFM events coordination with 9-Axis sensor (LSM9DS0) in userapace

#define NO_INTR_EVENT 0 //No Interrupt event
#define INTR_EVENT 1    //Interrupt event

#ifdef OFM_DEBUG_EN
static u8 OfmTestData[OFM_TEST_DATA_SIZE];
#endif

static int gv_Test_Counter = 0;

static int gv_TXcount_size = 0;
static int gv_OfmTestData_full = 0;


// First State: 		e_ofm_uncovered
// Interrupt: 			e_ofm_counting_covered_not_fired
//
// e_ofm_counting_covered_not_fired:
// Treshold Reached: 	e_ofm_counting_covered_has_fired
// Uncover: 			e_ofm_uncovered
// Timeout: 			e_ofm_covered_idle
//
// e_ofm_counting_covered_has_fired:
// Timeout: 			e_ofm_covered_scrolling
// Uncover: 			e_ofm_uncovered
//
// e_ofm_covered_idle:
// Uncover: 			e_ofm_uncovered
//
// e_ofm_covered_scrolling:
// Uncover: 			e_ofm_uncovered

typedef enum  {
	e_ofm_counting_covered_not_fired	= 0,
	e_ofm_counting_covered_has_fired	= 1,
	e_ofm_covered_scrolling				= 2,
	e_ofm_covered_idle					= 3,
	e_ofm_uncovered						= 4,
	e_ofm_first_tap						= 5,
	e_ofm_second_tap					= 6,
} ofm_state;

#define MOTION_THRESHOLD	70
//#define COVER_TIMER			500
#define SCROLLING_TIMER		50
#define SCROLLING_TIMER_ACC	15
#define MIN_SCROLLING_TIMER	20
#define COVER_THRESHOLD	    900
#define EXPOTIME_SUN	    30
#define MIN_TAP_SELECT		7
#define MAX_TAP_SELECT		50
#define MIN_TAP_TAP_DELAY	7
#define MAX_TAP_TAP_DELAY	50
#define MAX_SELECT_MOVE		20
#define MAX_BACK_MOVE		30
#define MIN_TAP_BACK		100

#define no_key     -1
#define down_key	0
#define right_key	1
#define left_key	2
#define up_key		3
#define enter_key	4
#define back_key	5
#define home_key	6

#define RIGHT_KEY_FORWARD

static void ofm_enable_irq(void);
static void ofm_disable_irq(void);
static int ofm_i2c_write(struct i2c_client *client, u_int8_t index, u_int8_t data);
static void ofm_motion_func(struct work_struct *work);
static void ofm_VD5377_manual_mode(void);
static void ofm_VD5377_auto_mode(void);
static void ofm_AMF_patch(void);
static void ofm_reg_dump(void);

#ifdef OFM_DEBUG_EN
static int ofm_image_dump(int _mode);
static int ofm_test_data_dump(void);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ofm_early_suspend(struct early_suspend *h);
static void ofm_late_resume(struct early_suspend *h);
#endif

#ifdef CONFIG_EARLYSUSPEND
/* kernel/power/earlysuspend.c */
void request_suspend_state(suspend_state_t state);
#endif

int g_bTimerEnabled = 0;
int g_bTimerLongEnabled = 0;
int g_bTimerClearEnabled = 0;
int g_iClickCnt = 0;
int g_bIsValidOfmDevice = 0;

s16 sum_x=0, sum_y=0, abs_vsqr=0;

int gv_TestSum_DeltaX;
int gv_TestSum_DeltaY;

int gv_OFM_dev_ver = -1; //Un-initialized OFM device

int gv_IRQEvent = 0;


struct event_time {
	long int start_sec;
	long int start_ns;
	long int end_sec;
	long int end_ns;
};

struct ofm_data{
	struct event_time pri_event; //start at motion IRQ and end at key event
	struct event_time sec_event; //start at e_ofm_first_tap and end at e_ofm_second_tap (Tap_Tap Enter key only)
	int key;
	u8 absvsqr;		//abs_vsqr
	u8 state;		//current_ofm_state
	u8 feature;		// When motion irq signaled
	u8 expotime;	// When motion irq signaled
	u8 stimer;		//ofm_state_timer
	u8 dummy1;
	u8 dummy2;
	u8 dummy3;
};

#define OFM_IOC_READ_EVENT	_IOWR('a', 0x01, struct ofm_data) // userspace interface

#ifdef OFM_9AXIS_EN
struct ofm_data ofm_data_current;	//OFM current data
struct ofm_data ofm_data_kevent;	//OFM key tgrigger data sent to userspace
#endif

struct ofm_pin{
	int 	pin;
	int 	irq;
	char	*name;
};

struct ofm{
	struct 	delayed_work work;
	struct	i2c_client *client;
	struct	input_dev *input_dev;
	struct 	miscdevice misc_dev;
	struct	ofm_pin *ofm_left;
	struct	ofm_pin *ofm_motion;
	struct	ofm_pin *ofm_standby;
	struct	ofm_pin *ofm_pwrdwn;

	struct	hrtimer timer_click; //click
	struct	hrtimer timer_long; //long click
	struct	hrtimer timer_clear;
	ktime_t	dclick_time;
	ktime_t lclick_time;
	ktime_t clear_time;
	//atomic_t enable;                /* attribute value */
	wait_queue_head_t signal;
	atomic_t interrupt_state;

	unsigned int  x_level;
	unsigned int  y_level;
	bool ofm_suspend;
	struct mutex ops_lock;
	struct early_suspend early_suspend;
	ofm_state current_ofm_state;
	unsigned int ofm_state_timer;
	unsigned int ofm_scrolling_speed;
    int ofm_key_code;
    char *ofm_img_data;

};

static struct ofm_pin ofm_left	 ={
	OFM_KEY_GPIO,
	-1,
	"ofm_left"
};

static struct ofm_pin ofm_motion	 ={
	OFM_MOTION_GPIO,
	-1,
	"ofm_motion"
};

static struct ofm_pin ofm_standby	={
	OFM_STANDBY_GPIO,
	-1,
	"ofm_standby"
};

static struct ofm_pin ofm_pwrdwn	={
	OFM_PWRDWN_GPIO,
	-1,
	"ofm_pwrdwn"
};


struct ofm ofm_global;
//motion Workqueue
struct workqueue_struct *ofm_workqueue;

static struct regulator *ofm_switch_reg;

//add by ybchoi 2011.06.22
unsigned int ofm_keypad_keycode_map[MAX_KEYPAD_CNT] = {
	KEY_DOWN,
	KEY_RIGHT,
	KEY_LEFT,
	KEY_UP,
	KEY_ENTER,
	KEY_BACK,
	KEY_HOME,
};
unsigned char ofm_keypad_keycode_str[MAX_KEYPAD_CNT][20] = {
	"KEY_DOWN",
	"KEY_RIGHT",
	"KEY_LEFT",
	"KEY_UP",
	"KEY_ENTER",
	"KEY_BACK",
	"KEY_HOME",
};

static int ofm_get_device_ver(void)
{

	s32 major = -1;
	s32 minor = -1;

	//Partron OFM device version
	//M33C01A_Rev_1.2 with VD5376 (ver 0.3)
	//MS37C01A_Rev_1.5 with VD5377 (ver 1.0)

	major = i2c_smbus_read_byte_data(ofm_global.client,0x00); //major
	if(major < 0) {
		printk("[OFM] %s:%d Read Error!!! (%d)\n",__FUNCTION__,__LINE__,major); // negative errno
		return major;
	}

	minor = i2c_smbus_read_byte_data(ofm_global.client,0x01); //minor
	if(minor < 0) {
		printk("[OFM] %s:%d Read Error!!! (%d)\n",__FUNCTION__,__LINE__,minor); // negative errno
		return minor;
	}

	if((major == 0) && (minor == 3)) //VD5376
		gv_OFM_dev_ver = OFM_DEV_VD5376;

	if((major == 1) && (minor == 0)) //VD5377
		gv_OFM_dev_ver = OFM_DEV_VD5377;

	return 0;
}

static void ofm_VD5377_auto_mode(void)
{

	gpio_set_value_cansleep(ofm_global.ofm_pwrdwn->pin, 0); //pwrdwn_pin set to low, MS37C01A power switched on
	mdelay(2); //delay 2ms

	gpio_set_value_cansleep(ofm_global.ofm_standby->pin, 0); //standby_pin set to low, MS37C01A STANDBY de-asserted
	mdelay(2); //delay 2ms

	ofm_i2c_write(ofm_global.client,0x16,0x1E); //software reset
	mdelay(1); //delay 2ms

	ofm_AMF_patch(); //Automatic Movement Filter Patch (Only used to Automatic power mode)

	ofm_i2c_write(ofm_global.client,0x29,0x14); //Min feature (0x05 5x16=80) (0x10 16x16=256) (0x14* 20x16=320) (0x20 32x16=512) (def 0x30 48x16=768) (0x40 64x16=1024)

	ofm_i2c_write(ofm_global.client,0x2A,0x04); //CPI 0x02=100 CPI, 0x04=200 CPI, 0x08=400 CPI, 0x0C=600 CPI, 0x10=800 CPI for X data (0x10)
	ofm_i2c_write(ofm_global.client,0x2B,0x04); //CPI 400 CPI for Y data (0x10)
	ofm_i2c_write(ofm_global.client,0x27,0x03); //X Y Direction invert_x, invert_y [DN=-y, UP=+y, LT=+x, RT=-x]

	//Automatic exposure control (6.4)
	ofm_i2c_write(ofm_global.client,0x43,0x01); //AUTO EXPOSURE_CONTROL 0=Disable 1=Enable (Auto exposure control)

	ofm_i2c_write(ofm_global.client,0x45,0xF0); //MAX_EXPO_PIX_THRESH_HIGH (MEP_HighT) high threshold value of max exposed pixel (def 0xF0 240)
	ofm_i2c_write(ofm_global.client,0x46,0xB4); //MAX_EXPO_PIX_THRESH_LOW (MEP_LowT) low threshold value of max exposed pixel (def 0xB4 180)

	ofm_i2c_write(ofm_global.client,0x49,0xFF); //EXPOTIME_MAX (ExpoMax) maximum exposure time (def 0xFF  255 for 3.3 kf/s)
	ofm_i2c_write(ofm_global.client,0x4A,0x01); //EXPOTIME_MIN (ExpoMin) minimum exposure time (def 0x01)  (0x10 16) (0x00 OK)

	ofm_i2c_write(ofm_global.client,0x4B,0x00); //EXPO_FRAME_UPDATE, exposure update frequency (def 0x01 update every two frames, 00-every frame)

	ofm_i2c_write(ofm_global.client,0x4E,0x08); //EXPOTIME_INC_STEP (IncStep) exposure increment step (def 0x04) 0x08
	ofm_i2c_write(ofm_global.client,0x4F,0x08); //EXPOTIME_DEC_STEP (DecStep) exposure decrement step  (def 0x04) 0x08
	ofm_i2c_write(ofm_global.client,0x50,0x40); //EXPOTIME_SAT_DEC_STEP (SatDecStep), exposure decrement step when max_expo_pix is saturated at 255.
													//(def 0x10 16)

	ofm_i2c_write(ofm_global.client,0x03,0xFC); //Set ANALOG_CTRL2 DMIB DAC Vref setting = 1.6V
	ofm_i2c_write(ofm_global.client,0x0C,0x5A); //Set Motion pin. IO not pulled down, normal config no open drain  HIGH on new board
	//ofm_i2c_write(ofm_global.client,0x0C,0x52); //Set Motion pin. IO pulled down (internal 35K pull-down resistor), normal config no open drain
	//ofm_i2c_write(ofm_global.client,0x0C,0xD2); //Set Motion pin. IO pulled down (internal 35K pull-down resistor), config open drain
	//ofm_i2c_write(ofm_global.client,0x0C,0xDA); //Set Motion pin. IO not pulled down, config open drain

	//Automatic frame rate control (6.7)
	//note: maximum frame rate is limited to 3.3 kf/s in sunlight timing mode.
	ofm_i2c_write(ofm_global.client,0x51,0x02); //Set sunlight mode 0x00(Normal) 0x01(Automatic) 0x02(always ON)
	ofm_i2c_write(ofm_global.client,0x1C,0x82); //Set FRAME_RATE_CONTROL Automatic, 2.9 kf/s (350 us period)
	//ofm_i2c_write(ofm_global.client,0x1C,0xA5); //Set FRAME_RATE_CONTROL Automatic, 3.3 kf/s (300 us period)

	ofm_i2c_write(ofm_global.client,0x32,0x30); //Set SPARE motion_threshold_low register for frame rate 3.3 kf/s
	ofm_i2c_write(ofm_global.client,0x28,0x74); //Set high-pass filter 0x74(5x5 filter), 0x54(3x3 filter)


	//ofm_i2c_write(ofm_global.client,0x05,0x0D); //system configuration (Automatic Mode, MOTION pin active HIGH, STANDBY not used 0000 1101)
	//ofm_i2c_write(ofm_global.client,0x05,0x09); //system configuration (Automatic Mode, MOTION pin active LOW,  STANDBY not used 0000 1001)
	ofm_i2c_write(ofm_global.client,0x05,0x1D); //system configuration (Automatic Mode, MOTION pin active HIGH, STANDBY used     0001 1101)  rd=> 0x9d
	//ofm_i2c_write(ofm_global.client,0x05,0x19); //system configuration (Automatic Mode, MOTION pin active LOW,  STANDBY used     0001 1001)	rd=> 0x99
	mdelay(2);

}

static void ofm_VD5377_manual_mode(void)
{
	gpio_set_value_cansleep(ofm_global.ofm_pwrdwn->pin, 0); //pwrdwn_pin set to low, MS37C01A power switched on
	mdelay(2); //delay 2ms

	gpio_set_value_cansleep(ofm_global.ofm_standby->pin, 0); //standby_pin set to low, MS37C01A STANDBY de-asserted
	mdelay(2); //delay 2ms

	ofm_i2c_write(ofm_global.client,0x16,0x1E); //software reset
	mdelay(2); //delay 2ms

	ofm_i2c_write(ofm_global.client,0x29,0x32); //Min feature (0x05 5x16=80) (0x10 16x16=256) (0x14* 20x16=320) (0x20 32x16=512) (def 0x30 48x16=768) (0x32 50x16=800) (0x40 64x16=1024)

	ofm_i2c_write(ofm_global.client,0x2A,0x04); //CPI 0x02=100 CPI, 0x04=200 CPI, 0x08=400 CPI, 0x0C=600 CPI, 0x10=800 CPI for X data (0x10)
	ofm_i2c_write(ofm_global.client,0x2B,0x04); //CPI 400 CPI for Y data (0x10)
	ofm_i2c_write(ofm_global.client,0x27,0x03); //X Y Direction invert_x, invert_y [DN=-y, UP=+y, LT=+x, RT=-x]

	//Automatic exposure control (6.4)
	ofm_i2c_write(ofm_global.client,0x43,0x01); //AUTO EXPOSURE_CONTROL 0=Disable 1=Enable (Auto exposure control)

	ofm_i2c_write(ofm_global.client,0x45,0xF0); //MAX_EXPO_PIX_THRESH_HIGH (MEP_HighT) high threshold value of max exposed pixel (def 0xF0 240)
	ofm_i2c_write(ofm_global.client,0x46,0xB4); //MAX_EXPO_PIX_THRESH_LOW (MEP_LowT) low threshold value of max exposed pixel (def 0xB4 180)

	ofm_i2c_write(ofm_global.client,0x49,0xFF); //EXPOTIME_MAX (ExpoMax) maximum exposure time (def 0xFF  255 for 3.3 kf/s)
	ofm_i2c_write(ofm_global.client,0x4A,0x01); //EXPOTIME_MIN (ExpoMin) minimum exposure time (def 0x01)  (0x10 16)

	ofm_i2c_write(ofm_global.client,0x4B,0x00); //EXPO_FRAME_UPDATE, exposure update frequency (def 0x01 update every two frames, 00-every frame)

	ofm_i2c_write(ofm_global.client,0x4E,0x08); //EXPOTIME_INC_STEP (IncStep) exposure increment step (def 0x04) 0x08
	ofm_i2c_write(ofm_global.client,0x4F,0x08); //EXPOTIME_DEC_STEP (DecStep) exposure decrement step  (def 0x04) 0x08
	ofm_i2c_write(ofm_global.client,0x50,0x40); //EXPOTIME_SAT_DEC_STEP (SatDecStep), exposure decrement step when max_expo_pix is saturated at 255.
													//(def 0x10 16)

	ofm_i2c_write(ofm_global.client,0x03,0xFC); //Set ANALOG_CTRL2 DMIB DAC Vref setting = 1.6V
	ofm_i2c_write(ofm_global.client,0x0C,0x5A); //Set Motion pin. IO not pulled down, normal config no open drain  HIGH on new board

	//Automatic frame rate control (6.7)
	//note: maximum frame rate is limited to 3.3 kf/s in sunlight timing mode.
	ofm_i2c_write(ofm_global.client,0x51,0x02); //Set sunlight mode 0x00(Normal) 0x01(Automatic) 0x02(always ON)
	ofm_i2c_write(ofm_global.client,0x1C,0xA5); //Set FRAME_RATE_CONTROL Automatic, 3.3 kf/s (300 us period)

	ofm_i2c_write(ofm_global.client,0x56,0x00); //0= CDS frame

	ofm_i2c_write(ofm_global.client,0x32,0x30); //Set SPARE motion_threshold_low register for frame rate 3.3 kf/s
	ofm_i2c_write(ofm_global.client,0x28,0x74); //Set high-pass filter 0x74(5x5 filter), 0x54(3x3 filter)

	ofm_i2c_write(ofm_global.client,0x05,0x1C); //system configuration (Manual Mode, MOTION pin active HIGH, STANDBY used     0001 1100) 	rd=> 0xdc

	mdelay(2);
}

static void ofm_ctrl_power(int on_off)
{
	s32 result;
	int empty_count = 0;

	printk("[OFM] ofm_ctrl_power (%s)\n", (on_off==OFM_ON)?"ON":"OFF");
	if(on_off == OFM_ON){

		if(gv_OFM_dev_ver == OFM_DEV_VD5377){

			ofm_VD5377_manual_mode();
			//ofm_VD5377_auto_mode();
		}
		else if(gv_OFM_dev_ver == OFM_DEV_VD5376){

			//powerdown_pin set to low
			//s3c_gpio_cfgpin(ofm_global.ofm_pd->pin, ofm_global.ofm_pd->pin_setting);
			//s3c_gpio_setpull(ofm_global.ofm_pd->pin, S3C_GPIO_PULL_DOWN);
			//s3c_gpio_setpin(ofm_global.ofm_pd->pin, 0);
			//mdelay(1); //wating for gpio get stable

			ofm_i2c_write(ofm_global.client,0x05,0x28); //power management setting (Manual power mode)
			mdelay(1);
			ofm_i2c_write(ofm_global.client,0x2A,0x08); //Set 400 CPI for X data
			ofm_i2c_write(ofm_global.client,0x2B,0x08); //Set 400 CPI for Y data
			ofm_i2c_write(ofm_global.client,0x27,0x1A); //Set X Y Direction
			ofm_i2c_write(ofm_global.client,0x29,0x05); //Set Min feature value   (5x64=320)
			ofm_i2c_write(ofm_global.client,0x0E,0x67); //Return Motion pin to MCU control
		}
		else{
			printk(KERN_INFO "[OFM] %s:%d  Un-initialized OFM device.\n",__FUNCTION__,__LINE__);
		}

		if(g_bIsValidOfmDevice) {
			result = i2c_smbus_read_word_data(ofm_global.client,0x21);
			printk("[OFM] remove waste data (%d/%d)\n", (result&0xff), ((result>>8)&0xff));
			while(result){
				result = i2c_smbus_read_word_data(ofm_global.client,0x21);
				printk("[OFM] remove waste data (%d/%d)\n", (result&0xff), ((result>>8)&0xff));
				empty_count++;
				if(empty_count > 10){
					printk("[OFM] remove waste data fource stop\n");
					break;
				}
			}
			ofm_enable_irq();
		}

		ofm_global.ofm_suspend = false;

	}
	else{ // OFM_OFF

		if(gv_OFM_dev_ver == OFM_DEV_VD5377){

			//pwrdwn_pin set to high
			gpio_set_value_cansleep(ofm_global.ofm_pwrdwn->pin, 1);
			mdelay(2); //delay 2ms

			//standby_pin set to high
			//gpio_set_value_cansleep(ofm_global.ofm_standby->pin, 1);
			//mdelay(2); //delay 2ms
		}
		else if(gv_OFM_dev_ver == OFM_DEV_VD5376){

			//VD5376 Registers in M33C01A
			ofm_i2c_write(ofm_global.client,0x0F,0x40); //Set Motion pin = 0(no motion)
			ofm_i2c_write(ofm_global.client,0x0E,0x65); //Set Motion pin to manual control
			ofm_i2c_write(ofm_global.client,0x05,0x2C); //Set Internal regulator to low power standby mode

			//powerdown_pin set to high
			//s3c_gpio_cfgpin(ofm_global.ofm_pd->pin, ofm_global.ofm_pd->pin_setting);
			//s3c_gpio_setpull(ofm_global.ofm_pd->pin, S3C_GPIO_PULL_UP);
			//s3c_gpio_setpin(ofm_global.ofm_pd->pin, 1);
		}
		else{
			printk(KERN_INFO "[OFM] %s:%d  Un-initialized OFM device.\n",__FUNCTION__,__LINE__);
		}

		if(g_bIsValidOfmDevice) {
			ofm_disable_irq();
		}
		ofm_global.ofm_suspend = true;
	}
}

static void ofm_click_run(int code)
{
	printk("[OFM] ofm_click_run %s KEY_PRESSED\n", (char *)&ofm_keypad_keycode_str[code]);
	input_report_key(ofm_global.input_dev, ofm_keypad_keycode_map[code], KEY_PRESSED);
	input_sync(ofm_global.input_dev);
	input_report_key(ofm_global.input_dev, ofm_keypad_keycode_map[code], KEY_RELEASED);
	input_sync(ofm_global.input_dev);
	g_bTimerEnabled = 0;
	g_iClickCnt = 0;
}

static void ofm_timer_enable(int mode)
{
	/* int hrtimer_start(struct hrtimer *timer,  	-->timer struct.
					ktime_t time,			-->timer expire time.
					const enum hrtimer_mode mode);  -->time mode(absolute mode or relative mode
	*/
	if(mode == DCLICK_TIMER_ID){
		printk("[OFM] ofm_timer_enable, timer delay %lldns\n", ktime_to_ns(ofm_global.dclick_time));
		hrtimer_start(&ofm_global.timer_click, ofm_global.dclick_time, HRTIMER_MODE_REL);
		g_bTimerEnabled = 1;
	}
	else if(mode == LCLICK_TIMER_ID){
		printk("[OFM] ofm_timer_long_enable, timer delay %lldns\n", ktime_to_ns(ofm_global.lclick_time));
		hrtimer_start(&ofm_global.timer_long, ofm_global.lclick_time, HRTIMER_MODE_REL);
		g_bTimerLongEnabled = 1;
	}
	else if(mode == CLEAR_TIMER_ID)	{
		hrtimer_start(&ofm_global.timer_clear, ofm_global.clear_time, HRTIMER_MODE_REL);
		g_bTimerClearEnabled = 1;
	}
}

static void ofm_timer_disable(int mode)
{
	if(mode == DCLICK_TIMER_ID){
		printk("[OFM] ofm_timer_click_disable \n");
		hrtimer_cancel(&ofm_global.timer_click);
		g_bTimerEnabled = 0;
	}
	else if(mode == LCLICK_TIMER_ID){
		printk("[OFM] ofm_timer_long_disable \n");
		hrtimer_cancel(&ofm_global.timer_long);
		g_bTimerLongEnabled = 0;
	}
	else if(mode == CLEAR_TIMER_ID)	{
		hrtimer_cancel(&ofm_global.timer_clear);
		g_bTimerClearEnabled = 0;
	}
}

static enum hrtimer_restart ofm_timer_click_func(struct hrtimer *timer)
{
	printk("[OFM] ***********timer execute click function***********\n");
	ofm_click_run(enter_key); //Timer exfire is Enter action
	g_iClickCnt = 0;

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart ofm_timer_long_func(struct hrtimer *timer)
{
	int down = 0;
	down = gpio_get_value(ofm_global.ofm_left->pin) ? 0 : 1; //  KEY_RELEASED : KEY_PRESSED
	if(down){
		printk("[OFM] ***********timer execute long function***********\n");
		ofm_click_run(back_key); //Timer exfire is Home action
		if(g_bTimerEnabled)
			ofm_timer_disable(0);
		g_iClickCnt = 0;
	}
	g_bTimerLongEnabled = 0;
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart ofm_timer_clear_func(struct hrtimer *timer)
{
	sum_x = 0;
	sum_y = 0;

	return HRTIMER_NORESTART;
}

static irqreturn_t ofm_left_event(int irq, void *dev_id)
{
	int down = 0;

	printk("[OFM] [%s::%s:%d]\n", __FILE__, __FUNCTION__, __LINE__ );

	disable_irq_nosync(irq);
	down = gpio_get_value(ofm_global.ofm_left->pin) ? 0 : 1; //  KEY_RELEASED : KEY_PRESSED
	if(down){
		printk("[OFM] BUTTON DOWN (%d)\n",down);
		//if(ofm_global.ofm_suspend)
		//	return IRQ_HANDLED;

		g_iClickCnt++;
		if(g_iClickCnt == 2){
			if(g_bTimerEnabled)
				ofm_timer_disable(DCLICK_TIMER_ID);
			if(g_bTimerLongEnabled)
				ofm_timer_disable(LCLICK_TIMER_ID);
			ofm_click_run(enter_key); //Double click is BACK action
			g_iClickCnt = 0;
		}
		else
			ofm_timer_enable(LCLICK_TIMER_ID);
	}
	else{
		printk("[OFM] BUTTON UP   (%d)\n",down);
		//if(ofm_global.ofm_suspend)
		//{
		//	ofm_ctrl_power(OFM_ON);
		//	return IRQ_HANDLED;
		//}
		if(!g_bTimerEnabled){
			if(g_iClickCnt == 1){
				ofm_timer_enable(DCLICK_TIMER_ID);
			}

			if(g_bTimerLongEnabled){
				ofm_timer_disable(LCLICK_TIMER_ID);
				g_bTimerLongEnabled = 0;
			}
		}
	}
	enable_irq(irq);

	return IRQ_HANDLED;
}

static irqreturn_t ofm_motion_event(int irq, void *dev_id)
{
	struct ofm *ofm = (struct ofm *)dev_id;

	//printk("[OFM] [%s::%s:%d]\n", __FILE__, __FUNCTION__, __LINE__ );

	if (!ofm)
	{
		printk("[OFM] ofm_motion_event interrupt error \n");
		return IRQ_HANDLED;
	}

    ofm_global.current_ofm_state = e_ofm_counting_covered_not_fired;
	ofm_global.ofm_state_timer = 0;
	sum_x = 0;
	sum_y = 0;
	abs_vsqr = 0;
	gv_IRQEvent = 1;
#ifdef OFM_9AXIS_EN
	memset(&ofm_data_current,0,sizeof(ofm_data_current));
	memset(&ofm_data_kevent,0,sizeof(ofm_data_kevent));
#endif

	disable_irq_nosync(irq);
	schedule_delayed_work(&ofm->work, 0);
	return IRQ_HANDLED;
}

static void ofm_reg_dump(void) {
	s32 result;

 	printk("[OFM] ofm_reg_dump begin...\n");

 	if(gv_OFM_dev_ver == OFM_DEV_VD5377){

		result = (s32)gpio_get_value(ofm_global.ofm_motion->pin);
		printk("[OFM] ofm_motion->pin=%d\n",result);

		//0x91 SYSTEM_STATE
		result = i2c_smbus_read_byte_data(ofm_global.client,0x91);
		printk("[OFM] 0x91 SYSTEM_STATE=%d\n",result);

		//0x0C[2] GPIO_MOTION gpio_motion_zi 2 PR 01 MOTION IO value
		result = i2c_smbus_read_byte_data(ofm_global.client,0x0C);
		if(result < 0) // negative errno
			printk("[OFM] 0x0C gpio_motion_zi=%d\n",result);
		else
			printk("[OFM] 0x0C gpio_motion_zi=%d\n",((result>>2)&0x01));

		//0x23[3] OVERFLOW no_motion 3 PR 01 [0 = Motion, 1 = No motion]
		result = i2c_smbus_read_byte_data(ofm_global.client,0x23);
		if(result < 0) // negative errno
			printk("[OFM] 0x23 no_motion=%d\n",result);
		else
			printk("[OFM] 0x23 no_motion=%d\n",((result>>3)&0x01));

		//0x2F MAX_ABS_MOTION max_abs_motion 6:0 PR 00
		result = i2c_smbus_read_byte_data(ofm_global.client,0x2F);
		if(result < 0) // negative errno
			printk("[OFM] 0x2F max_abs_motion=%d\n",result);
		else
			printk("[OFM] 0x2F max_abs_motion=%d\n",(result&0x7F));

		//0x31 FEATURES features_report 7:0 PR 00
		result = i2c_smbus_read_byte_data(ofm_global.client,0x31);
		if(result < 0) // negative errno
			printk("[OFM] 0x31 Features=%d\n",result);
		else
			printk("[OFM] 0x31 Features=%d\n",(result*16));

		//0x21 XY Motion Data
		result = i2c_smbus_read_word_data(ofm_global.client,0x21);
		if(result < 0) // negative errno
			printk("[OFM] 0x21 Motion Data x=%d  y=%d\n",result,result);
		else
			printk("[OFM] 0x21 Motion Data x=%d  y=%d\n",(result&0xff),((result>>8)&0xff));

		//0x0E[2] GPIO_STANDBY gpio_standby_zi 2 PR 00 STANDBY IO value
		result = i2c_smbus_read_byte_data(ofm_global.client,0x0E);
		if(result < 0) // negative errno
			printk("[OFM] 0x0E gpio_standby_zi=%d\n",result);
		else
			printk("[OFM] 0x0E gpio_standby_zi=%d\n",((result>>2)&0x01));

		//0x2C 	FRAME_AVERAGE frame_avg 7:0 PR 00 Frame average calculated over a 16 x 16 centered window.
		result = i2c_smbus_read_byte_data(ofm_global.client,0x2C);
		printk("[OFM] 0x2C frame_avg value=%d\n",result);

		//0x43 EXPOSURE_CONTROL
		result = i2c_smbus_read_byte_data(ofm_global.client,0x43);
		if(result < 0) // negative errno
			printk("[OFM] 0x43 autoexpo_en=%d _status=%d _limit_flag=%d\n",result,result,result);
		else
			printk("[OFM] 0x43 autoexpo_en=%d _status=%d _limit_flag=%d\n",((result)&0x01),((result>>4)&0x07),((result>>7)&0x01));

		//0x44 MAX_EXPO_PIX max_exposed_pixel_value 7:0 PR 00 Second maximum pixel value of the current frame (before CDS)
		result = i2c_smbus_read_byte_data(ofm_global.client,0x44);
		printk("[OFM] 0x44 max_exposed_pixel=%d\n",result);

		//0x45 MAX_EXPO_PIX_THRESH_HIGH max_exposed_pixel_thresh_high
		result = i2c_smbus_read_byte_data(ofm_global.client,0x45);
		printk("[OFM] 0x45 max_exposed_pixel_thresh_high=%d\n",result);

		//0x46 MAX_EXPO_PIX_THRESH_LOW
		result = i2c_smbus_read_byte_data(ofm_global.client,0x46);
		printk("[OFM] 0x46 max_exposed_pixel_thresh_low=%d\n",result);

		//0x47 EXPOTIME
		result = i2c_smbus_read_byte_data(ofm_global.client,0x47);
		printk("[OFM] 0x47 exposure_time=%d\n",result);

		//0x49 EXPOTIME_MAX
		result = i2c_smbus_read_byte_data(ofm_global.client,0x49);
		printk("[OFM] 0x49 exposure_time_max=%d\n",result);

		//0x4A EXPOTIME_MIN
		result = i2c_smbus_read_byte_data(ofm_global.client,0x4A);
		printk("[OFM] 0x4A exposure_time_min=%d\n",result);

		//0x4B EXPO_FRAME_UPDATE
		result = i2c_smbus_read_byte_data(ofm_global.client,0x4B);
		printk("[OFM] 0x4B autoexpo_frame_update=%d\n",result);

		//0x4E EXPOTIME_INC_STEP
		result = i2c_smbus_read_byte_data(ofm_global.client,0x4E);
		printk("[OFM] 0x4E expo_inc_step=%d\n",result);

		//0x4F EXPOTIME_DEC_STEP
		result = i2c_smbus_read_byte_data(ofm_global.client,0x4F);
		printk("[OFM] 0x4F expo_dec_step=%d\n",result);

		//0x50 EXPOTIME_SAT_DEC_STEP
		result = i2c_smbus_read_byte_data(ofm_global.client,0x50);
		printk("[OFM] 0x50 expo_sat_dec_step=%d\n",result);

		//0x1C FRAME_RATE_CONTROL
		result = i2c_smbus_read_byte_data(ofm_global.client,0x1C);
		if(result < 0) // negative errno
			printk("[OFM] 0x1C frame_rate _sel=%d _ctrl=%d max_auto_frame_rate=%d\n",result,result,result);
		else
			printk("[OFM] 0x1C frame_rate _sel=%d _ctrl=%d max_auto_frame_rate=%d\n",(result&0x07), ((result>>4)&0x01), ((result>>5)&0x07));

		//0x32 SPARE
		result = i2c_smbus_read_byte_data(ofm_global.client,0x50);
		if(result < 0) // negative errno
			printk("[OFM] 0x32 SPARE motion_threshold_low_comp=%d\n",result);
		else
			printk("[OFM] 0x32 SPARE motion_threshold_low_comp=%d\n",((result>>4)&0x0F));

		//0x8D AUTO_MOVEMENT_CTRL1
		result = i2c_smbus_read_byte_data(ofm_global.client,0x8D);
		if(result < 0) // negative errno
			printk("[OFM] 0x8D AUTO_MOVEMENT_CTRL1 Enable=%d FrameNb=%d SaturatedExpo=%d\n",result,result,result);
		else
			printk("[OFM] 0x8D AUTO_MOVEMENT_CTRL1 Enable=%d FrameNb=%d SaturatedExpo=%d\n",(result&0x01),((result>>1)&0x3F),((result>>7)&0x01));

		//0x8E AUTO_MOVEMENT_CTRL2
		result = i2c_smbus_read_byte_data(ofm_global.client,0x8E);
		if(result < 0) // negative errno
			printk("[OFM] 0x8E AUTO_MOVEMENT_CTRL2 FilterLatency=%d FilterLoop=%d\n",result, result);
		else
			printk("[OFM] 0x8E AUTO_MOVEMENT_CTRL2 FilterLatency=%d FilterLoop=%d\n",(result&0x0F), ((result>>4)&0x0F));
 	}
 	else if(gv_OFM_dev_ver == OFM_DEV_VD5376){

		result = i2c_smbus_read_word_data(ofm_global.client,0x23);
		printk("[%s:%d] 0x23[0]:X_Oveflow=%d, 0x23[1]Y_Overflow=%d, 0x23[2]:motion_detected=%d, 0x23[3]NoMotion=%d\n",__FUNCTION__,__LINE__,(result&0x01),((result>>1)&0x01),((result>>2)&0x01),((result>>3)&0x01));

		result = i2c_smbus_read_word_data(ofm_global.client,0x31);
		printk("[%s:%d] 0x31:Features Count=%d\n",__FUNCTION__,__LINE__,(result&0xffff));

		result = i2c_smbus_read_word_data(ofm_global.client,0x41);
		printk("[%s:%d] 0x41:Exposure=%d\n",__FUNCTION__,__LINE__,(result&0xff));

		result = i2c_smbus_read_word_data(ofm_global.client,0x4f);
		printk("[%s:%d] 0x4F:Max_Exposed_Pixel=%d\n",__FUNCTION__,__LINE__,(result&0xff));
 	}
 	else{
 		printk(KERN_INFO "[OFM] %s:%d  Un-initialized OFM device.\n",__FUNCTION__,__LINE__);
 	}

 	printk("[OFM] ofm_reg_dump end.\n");
}


#ifdef OFM_DEBUG_EN
/** ofm_test_data_dump()
 * Reads OFM test data (OfmTestData[]), converts in *.csv format and reinitialize test data counter(gv_Test_Counter).
 * shell command "echo 7 > /dev/ofm_driver" will print test data to shell
 */
static int ofm_test_data_dump(void)
{
	int i=0;

	char* OfmTestDataStr;
	char* output;

	OfmTestDataStr  = kmalloc(256, GFP_KERNEL);
	if(OfmTestDataStr == NULL){
		printk(KERN_INFO "[OFM] %s:%d  kmalloc OfmTestDataStr error\n",__FUNCTION__,__LINE__);
		return -ENOMEM;
	}

	printk(KERN_INFO "[OFM] %s:%d  creating CSV data , Max bytes=%d \n",__FUNCTION__,__LINE__,gv_Test_Counter);

	memset(OfmTestDataStr,0,256);
	output=OfmTestDataStr;
	for(i=0; i<gv_Test_Counter ; i++)
	{

		if(OfmTestData[i]==240) { // ENDL
			printk(KERN_INFO "[OFM-TEST] %s\n", OfmTestDataStr);
			output = OfmTestDataStr;
		}
		else {
			output += sprintf(output, "%d,",OfmTestData[i]);
		}
	}
	mdelay(200);
	printk(KERN_INFO "\n[OFM] %s:%d  done Max bytes=%d\n",__FUNCTION__,__LINE__,gv_Test_Counter);

	memset(OfmTestData,0,OFM_TEST_DATA_SIZE);
	gv_Test_Counter =0;

	kfree(OfmTestDataStr);
	return 0;

}
#endif //OFM_DEBUG_EN

#ifdef OFM_DEBUG_EN
/** ofm_image_dump()
 * Captures the OFM image as described in VD5377 datasheet "Sec.9 Image capture "
 * shell command "echo 8 > /dev/ofm_driver" will create a grey level image dump in local memory/
 * shell command "echo 9 > /dev/ofm_driver" will create a real image dump in local memory.
 * shell command "cat /dev/ofm_driver > /ofmimage.raw" will create the hex data file
 * N.B. To create successive image dumps, cat comamnd must be executed after each echo command,
 * this will clear ofm_global.ofm_img_data pointer.
 */
static int ofm_image_dump(int _mode)
{
	int ret=0, ret1=0;
	int i=0, j=0;
	int mode;
	s32 temp;
	s32 result[OFM_IMAGE_INFO_SIZE];
	char* tmp;

	char* dataStr;
	char* output;
	mode=_mode;

	tmp = kmalloc(OFM_IMAGE_DUMP_SIZE+OFM_IMAGE_INFO_SIZE, GFP_KERNEL);
	if((tmp == NULL) || (ofm_global.ofm_img_data != NULL)){
		kfree(tmp);
		return -ENOMEM;
	}
	ofm_global.ofm_img_data = tmp;	//clears by ofm_read

	dataStr  = kmalloc(256, GFP_KERNEL);
	if(dataStr == NULL){
		return -ENOMEM;
	}
	output = dataStr;

	memset(result,0,OFM_IMAGE_INFO_SIZE);

 	printk(KERN_INFO "\n\n\n[OFM] ofm_image_dump begin mode:%d...\n", mode);

 	if(gv_OFM_dev_ver == OFM_DEV_VD5377){

 		//record image info
 		result[0] = 0xAA;
 		result[1] = 0xAA;

 			temp = i2c_smbus_read_word_data(ofm_global.client,0x21); //XY Motion Data
 		result[2] = (temp&0xff); //Motion Data x
 		result[3] = ((temp>>8)&0xff); //Motion Data y

 		result[4] = i2c_smbus_read_byte_data(ofm_global.client,0x31); //features
 		result[5] = i2c_smbus_read_byte_data(ofm_global.client,0x2C); //FRAME_AVERAGE
 		result[6] = i2c_smbus_read_byte_data(ofm_global.client,0x44); //max_exposed_pixel
		result[7] = i2c_smbus_read_byte_data(ofm_global.client,0x47); //EXPOTIME
 		result[8] = i2c_smbus_read_byte_data(ofm_global.client,0x45); //MAX_EXPO_PIX_THRESH_HIGH [0xF0] 240
 		result[9] = i2c_smbus_read_byte_data(ofm_global.client,0x46); //MAX_EXPO_PIX_THRESH_LOW [0xB4] 180
 		result[10] = i2c_smbus_read_byte_data(ofm_global.client,0x49); //EXPOTIME_MAX [0xFF]
 		result[11] = i2c_smbus_read_byte_data(ofm_global.client,0x4A); //EXPOTIME_MIN [0x00]
 		result[12] = i2c_smbus_read_byte_data(ofm_global.client,0x4B); //EXPO_FRAME_UPDATE rate [0x00]
 		result[13] = i2c_smbus_read_byte_data(ofm_global.client,0x4E); //EXPOTIME_INC_STEP [0x08]
 		result[14] = i2c_smbus_read_byte_data(ofm_global.client,0x4F); //EXPOTIME_DEC_STEP [0x08]
 		result[15] = i2c_smbus_read_byte_data(ofm_global.client,0x50); //EXPOTIME_SAT_DEC_STEP [0x40]
 		result[16] = i2c_smbus_read_byte_data(ofm_global.client,0x43); //EXPOSURE_CONTROL status[6:4] limit[7]
 		result[17] = i2c_smbus_read_byte_data(ofm_global.client,0x51); //sunlight mode

 			ret = i2c_smbus_read_byte_data(ofm_global.client,0x28); //0x74(5x5 filter),
 			ofm_i2c_write(ofm_global.client,0x28,(u8)(ret&0xEF)); // Disable Filter (clear bit 4, 0x28)
 			mdelay(1);

 		result[18] = i2c_smbus_read_byte_data(ofm_global.client,0x31); //features no filter
 			ofm_i2c_write(ofm_global.client,0x28,0x74); 	// restore reg 0x28  5x5filtr

 		result[19] = i2c_smbus_read_byte_data(ofm_global.client,0x25); //motion threshold


 		//Init seq. image dump
 		gpio_set_value_cansleep(ofm_global.ofm_motion->pin, 0); //disable motion_pin
 		mdelay(1);

 		ofm_i2c_write(ofm_global.client,0x03,0x70); //Set led_dac_ctrl to max (0x03=0x70) 0111

 		ret = i2c_smbus_read_byte_data(ofm_global.client,0x15);
 		ofm_i2c_write(ofm_global.client,0x15, (u8)(ret|0x20)); //Set 0x20 clk_framedump_en b[5]=1	0010 0000

 		ret = i2c_smbus_read_byte_data(ofm_global.client,0x16);
 		ofm_i2c_write(ofm_global.client,0x16,(u8)(ret|0x20)); //Set 0x20 framedump_reset_n b[5]=1	0010 0000

 		//(mode==8|mode==9|mode==10|mode==11)

 		//ret = i2c_smbus_read_byte_data(ofm_global.client,0x56);
 		switch (mode) //Set output image type (FC=0= CDS frame, 2 = exposed frame, 3 = black frame)
 		{
 			case 9:
 				//ofm_i2c_write(ofm_global.client,0x56,(u8)(ret&0xFC)); //0= CDS frame
 				ofm_i2c_write(ofm_global.client,0x56,0x00); //0= CDS frame
 			break;
 			case 10:
 				//ofm_i2c_write(ofm_global.client,0x56,(u8)(ret&0xFC)); //2 = exposed frame
 				ofm_i2c_write(ofm_global.client,0x56,0x10); //2 = exposed frame
 			break;
 			case 11:
 				//ofm_i2c_write(ofm_global.client,0x56,(u8)(ret&0xFC)); //3 = black frame
 				ofm_i2c_write(ofm_global.client,0x56,0x11); 	//3 = black frame
 			break;
 			default:
 				ofm_i2c_write(ofm_global.client,0x56,0x00); //0= CDS frame
 		}

 		ret = i2c_smbus_read_byte_data(ofm_global.client,0x7C);
 		ofm_i2c_write(ofm_global.client,0x7C,(u8)(ret&0xFE)); // Disable I2C auto increment (reset bit 0 of reg 0x7C) i2cs_index_auto_inc_en b[0]=0 Datasheet 0x7C

 		ret = i2c_smbus_read_byte_data(ofm_global.client,0x19);
 		ofm_i2c_write(ofm_global.client,0x19,(u8)(ret&0x7F)); // Clear motion_engine_start ( reset bit 7 in register 0x19)

 		ofm_i2c_write(ofm_global.client,0x59, 0x01); //Set 0x01 framedump_start b[1]=1 0000 0010

 		if(mode==8)
 			ofm_i2c_write(ofm_global.client,0x59,0x81); //Set 0x80 framedump_mire b[7]=1 1000 0010 outputs a grey scale image instead of the image data.

 		udelay(250); //Wait for 250us

 		ret1 = i2c_smbus_read_byte_data(ofm_global.client,0x56); //CDSOUT_SEL
 		ret = i2c_smbus_read_byte_data(ofm_global.client,0x59); //read framedump_ready b[2]=1    0000 0100 (0x04)
 		printk(KERN_INFO "[OFM] %s:%d  framedump_ready=0x%x CDSOUT=0x%x\n",__FUNCTION__,__LINE__,ret,ret1);

 		if(ret & 0x04) //framedump_ready
 		{
 			ret=mpu_i2c_read(ofm_global.client->adapter, ofm_global.client->addr, 0x58, tmp, OFM_IMAGE_DUMP_SIZE);
 			printk(KERN_INFO "[OFM] %s:%d  read image ret=%d \n",__FUNCTION__,__LINE__,ret);
 			if(ret==0)
 			{
 		 		for(i=OFM_IMAGE_DUMP_SIZE,j=0; i<(OFM_IMAGE_DUMP_SIZE+OFM_IMAGE_INFO_SIZE) ; i++,j++)
 		 		{
 		 			tmp[i]=(u8)result[j];
 		 		}

 		 		//for(i=0; i<=OFM_IMAGE_DUMP_SIZE ; i++,j++)
 		 		for(i=0,j=0; i<=(OFM_IMAGE_DUMP_SIZE+OFM_IMAGE_INFO_SIZE) ; i++,j++)
 		 		{
 		 			if(j==20)
 		 			{
 		 				printk(KERN_INFO "[OFM-IMAGE] %s\n", dataStr);
 		 				memset(dataStr,0,256);
 		 				output = dataStr;
 		 				j=0;
 		 			}
 		 			output += sprintf(output, "%02X,",tmp[i]);
 		 		}
 				gv_TXcount_size = OFM_IMAGE_DUMP_SIZE+OFM_IMAGE_INFO_SIZE;
 			}
 		}

 		//exit seq.
 		ofm_i2c_write(ofm_global.client,0x59,0x00); //Set 0x00 framedump_start b[1]=0

 		ret = i2c_smbus_read_byte_data(ofm_global.client,0x19);
 		ofm_i2c_write(ofm_global.client,0x19,(u8)(ret|0x80)); // Set motion_engine_start ( set bit 7 in register 0x19 )

 		ret = i2c_smbus_read_byte_data(ofm_global.client,0x7C);
 		ofm_i2c_write(ofm_global.client,0x7C,(u8)(ret|0x01)); //Set i2cs_index_auto_inc_en b[0]=1 //Datasheet 0x7C

 		ret = i2c_smbus_read_byte_data(ofm_global.client,0x16);
 		ofm_i2c_write(ofm_global.client,0x16,(u8)(ret&0xDF)); //Set framedump_reset_n b[5]=0

 		ret = i2c_smbus_read_byte_data(ofm_global.client,0x15);
 		ofm_i2c_write(ofm_global.client,0x15,(u8)(ret&0xDF)); //Set clk_framedump_en b[5]=0

 		ofm_i2c_write(ofm_global.client,0x56,0x00); //0= CDS frame

 		ofm_i2c_write(ofm_global.client,0x03,0xFC);

 		gpio_set_value_cansleep(ofm_global.ofm_motion->pin, 1); //enable motion_pin
 		mdelay(1);

 		printk(KERN_INFO "[OFM] ofm_image_dump created. Reseting device.\n\n\n");

		ofm_ctrl_power(OFM_OFF);
		ofm_ctrl_power(OFM_ON);

 	}
 	else{
 		printk(KERN_INFO "[OFM] %s:%d  Un-supported device.\n",__FUNCTION__,__LINE__);
 		ret= -ENODEV;
 	}

 	kfree(dataStr);

	return 0;
}
#endif //OFM_DEBUG_EN


// First State: 		e_ofm_uncovered
// Interrupt: 			e_ofm_counting_covered_not_fired
//
// e_ofm_counting_covered_not_fired:
// Treshold Reached: 	e_ofm_counting_covered_has_fired
// Uncover: 			e_ofm_uncovered
// Timeout: 			e_ofm_covered_idle
//
// e_ofm_counting_covered_has_fired:
// Timeout: 			e_ofm_covered_scrolling
// Uncover: 			e_ofm_uncovered
//
// e_ofm_covered_idle:
// Uncover: 			e_ofm_uncovered
//
// e_ofm_covered_scrolling:
// Uncover: 			e_ofm_uncovered

static void ofm_trigger_key(int key_code) {
	//printk("[%s] sum_x=%d sum_y=%d state=%d timer=%d speed=%d abs_v=%d\n",(char *)&ofm_keypad_keycode_str[key_code],sum_x,sum_y,ofm_global.current_ofm_state,ofm_global.ofm_state_timer,ofm_global.ofm_scrolling_speed,abs_vsqr);
#ifdef OFM_9AXIS_EN
	ofm_data_current.key = key_code;
	memcpy(&ofm_data_kevent,&ofm_data_current,sizeof(ofm_data_kevent));

/**
	printk(KERN_INFO "[OFM] ofm_data_kevent: key=%d state=%d stimer=%d feature=%d expotime=%d\n",
			ofm_data_kevent.key,
			ofm_data_kevent.state,
			ofm_data_kevent.stimer,
			ofm_data_kevent.feature,
			ofm_data_kevent.expotime);

	printk(KERN_INFO "[OFM] ofm_data_kevent: timePri Start sec=%ld ns=%ld End sec=%ld ns=%ld\n",
			ofm_data_kevent.pri_event.start_sec,
			ofm_data_kevent.pri_event.start_ns,
			ofm_data_kevent.pri_event.end_sec,
			ofm_data_kevent.pri_event.end_ns);

	printk(KERN_INFO "[OFM] ofm_data_kevent: timeSec Start sec=%ld ns=%ld End sec=%ld  ns=%ld\n",
			ofm_data_kevent.sec_event.start_sec,
			ofm_data_kevent.sec_event.start_ns,
			ofm_data_kevent.sec_event.end_sec,
			ofm_data_kevent.sec_event.end_ns);
**/
	atomic_set(&ofm_global.interrupt_state, INTR_EVENT);
	wake_up(&ofm_global.signal);
#else
	input_report_key(ofm_global.input_dev, ofm_keypad_keycode_map[key_code], KEY_PRESSED);
	input_sync(ofm_global.input_dev);

	input_report_key(ofm_global.input_dev, ofm_keypad_keycode_map[key_code], KEY_RELEASED);
	input_sync(ofm_global.input_dev);
#endif
	printk("[OFM] ofm_trigger_key %s KEY_PRESSED\n", (char *)&ofm_keypad_keycode_str[key_code]);
}

static void ofm_counting_covered_not_fired(struct work_struct *work, s8 x, s8 y)
{
	ofm_global.ofm_key_code = MAX_KEYPAD_CNT;

//    ofm_reg_dump();

	sum_x += x;
	sum_y += y;
	abs_vsqr += (x*x + y*y);
	//printk("[OFM] %s, %d [ %d x %d ] + [ %d x %d ] abs_vsqr=%d ofm_state_timer=%d\n", __func__, __LINE__,x,y,sum_x,sum_y,abs_vsqr,ofm_global.ofm_state_timer);

	if(abs(sum_x) >= abs(sum_y)) {
		if(abs(sum_x) >= MOTION_THRESHOLD) {
#ifdef OFM_ROTATED
			if(sum_x > 0)	ofm_global.ofm_key_code = up_key;
			else			ofm_global.ofm_key_code = down_key;
#else
#ifdef RIGHT_KEY_FORWARD
			if(sum_x > 0)	ofm_global.ofm_key_code = right_key;
			else			ofm_global.ofm_key_code = left_key;
#else
			if(sum_x > 0)	ofm_global.ofm_key_code = left_key;
			else			ofm_global.ofm_key_code = right_key;
#endif
#endif
		}
		else if(abs(sum_y) >= MOTION_THRESHOLD) {
#ifdef OFM_ROTATED
#ifdef RIGHT_KEY_FORWARD
			if(sum_y < 0)	ofm_global.ofm_key_code = left_key;
			else			ofm_global.ofm_key_code = right_key;
#else
			if(sum_y < 0)	ofm_global.ofm_key_code = right_key;
			else			ofm_global.ofm_key_code = left_key;
#endif
#else
			if(sum_y < 0)	ofm_global.ofm_key_code = down_key;
			else			ofm_global.ofm_key_code = up_key;
#endif
		}
	} else {
		if(abs(sum_y) >= MOTION_THRESHOLD) {
#ifdef OFM_ROTATED
#ifdef RIGHT_KEY_FORWARD
			if(sum_y < 0)	ofm_global.ofm_key_code = left_key;
			else			ofm_global.ofm_key_code = right_key;
#else
			if(sum_y < 0)	ofm_global.ofm_key_code = right_key;
			else			ofm_global.ofm_key_code = left_key;
#endif
#else
			if(sum_y < 0)	ofm_global.ofm_key_code = down_key;
			else			ofm_global.ofm_key_code = up_key;
#endif
		}
		else if(abs(sum_x) >= MOTION_THRESHOLD) {
#ifdef OFM_ROTATED
			if(sum_x > 0)	ofm_global.ofm_key_code = up_key;
			else			ofm_global.ofm_key_code = down_key;
#else
#ifdef RIGHT_KEY_FORWARD
			if(sum_x > 0)	ofm_global.ofm_key_code = right_key;
			else			ofm_global.ofm_key_code = left_key;
#else
			if(sum_x > 0)	ofm_global.ofm_key_code = left_key;
			else			ofm_global.ofm_key_code = right_key;
#endif
#endif
		}
	}

	if(ofm_global.ofm_key_code != MAX_KEYPAD_CNT) {
		ofm_trigger_key(ofm_global.ofm_key_code);
		sum_x = 0;
		sum_y = 0;

#ifdef OFM_DEBUG_EN
		OfmTestData[gv_Test_Counter-1]=100; //100 swipe key
		ofm_image_dump(9);
#endif
		ofm_global.ofm_state_timer = 0;
		ofm_global.ofm_scrolling_speed = SCROLLING_TIMER;
		ofm_global.current_ofm_state = e_ofm_counting_covered_has_fired;
	} else {
		ofm_global.ofm_state_timer++;

		if(ofm_global.ofm_state_timer >= MIN_TAP_BACK) {
			if(abs_vsqr < MAX_BACK_MOVE*MAX_BACK_MOVE) {
				ofm_trigger_key(back_key);

#ifdef OFM_DEBUG_EN
				OfmTestData[gv_Test_Counter-1]=250; //250  back key
				ofm_image_dump(9);
#endif
			}
			ofm_global.ofm_state_timer = 0;
			ofm_global.current_ofm_state = e_ofm_covered_idle;
		}
	}
}

static void ofm_counting_covered_has_fired(struct work_struct *work) {
	//printk("[OFM] %s, %d [ %d x %d ]\n", __func__, __LINE__,sum_x,sum_y);
	ofm_global.ofm_state_timer++;

	if(ofm_global.ofm_state_timer >= ofm_global.ofm_scrolling_speed) {
//		ofm_trigger_key(ofm_global.ofm_key_code);

		ofm_global.ofm_state_timer = 0;
		ofm_global.ofm_scrolling_speed -= SCROLLING_TIMER_ACC;
		ofm_global.current_ofm_state = e_ofm_covered_scrolling;
	}
}

static void ofm_covered_scrolling(struct work_struct *work) {
	//printk("[OFM] %s, %d [ %d x %d ]\n", __func__, __LINE__,sum_x,sum_y);
	ofm_global.ofm_state_timer++;

	if(ofm_global.ofm_state_timer >= ofm_global.ofm_scrolling_speed) {
		ofm_trigger_key(ofm_global.ofm_key_code);

		ofm_global.ofm_state_timer = 0;
		ofm_global.ofm_scrolling_speed -= SCROLLING_TIMER_ACC;
		if(ofm_global.ofm_scrolling_speed < MIN_SCROLLING_TIMER) {
			ofm_global.ofm_scrolling_speed = MIN_SCROLLING_TIMER;
		}
	}
}

static void ofm_motion_func(struct work_struct *work)
{
    s32 result, features=0, expotime=0, featcount=0;
	s8 x = 0;
	s8 y = 0;
	struct timespec ts;

//	printk(KERN_INFO "[OFM] sum_x=%d sum_y=%d state=%d timer=%d speed=%d\n",sum_x,sum_y,ofm_global.current_ofm_state,ofm_global.ofm_state_timer,ofm_global.ofm_scrolling_speed);

	if(gv_OFM_dev_ver == OFM_DEV_VD5377){
		expotime = i2c_smbus_read_byte_data(ofm_global.client,0x47); //EXPOTIME
		if(expotime < 0) {
			printk(KERN_INFO "[OFM] ofm_motion_func expotime read error status=%d \n",expotime); // negative errno
			expotime = 0;
		}

		featcount = i2c_smbus_read_byte_data(ofm_global.client,0x31); //FEATURES
		if(featcount < 0) {
			printk(KERN_INFO "[OFM] ofm_motion_func features read error status=%d (0x%x)\n",featcount, featcount); // negative errno
			featcount = 0;
		}

		features = featcount*16;

		result = i2c_smbus_read_word_data(ofm_global.client,0x21);
		if(result < 0) {
			ofm_global.current_ofm_state = e_ofm_uncovered;
		}
		x = (result&0xff);
		y = ((result>>8)&0xff);

		result = i2c_smbus_read_byte_data(ofm_global.client,0x23);
		if(((result>>3)&0x01)==0) //0=Motion, 1=NoMotion
		{
			// set motion_acc_flush_en (b[5]=1) to flushes the motion accumulators (self cleared)
			ofm_i2c_write(ofm_global.client,0x23, (u8)(result|0x20));
		}

		//printk(KERN_INFO "[OFM] ofm_motion_func  x=%d  y=%d features=%d\n",x,y,features);

#ifdef OFM_DEBUG_EN
		if(gv_Test_Counter > (OFM_TEST_DATA_SIZE-40))
		{
			gv_OfmTestData_full++;
			printk(KERN_INFO "[OFM] ofm_motion_func gv_Test_Counter=%d full=%d\n",gv_Test_Counter, gv_OfmTestData_full);
			gv_Test_Counter = 0 ;
		}
		OfmTestData[gv_Test_Counter++]=240; 									//240 new line tag
		OfmTestData[gv_Test_Counter++]=ofm_global.current_ofm_state;			//csv col-A = state
		OfmTestData[gv_Test_Counter++]=(expotime != 240) ? expotime : 241;		//csv col-B = expotime
		OfmTestData[gv_Test_Counter++]=(featcount != 240) ? featcount : 241;	//csv col-C = feature
		OfmTestData[gv_Test_Counter++]=(abs_vsqr <= 255) ? abs_vsqr : 255; 		//csv col-D = absmot (abs_vsqr)
		OfmTestData[gv_Test_Counter++]=ofm_global.ofm_state_timer;				//csv col-E = timer
		OfmTestData[gv_Test_Counter++]=0; 										//csv col-F = key events 100=SWIPE-KEY, 200=ENTER-KEY, 250=BACK-KEY
#endif

#ifdef OFM_9AXIS_EN
		getnstimeofday(&ts);

		if(gv_IRQEvent == 1) {
			// Capture start time, Feature, Expotime
			ofm_data_current.pri_event.start_sec = ts.tv_sec;
			ofm_data_current.pri_event.start_ns = ts.tv_nsec;

			ofm_data_current.feature = featcount; //Feature count
			ofm_data_current.expotime = expotime; //Expotime
			gv_IRQEvent = 0;
		}
		ofm_data_current.pri_event.end_sec = ts.tv_sec;
		ofm_data_current.pri_event.end_ns = ts.tv_nsec;

		ofm_data_current.key = no_key;
		ofm_data_current.absvsqr = (abs_vsqr <= 255) ? abs_vsqr : 255;
		ofm_data_current.state = ofm_global.current_ofm_state;
		ofm_data_current.stimer = ofm_global.ofm_state_timer;
#endif

	}
	else if(gv_OFM_dev_ver == OFM_DEV_VD5376){

		result = i2c_smbus_read_word_data(ofm_global.client,0x21);
		if(result < 0) {
			ofm_global.current_ofm_state = e_ofm_uncovered;
		}
		x = (result&0xff);
		y = ((result>>8)&0xff);

		printk(KERN_INFO "[OFM] ofm_motion_func  x=%d  y=%d\n",x,y);

		result = i2c_smbus_read_word_data(ofm_global.client,0x31);
		features = ((result&0xff)<<8)|((result>>8)&0xff);
	}
	else{
		printk(KERN_INFO "[OFM] %s:%d  Un-initialized OFM device.\n",__FUNCTION__,__LINE__);
	}

	//printk(KERN_INFO "[OFM] COVER=%d\n",features);
	if(features < COVER_THRESHOLD || expotime < EXPOTIME_SUN) {  // UNCOVERED
		//printk(KERN_INFO "[OFM] [UNCOVER] %d\n",result);
		if(((ofm_global.current_ofm_state == e_ofm_counting_covered_not_fired) || (ofm_global.current_ofm_state == e_ofm_second_tap)) &&
			(ofm_global.ofm_state_timer >= MIN_TAP_SELECT) &&
			(ofm_global.ofm_state_timer <= MAX_TAP_SELECT) &&
			(abs_vsqr < MAX_SELECT_MOVE*MAX_SELECT_MOVE)) {
			if(ofm_global.current_ofm_state == e_ofm_counting_covered_not_fired) {
				//printk(KERN_INFO "[OFM] 1ST TAP DETECTED state_timer=%d\n",ofm_global.ofm_state_timer);
				ofm_global.current_ofm_state = e_ofm_first_tap;
				ofm_global.ofm_state_timer = 0;
#ifdef OFM_9AXIS_EN
				ofm_data_current.sec_event.start_sec = ts.tv_sec;
				ofm_data_current.sec_event.start_ns = ts.tv_nsec;
#endif
			} else { //e_ofm_second_tap
				//printk(KERN_INFO "[OFM] ENTER_KEY\n");
				ofm_trigger_key(enter_key);
#ifdef OFM_DEBUG_EN
				OfmTestData[gv_Test_Counter-1]=200; //200 enter_key
				ofm_image_dump(9);
#endif
				ofm_global.current_ofm_state = e_ofm_uncovered;
			}
		} else {
			if((ofm_global.current_ofm_state == e_ofm_first_tap) &&
				(ofm_global.ofm_state_timer <= MAX_TAP_TAP_DELAY)) {
				ofm_global.ofm_state_timer++;
			} else {
				//printk(KERN_INFO "[OFM] UNCOVER e_ofm_uncovered state_timer=%d\n",ofm_global.ofm_state_timer);
				ofm_global.current_ofm_state = e_ofm_uncovered;
			}
		}
	}
	else if(ofm_global.current_ofm_state == e_ofm_first_tap) {
		if(ofm_global.ofm_state_timer < MIN_TAP_TAP_DELAY) {
//			printk(KERN_INFO "IDLE %d\n",ofm_global.ofm_state_timer);
			ofm_global.current_ofm_state = e_ofm_covered_idle;
		} else {
			//printk(KERN_INFO "[OFM] 2ND TAP DETECTED state_timer=%d\n",ofm_global.ofm_state_timer);
		    ofm_global.current_ofm_state = e_ofm_second_tap;
		    ofm_global.ofm_state_timer = 0;
		    sum_x = 0;
		    sum_y = 0;
		    abs_vsqr = 0;
#ifdef OFM_9AXIS_EN
			ofm_data_current.sec_event.end_sec = ts.tv_sec;
			ofm_data_current.sec_event.end_ns = ts.tv_nsec;
#endif
		}
	}
	else if((ofm_global.current_ofm_state == e_ofm_counting_covered_not_fired) || (ofm_global.current_ofm_state == e_ofm_second_tap)) {
		if(ofm_global.ofm_state_timer == 0) {
//	       printk(KERN_INFO "[COVER] %d\n",result);
		}
		ofm_counting_covered_not_fired(work, x, y);
	} else if(ofm_global.current_ofm_state == e_ofm_counting_covered_has_fired) {
		ofm_counting_covered_has_fired(work);
	} else if(ofm_global.current_ofm_state == e_ofm_covered_scrolling) {
		ofm_covered_scrolling(work);
	}

	if(ofm_global.current_ofm_state == e_ofm_uncovered) {
		enable_irq(ofm_global.ofm_motion->irq);
	} else {
		schedule_delayed_work(&ofm_global.work, 1);
	}

#ifdef OFM_9AXIS_EN
	/**
	printk(KERN_INFO "[OFM] ofm_motion_func time sec=%ld ns=%ld key=%d state=%d stimer=%d\n",
			ofm_data_current.time_sec, ofm_data_current.time_ns,ofm_data_current.key,ofm_data_current.state,ofm_data_current.stimer);
	printk(KERN_INFO "[OFM] ofm_motion_func feature=%d expotime=%d\n",ofm_data_current.feature, ofm_data_current.expotime);
	**/
#endif

	return;
}

static int ofm_i2c_write(struct i2c_client *client, u_int8_t index, u_int8_t data)
{
	int result;
	u_int8_t buf[2] = {index , data};

	result= i2c_master_send(client, buf, 2);
	//printk("[OFM] %s(0x%02x, 0x%02x), %d, ret : %d\n", __func__, index, data, __LINE__, result );
	if(result>=0)
		return 0;

	printk("[OFM] ERROR i2c send!!!index(%x) data(%x) return (%x)\n",index,data,result);
	return result;
}


static void ofm_enable_irq(void)
{
//	enable_irq(ofm_global.client->irq);
//	printk("[OFM] %s: Enabling irq %d \n",	__FUNCTION__, ofm_global.client->irq);

	enable_irq(ofm_global.ofm_motion->irq);
	printk("[OFM] %s: Enabling motion irq %d\n", __FUNCTION__, ofm_global.ofm_motion->irq);

	enable_irq(ofm_global.ofm_left->irq);
	printk("[OFM] %s: Enabling dome key irq %d\n", __FUNCTION__, ofm_global.ofm_left->irq);
}

static void ofm_disable_irq(void)
{
//	if(atomic_read(&ofm_global.enable)){
//		printk("[OFM] %s: enable flag set to DISABLE\n", __FUNCTION__);
//		atomic_set(&ofm_global.enable, 0);
//	}

//	disable_irq(ofm_global.client->irq);
//	printk("[OFM] %s: Disabling irq %d\n", __FUNCTION__, ofm_global.client->irq);

	disable_irq(ofm_global.ofm_motion->irq);
	printk("[OFM] %s: Disabling motion irq %d\n", __FUNCTION__, ofm_global.ofm_motion->irq);

	disable_irq(ofm_global.ofm_left->irq);
	printk("[OFM] %s: Disabling dome key irq %d\n", __FUNCTION__, ofm_global.ofm_left->irq);
}

//usage : "cat ofmgetlevel"
static ssize_t ofm_level_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;
	printk(KERN_INFO "[OFM] access ofm_level_read!!!\n");

	ofm_reg_dump();

	//count = sprintf(buf,"%d,%d\n", gv_TestSum_DeltaX, gv_TestSum_DeltaY);
	count = sprintf(buf,"%d,%d\n", ofm_global.x_level, ofm_global.y_level);
	return count;
}

//usage : "vi ofmsetlevel" -> insert value (0~8) -> :wq(save&exit)
static ssize_t ofm_level_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	unsigned long level;

	rc = strict_strtoul(buf, 0, &level);
	if (rc)
	{
		printk(KERN_INFO "[OFM] ofm_level_store : rc = %d\n", rc);
		return rc;
	}

	mutex_lock(&ofm_global.ops_lock);
	printk(KERN_INFO "[OFM] write ofm_level_store : Level = %lu\n", level);
	if((level >= 0) && (level < OFM_SENSITIVITY_MAX_LEVEL))
	{
		ofm_global.x_level = level;
		ofm_global.y_level = level;
	}

	rc = count;
	mutex_unlock(&ofm_global.ops_lock);

	return rc;
}

//usage # echo 0 > ofmsetonoff
//usage # echo 1 > ofmsetonoff
static ssize_t ofm_action_onoff(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	unsigned long on_off;

	rc = strict_strtoul(buf, 0, &on_off);
	if (rc)
	{
		printk(KERN_INFO "[OFM] ofm_action_store : rc = %d\n", rc);
		return rc;
	}

	mutex_lock(&ofm_global.ops_lock);
	printk(KERN_INFO "[OFM] write ofm_action_store : on/off = %lu\n", on_off);

	ofm_ctrl_power(on_off);

	rc = count;
	mutex_unlock(&ofm_global.ops_lock);

	return rc;
}

//usage # cd /sys/devices/platform/omap/omap_i2c.4/i2c-4/4-0053/input/ofm_driver/
//usage # echo 0 > ofmsetstandby
//usage # echo 1 > ofmsetstandby
static ssize_t ofm_standby_onoff(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	unsigned long on_off;

	rc = strict_strtoul(buf, 0, &on_off);
	if (rc)
	{
		printk(KERN_INFO "[OFM] ofm_standby_onoff : rc = %d\n", rc);
		return rc;
	}

	mutex_lock(&ofm_global.ops_lock);
	printk(KERN_INFO "[OFM] %s : on/off = %lu\n",__FUNCTION__, on_off);

	gpio_set_value_cansleep(ofm_global.ofm_standby->pin, on_off);
	mdelay(1); //wating for gpio set stable 1ms

	rc = count;
	mutex_unlock(&ofm_global.ops_lock);

	return rc;
}

//usage # cd /sys/devices/platform/omap/omap_i2c.4/i2c-4/4-0053/input/ofm_driver/
//usage # echo 0 > ofmsetpwrdwn
//usage # echo 1 > ofmsetpwrdwn
static ssize_t ofm_pwrdwn_onoff(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	unsigned long on_off;

	rc = strict_strtoul(buf, 0, &on_off);
	if (rc)
	{
		printk(KERN_INFO "[OFM] ofm_pwrdwn_onoff : rc = %d\n", rc);
		return rc;
	}

	mutex_lock(&ofm_global.ops_lock);
	printk(KERN_INFO "[OFM] %s : on/off = %lu\n",__FUNCTION__, on_off);

	gpio_set_value_cansleep(ofm_global.ofm_pwrdwn->pin, on_off);
	mdelay(1); //wating for gpio set stable 1ms

	rc = count;
	mutex_unlock(&ofm_global.ops_lock);

	return rc;
}


//AMF(Automatic Movement Filter) Patch
//Ref. VD5377 Porting Guide Rev 2.0.0

static void ofm_AMF_patch(void)
{

	int8_t tempI2CBUF1[3] = {0x02, 0x02, 0x88};
	int8_t tempI2CBUF2[42] = {
				0x30, 0x09, 0x1C, 0xE5, 0x18, 0x70, 0x16, 0xC2, 0x09, 0xE5,
				0x1C, 0x54, 0x0F, 0xC4, 0x54, 0xF0, 0xFF, 0x90, 0x00, 0x17,
				0xE0, 0x54, 0x0F, 0x4F, 0xF0, 0xC2, 0x08, 0x80, 0x02, 0x15,
				0x18, 0x90, 0x00, 0x17, 0xE0, 0x44, 0x08, 0xF0, 0x02, 0x05, 0x30, 0x22
	};

	int8_t tempI2CBUF3[3] = {0x02, 0x7D, 0x00};
	int8_t tempI2CBUF4[3] = {0x05, 0x29, 0x03};

	//patch start
	int i;
	uint8_t major, minor;

	//VD5377 rev 1.0 (0.0)
	//VD5377 rev 2.0 (1.0)
	major = i2c_smbus_read_byte_data(ofm_global.client,0x00);
	minor = i2c_smbus_read_byte_data(ofm_global.client,0x01);

	if( (major == 1) && (minor == 0) )
	{
		ofm_i2c_write(ofm_global.client,0x14,0x0d);
		ofm_i2c_write(ofm_global.client,0x6f,0x02);

		for(i = 0; i < 3; i++ ){
			ofm_i2c_write(ofm_global.client,0xa0+i,tempI2CBUF1[ i ]);
		}

		for(i = 0 ; i < 42;i++ ){
			ofm_i2c_write(ofm_global.client,0xa3+i,tempI2CBUF2[ i ]);
		}

		for(i = 0 ; i < 3;i++ ){
			ofm_i2c_write(ofm_global.client,0x70+i,tempI2CBUF3[ i ]);
		}
		for(i = 0 ; i < 3;i++ ){
			ofm_i2c_write(ofm_global.client,0x73+i,tempI2CBUF4[ i ]);
		}
		ofm_i2c_write(ofm_global.client,0x6f,0x0d);
	}//patch end

	//Automatic movement filter setting
	ofm_i2c_write(ofm_global.client,0x8d,0x0f);
	ofm_i2c_write(ofm_global.client,0x8e,0x41);
}


static struct device_attribute dev_attr_set_level = __ATTR(ofmsetlevel, 0664 , NULL, ofm_level_store);
static struct device_attribute dev_attr_get_level = __ATTR(ofmgetlevel, 0444, ofm_level_read, NULL);
static struct device_attribute dev_attr_ofm_onoff = __ATTR(ofmsetonoff, 0664 , NULL, ofm_action_onoff);
static struct device_attribute dev_attr_ofm_standby = __ATTR(ofmsetstandby, 0664 , NULL, ofm_standby_onoff);
static struct device_attribute dev_attr_ofm_pwrdwn  = __ATTR(ofmsetpwrdwn, 0664 , NULL, ofm_pwrdwn_onoff);

static struct attribute *ofm_sysfs_attrs[] = {
	&dev_attr_set_level.attr,
	&dev_attr_get_level.attr,
	&dev_attr_ofm_onoff.attr,
	&dev_attr_ofm_standby.attr,
	&dev_attr_ofm_pwrdwn.attr,
	NULL
};

static struct attribute_group ofm_attribute_group = {
	.attrs = ofm_sysfs_attrs,
};

static int ofm_open(struct inode* inode, struct file* file)
{
	//misc? driver require this for file_operations for some reasons --
	return 0;
}

//# cat /dev/ofm_driver > /ofmImage.raw //8-bit raw image
static ssize_t ofm_read(struct file *filp, char __user *buff, size_t count,loff_t *f_pos)
{
	int ret = 0;
	int TXcount;
	char *tmp;
	mutex_lock(&ofm_global.ops_lock);

	TXcount = gv_TXcount_size;

	printk(KERN_INFO "[OFM] %s:%d  TXcount=%d count=%d f_pos=%d \n",__FUNCTION__,__LINE__,TXcount,count,(int)*f_pos);

	if(ofm_global.ofm_img_data == NULL || TXcount==0){ //no data to transfer
		printk(KERN_INFO "[OFM] %s:%d  TXcount=%d\n",__FUNCTION__,__LINE__,TXcount);


		if(ofm_global.ofm_img_data == NULL)
			printk(KERN_INFO "[OFM] %s:%d  ofm_img_data=NULL\n",__FUNCTION__,__LINE__);

		ret = 0;
		goto read_done;
	}
	tmp = ofm_global.ofm_img_data;

	if((*f_pos + count) > TXcount)
	    		count = (TXcount - *f_pos);

	if(count>0)
	{
		ret =(copy_to_user(buff, (tmp+*f_pos), count));
		printk(KERN_INFO "[OFM] %s:%d  count=%d ret=%d\n",__FUNCTION__,__LINE__,count, ret);
		if(ret>0)
		{
			printk(KERN_INFO "[OFM] %s:%d  copy_to_user failed!\n",__FUNCTION__,__LINE__);
			count = 0;
			*f_pos = 0;
		}
		*f_pos += count;
		ret=count;
	}
	else
	{
		ret = 0;
		count = 0;
		*f_pos = 0;
	}

read_done:
	kfree(ofm_global.ofm_img_data);
	ofm_global.ofm_img_data = NULL;
	gv_TXcount_size = 0;
	mutex_unlock(&ofm_global.ops_lock);

	return ret;
}

static ssize_t ofm_write(struct file *filp, const char __user *buff, size_t count,loff_t *f_pos)
{
	int ret = 0;
	char *tmp;
	unsigned long mode = -1;

	tmp = kmalloc(count, GFP_KERNEL);
	if(tmp == NULL){
		ret=-ENOMEM;
		goto write_done;
	}

	ret=copy_from_user(tmp, buff, count);
	printk(KERN_INFO "[%s:%u] copy_from_user ret=%d count=%d\n",__FUNCTION__,__LINE__,(int)ret,(int)count);
	if(ret==0)
	{
		mode = simple_strtoul(tmp, NULL, 0);

		printk(KERN_INFO "[%s:%u] mode=%d\n",__FUNCTION__,__LINE__,(int)mode);

		switch(mode)
		{
			case 7:
#ifdef OFM_DEBUG_EN
				ret= ofm_test_data_dump();	//# echo 7 > /dev/ofm_driver
#endif
			break;
			case 8:  //debug grey level
			case 9:  //CDS frame
			case 10: //exposed frame
			case 11: //black frame
#ifdef OFM_DEBUG_EN
				ret= ofm_image_dump((int)mode);	//# echo 8 > /dev/ofm_driver
#endif
			break;
			default:
				ret=-EINVAL;
			break;
		}
	}
	else
		ret= -EFAULT;

write_done:
	kfree(tmp);

	return -EINVAL;
}

static long ofm_ioctl(struct file *filp,unsigned int cmd, unsigned long arg)
{
	int ret=0;
	unsigned long count=0;
	u8 *tmp;

	ret=mutex_lock_interruptible(&ofm_global.ops_lock);
	if(ret)
	{
		printk("%s: mutex_lock_interruptible returned %d\n",__func__, ret);
		goto ioctl_done;
	}

	switch(cmd)
	{
		case OFM_IOC_READ_EVENT:
#ifdef OFM_9AXIS_EN
			count= sizeof(ofm_data_kevent);
			tmp = (u8 *)&ofm_data_kevent;
#endif
		break;
		default:
			ret=-EINVAL;
		break;
	}

	if(count){
		ret = copy_to_user((u8 __user *)arg, tmp, count);
	}

ioctl_done:
	mutex_unlock(&ofm_global.ops_lock);
	return ret;
}

static unsigned int ofm_poll(struct file *filp, struct poll_table_struct *wait)
{
	poll_wait(filp, &ofm_global.signal, wait);
	if(atomic_read(&ofm_global.interrupt_state)==INTR_EVENT){
		atomic_set(&ofm_global.interrupt_state, NO_INTR_EVENT);
		return (POLLIN | POLLRDNORM); /* readable */
	}
	return 0;
}

/* File operations on ofm device */
static const struct file_operations ofm_fops =
{
	.owner                  = THIS_MODULE,
	.open                   = ofm_open,
	.read                   = ofm_read,
	.write                  = ofm_write,
	.poll                   = ofm_poll,
	.unlocked_ioctl         = ofm_ioctl,
};

static int __devinit ofm_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//struct ofm	*ofm;
	int result;
	int	key, code; //add by ybchoi 2011.06.22

	printk("[OFM] %s, %d\n", __func__, __LINE__ );
	printk("[OFM] I2C device probe  \n");

	ofm_global.client = client;
	i2c_set_clientdata(client, &ofm_global);
	printk("[OFM] slave address 0x%02x\n", client->addr);

	ofm_global.input_dev = input_allocate_device();
	if (ofm_global.input_dev == NULL)
	{
		dev_err(&client->dev, "[OFM] Failed to allocate input device.\n");
		result = -ENOMEM;
		goto err_input_allocate;
	}

	//ofm = kzalloc(sizeof(struct ofm), GFP_KERNEL);
	//if(!ofm)
	//{
	//	dev_err(&client->dev, "[OFM] failed to allocate driver data\n");
	//	result = -ENOMEM;
	//	goto err_kzalloc;
	//}

	ofm_global.input_dev->name = DRIVER_NAME;
    ofm_global.input_dev->dev.init_name = DRIVER_NAME;
	ofm_global.input_dev->id.bustype = BUS_I2C;
	ofm_global.input_dev->dev.parent = &client->dev;
	ofm_global.input_dev->phys = "ofm";
	ofm_global.input_dev->id.vendor = 0x0001;//0xDEAD;
	ofm_global.input_dev->id.product = 0x0001;//0xBEEF;
	ofm_global.input_dev->id.version = 0x0100;//0x01;

	/* Set event key bits */
	printk("[OFM] %s : Set event key bits\n", __func__);
	set_bit(EV_KEY, ofm_global.input_dev->evbit);
	/* Allocation key event bits */
	for(key = 0; key < MAX_KEYPAD_CNT; key++){
		code = ofm_keypad_keycode_map[key];
		if(code<=0)
			continue;
		set_bit(code&KEY_MAX, ofm_global.input_dev->keybit);
	}

	mutex_init(&ofm_global.ops_lock);
	result = input_register_device(ofm_global.input_dev);
	if (result){
		dev_err(&client->dev, "[OFM] %s(%s): Unable to register %s input device\n", __FILE__, __FUNCTION__, ofm_global.input_dev->name);
		goto err_input_reg;
	}

	/*sys_fs*/
	result = sysfs_create_group(&ofm_global.input_dev->dev.kobj,&ofm_attribute_group);
	if (result) {
		printk("[OFM] Creating sysfs attribute group failed");
		goto sysfs_create_err;
	}

	// Register misc device
	ofm_global.misc_dev.minor = MISC_DYNAMIC_MINOR;
	ofm_global.misc_dev.name = DRIVER_NAME;
	ofm_global.misc_dev.fops = &ofm_fops;
	result = misc_register(&ofm_global.misc_dev);
	if (result < 0)
	{
		printk (KERN_ERR "[OFM] Register misc device\n");
		goto sysfs_create_err;
	}
	ofm_switch_reg = regulator_get(&ofm_global.input_dev->dev, "ofm_switch" );

	if (IS_ERR(ofm_switch_reg)) {
		result = PTR_ERR(ofm_switch_reg);
		printk(KERN_ERR "couldn't get OFM (Finger Nav) Switch regulator %d\n", result);
		goto regulator_err;
	}
	regulator_enable(ofm_switch_reg);

	/*Default Sensitivity Level*/
	ofm_global.x_level = OFM_SENSITIVITY_X_DEFAULT;
	ofm_global.y_level = OFM_SENSITIVITY_Y_DEFAULT;
	ofm_global.ofm_img_data = NULL;

#ifdef OFM_DEBUG_EN
	memset(OfmTestData,0,OFM_TEST_DATA_SIZE);
#endif

#ifdef OFM_9AXIS_EN
	memset(&ofm_data_current,0,sizeof(ofm_data_current));
	memset(&ofm_data_kevent,0,sizeof(ofm_data_kevent));
#endif

    ofm_global.current_ofm_state = e_ofm_uncovered;
	ofm_global.ofm_state_timer = 0;

	printk("[OFM] %s : Set hard click timer\n", __func__);
	/*set dome key pressing start time check timer*/
	hrtimer_init(&ofm_global.timer_click, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ofm_global.dclick_time = ns_to_ktime(DCLICK_TIME);
	ofm_global.timer_click.function =ofm_timer_click_func;

	/*set dome key long press check timer*/
	hrtimer_init(&ofm_global.timer_long, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ofm_global.lclick_time = ns_to_ktime(LCLICK_TIME);
	ofm_global.timer_long.function =ofm_timer_long_func;

	/*set summing data clear check timer*/
	hrtimer_init(&ofm_global.timer_clear, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ofm_global.clear_time = ns_to_ktime(CLEAR_TIME);
	ofm_global.timer_clear.function =ofm_timer_clear_func;

	//gpio_request for standby and pwrdwn pins are defined in ../arch/arm/mach-omap2/board-4430jet.c

	//set standby pin
	ofm_global.ofm_standby = &ofm_standby;
	result = gpio_cansleep(ofm_global.ofm_standby->pin);
	if (result) {
		printk(KERN_INFO "[OFM] %s : Key:GPIO %d standby cansleep failed (result=%d)\n",__FUNCTION__, ofm_global.ofm_standby->pin, result);
		goto err_reg_gpio;
	}
	gpio_set_value_cansleep(ofm_global.ofm_standby->pin, 0); //standby_pin set to low
	mdelay(1);
	printk(KERN_INFO "[OFM] %s : Key:GPIO %d standby Success\n",__FUNCTION__, ofm_global.ofm_standby->pin);

	//set pwrdwn pin
	ofm_global.ofm_pwrdwn = &ofm_pwrdwn;
	result = gpio_cansleep(ofm_global.ofm_pwrdwn->pin);
	if (result) {
		printk(KERN_INFO "[OFM] %s : Key:GPIO %d pwrdwn cansleep failed (result=%d)\n",__FUNCTION__, ofm_global.ofm_pwrdwn->pin, result);
		goto err_reg_gpio;
	}
	gpio_set_value_cansleep(ofm_global.ofm_pwrdwn->pin, 0); //pwrdwn_pin set to low
	mdelay(1);
	printk(KERN_INFO "[OFM] %s : Key:GPIO %d pwrdwn Success\n",__FUNCTION__, ofm_global.ofm_pwrdwn->pin);

	// standby and pwrdwn pins must be initialized before reading OFM device version
	// work queue and motion interrupts only be initialized for a valid device
	result = ofm_get_device_ver();
	if (result == 0) {

		printk(KERN_INFO "[OFM] %s : ofm_device_ver=%d\n",__FUNCTION__,gv_OFM_dev_ver);
		g_bIsValidOfmDevice = 1;

		atomic_set(&ofm_global.interrupt_state, NO_INTR_EVENT);
		init_waitqueue_head(&ofm_global.signal);

		/* !!!! work queue must be init before motion interrupt define. !!!!*/
		/*read x,y data by polling*/
		printk("[OFM] %s : INIT DELAYED WORK\n", __func__);
		INIT_DELAYED_WORK(&ofm_global.work, ofm_motion_func);
		schedule_delayed_work(&ofm_global.work, 1);

		/*set motion pin interrupt*/
		printk("[OFM] %s : Set motion pin interrupt\n", __func__);
		ofm_motion.irq = gpio_to_irq(ofm_motion.pin);

		ofm_global.ofm_motion= &ofm_motion;
		result = request_irq (ofm_global.ofm_motion->irq,ofm_motion_event,IRQF_TRIGGER_RISING,ofm_global.ofm_motion->name,&ofm_global);
		if (result) {
			printk(KERN_INFO "[OFM] %s : Motion:Request IRQ  %d  failed\n",__FUNCTION__, ofm_global.ofm_motion->irq);
			goto err_reg_irq;
		}
		printk(KERN_INFO "[OFM] %s : Motion:Request IRQ  %d  Success\n",__FUNCTION__, ofm_global.ofm_motion->irq);

		//set dome key interrupt
		printk("[OFM] %s : Set dome key interrupt\n", __func__);
		ofm_left.irq = gpio_to_irq(ofm_left.pin);

		ofm_global.ofm_left = &ofm_left;
		result = request_irq (ofm_global.ofm_left->irq,ofm_left_event,IRQF_DISABLED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,ofm_global.ofm_left->name,&ofm_global);
		if (result) {
			printk(KERN_INFO "[OFM] %s : Key:Request IRQ  %d  failed\n",__FUNCTION__, ofm_global.ofm_left->irq);
			goto err_reg_irq;
		}
		printk(KERN_INFO "[OFM] %s : Key:Request IRQ  %d  Success\n",__FUNCTION__, ofm_global.ofm_left->irq);

		ofm_disable_irq();
		ofm_ctrl_power(OFM_ON);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ofm_global.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
	ofm_global.early_suspend.suspend = ofm_early_suspend;
	ofm_global.early_suspend.resume = ofm_late_resume;
	register_early_suspend(&ofm_global.early_suspend);
#endif

	printk("[OFM] %s : probe complite!!!!!!\n", __func__);

	return 0;

	err_input_allocate:
		i2c_set_clientdata(client, NULL);
		printk("[OFM] I2C device probe error(%d) \n", result);
        regulator_err:
		sysfs_remove_group(&(ofm_global.input_dev->dev.kobj), &ofm_attribute_group);
	sysfs_create_err:
		input_unregister_device(ofm_global.input_dev);
//		misc_deregister(&(ofm_global.input_dev->dev));
	err_input_reg:
		printk("[OFM] input register device error (%d) \n",result);
		input_free_device(ofm_global.input_dev);
		//kfree(&ofm_global);
		dev_set_drvdata(&client->dev, NULL);
		printk("[OFM] I2C device probe error (%d) \n",result);
	err_reg_irq:
		//ofm_global.input_dev = NULL; /* so we dont try to free it below */
	err_reg_gpio:

	return result;
}

static __devexit int ofm_i2c_remove(struct i2c_client *client)
{
	struct ofm *ofm = i2c_get_clientdata(client);
	unregister_early_suspend(&ofm_global.early_suspend);
	input_unregister_device(ofm_global.input_dev);
	destroy_workqueue(ofm_workqueue);
	kfree(ofm);
	return 0;
}

static int ofm_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct ofm *ofm = i2c_get_clientdata(client);
	printk(KERN_INFO "[OFM] %s+\n", __func__ );
	ret = cancel_delayed_work_sync(&ofm ->work);
	if (ret) /* if work was pending disable-count is now 2 */
	{
		printk(KERN_INFO "[OFM] %s : enable irq ret=%d\n",__func__, ret);
	}
	ofm_ctrl_power(OFM_OFF);
	printk(KERN_INFO "[OFM] %s-\n", __func__ );
	return 0;
}

static int ofm_resume(struct i2c_client *client)
{
	printk(KERN_INFO "[OFM] %s+\n", __func__ );
	ofm_ctrl_power(OFM_ON);
	printk(KERN_INFO "[OFM] %s-\n", __func__ );
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ofm_early_suspend(struct early_suspend *h)
{
	ofm_suspend(ofm_global.client, PMSG_SUSPEND);
}

static void ofm_late_resume(struct early_suspend *h)
{
	ofm_resume(ofm_global.client);
}
#endif

static const struct i2c_device_id ofm_i2c_id[]={
	{DRIVER_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ofm_i2c_id);

static struct i2c_driver ofm_i2c_driver = {
	.probe	= ofm_i2c_probe,
	.remove	= ofm_i2c_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = ofm_suspend,
	.resume = ofm_resume,
#endif
	.id_table = ofm_i2c_id,
    .driver =
    {
        .owner = THIS_MODULE,
        .name = DRIVER_NAME,
    },
};

static int __init ofm_init(void)
{
	int ret;
	printk("[OFM] %s, %d\n", __func__, __LINE__ );

#if 0 //for samsung dd
	gpio_request(OTP_POWER_EN, "OTP_POWER_EN"); //power_down pin
    gpio_direction_output(OTP_POWER_EN, 1);
    bcm_gpio_pull_up(OTP_POWER_EN, true);
    bcm_gpio_pull_up_down_enable(OTP_POWER_EN, true);
    mdelay(50);

    gpio_request(OTP_SHDN, "OTP_SHDN"); //motion pin
    gpio_direction_output(OTP_SHDN, 0);
    mdelay(50);
#endif
	ret = i2c_add_driver(&ofm_i2c_driver);
	if(ret!=0)
		printk("[OFM] I2C device init Faild! return(%d) \n",  ret);
	printk("[OFM] I2C device init Sucess\n");
	return ret;
}

static void __exit ofm_exit(void)
{
	printk("[OFM] %s, %d\n", __func__, __LINE__ );
	i2c_del_driver(&ofm_i2c_driver);
}
module_init(ofm_init);
module_exit(ofm_exit);
MODULE_DESCRIPTION("OFM Device Driver");
MODULE_AUTHOR("Partron InputDevice Team(ybchoi@partron.co.kr)");
MODULE_LICENSE("GPL");

