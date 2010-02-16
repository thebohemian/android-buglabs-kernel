/*
 * 	bmi_lcd.c
 *
 * 	BMI LCD device driver
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * Include files
 */

#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/system.h>
#include <mach/mxc_i2c.h>
#include <mach/mx31bug_cpld.h>
#include <linux/bmi.h>
#include <linux/bmi/bmi-control.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/bmi/bmi_lcd.h>
#include <mach/ipu.h>

#undef ACCELEROMETER
#define ACCELEROMETER

#ifdef ACCELEROMETER
#include "acc.h"
#endif //ACCELEROMETER

#define DEBUG
#undef DEBUG

#define BMILCD_VERSION		"1.2"		// driver version
#define BUF_MAX_SIZE		0x20		// spi buffer size
#define WORK_DELAY		(1)		// interrupt work handler delay
#define DEBOUNCE		10		// touch screen debounce
#define X_PLATE			400		// touch screen X plate resistance //pjg - This is not the correct value
#define BMI_SLOT_NUM		(4)		// number of BMI slots
#define MAX_STRG		(40)		// Max string buffer size

#define	VSYNC_DISABLE		0x0
#define	VSYNC_ENABLE		0x1

	// lcd
struct lcd_interface {
	char			lcd_type[MAX_STRG];	// text description of LCD type
	u8			suspended;		// power management state
	u8			rotation;		// screen rotation
	u8			disp;			// display number (DISP0 or DISP1)
	u8			addr_mode;		// display addressing mode
	u8			vsync_mode;		// VSYNC signal enable (VSYNC_ENABLE | VSYNC_DISABLE)
	u8			bus_if_type;		// bus type (XY | FullWoBE | FullWithBE)
	ipu_adc_sig_cfg_t	adc_sig;		// IPU ADC set-up parameters
	ipu_di_signal_cfg_t	di_sig;			// IPU DI set-up parameters
}; 

static struct lcd_interface s320x240_lcd_interface = {
	.lcd_type = "MXCFB_SHARP_320X240",
	.suspended = 0,
	.rotation = IPU_ROTATE_NONE,
	.disp = DISP0,
	.vsync_mode = VSYNC_DISABLE,
	.bus_if_type = XY,
	.adc_sig = { 0, 0, 0, 0, 0, 0, 0, 0, IPU_ADC_BURST_WCS, IPU_ADC_IFC_MODE_SYS80_TYPE2,
			16, 0, 0, IPU_ADC_SER_NO_RW },
	.di_sig = { 0,0,0,0,0,0,0,0 },		//pjg - reserved for multiple LCD driver
};


struct bmi_lcd;

struct bmi_lcd_ops {
	void *(*config) (int disp);				// LCD configuration/initialization
	void *(*reset) (int slot);				// LCD reset
	int *(*suspend) (struct bmi_lcd *blcd);			// power management
	int *(*resume) (struct bmi_lcd *blcd);			// power management
	int *(*disp_on) (int disp);				// display on
	int *(*disp_off) (int disp);				// display off
	int (*activate) (struct bmi_lcd *lcd, int slot);	// enable LCD backlight, touchscreen, accelerometer, ...
	int (*deactivate) (struct bmi_lcd *lcd, int slot);	// disable LCD backlight, touchscreen, accelerometer, ...
};


struct bmi_lcd {
	struct lcd_interface interface;		// pointer to this struct is returned by config()
	struct bmi_lcd_ops lcd_ops;		// function pointers
};


int register_bmi_lcd(struct bmi_lcd *blcd, int slot);
int unregister_bmi_lcd(struct bmi_lcd *blcd, int slot);

	// private device structure
struct pbmi_lcd
{
	int			open_flag;				// force single open
	unsigned int		lcd_cnt;				// number of LCD's present
	unsigned int		active;					// indication of LCD presence
	unsigned int		activated[BMI_SLOT_NUM];		// indication of LCD presence

	struct bmi_lcd 		*blcd[BMI_SLOT_NUM];			// BMI LCD structure - placeholder for multiple display types
	struct bmi_device	*bdev[BMI_SLOT_NUM];			// BMI device per slot
	unsigned int		interrupt[BMI_SLOT_NUM];		// input device interrupt handlers
	char			int_name[MAX_STRG];			// interrupt name

	struct input_dev 	*input_dev[BMI_TS_NUM];			// input device (touch screen and accelerometer)
	struct timer_list	timer[BMI_SLOT_NUM];			// touch timer
	
  int pen_down[BMI_SLOT_NUM];
  int scount[BMI_SLOT_NUM];

	struct spi_device	*spi[BMI_SLOT_NUM];			// touch screen device interface
	struct semaphore	sem[BMI_SLOT_NUM];			// spi semaphore
	char			rbuf[BMI_SLOT_NUM][BUF_MAX_SIZE];	// spi read buffer
	char			wbuf[BMI_SLOT_NUM][BUF_MAX_SIZE];	// spi write buffer

#ifdef ACCELEROMETER
  struct acc_dev             acc[BMI_SLOT_NUM];
#endif
};

static struct pbmi_lcd pbmi_lcd;	// LCD device sructure

/*
 * 	BMI set up
 */

	// BMI device ID table
static struct bmi_device_id bmi_lcd_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_LCD_SHARP_320X240, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },					  /* terminate list */
};

MODULE_DEVICE_TABLE(bmi, bmi_lcd_tbl);

/*printk(KERN_INFO "MDT: 0x%x\n", __mod_bmi_device_table);*/

int	bmi_lcd_probe(struct bmi_device *bdev);
void	bmi_lcd_remove(struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_lcd_driver = 
{
	.name = "bmi_lcd", 
	.id_table = bmi_lcd_tbl, 
	.probe   = bmi_lcd_probe, 
	.remove  = bmi_lcd_remove, 
};

//Accelerometer driver structure


/*
 * 	I2C set up
 */

	// I2C Slave Address
#define BMI_IOX_I2C_ADDRESS	0x71	// 7-bit address
#define BMI_ACC_I2C_ADDRESS	0x17	// 7-bit address

	// I2C IOX register addresses
#define IOX_INPUT_REG		0x0	// IOX input data register
#define IOX_OUTPUT_REG		0x1	// IOX output data register
#define IOX_POLARITY_REG	0x2	// IOX polarity data register
#define IOX_CONTROL		0x3	// IOX direction control register
#define IOX_B1			(0)	// bit 0 - backlight control
#define IOX_A1_A2		(1)	// bit 1 - backlight control
#define IOX_ACC_RST_N		(2)	// bit 2 - acceleromter reset
#define IOX_VSYNC_EN_N		(3)	// bit 3 - VSYNC output buffer enable
#define IOX_LCD_RST_N		(4)	// bit 4 - LCD reset
#define IOX_SERDES_PD_N		(5)	// bit 5 - SERDES power down
#define IOX_X_INT		(6)	// bit 6 - accelerometer interrupt
#define IOX_Y_INT		(7)	// bit 7 - accelerometer interrupt

	// I2C ACC register addresses - OKI
#define ACC_PAGESEL			0x1E	// device ready status
	// page 0
#define ACC_DVRST			0x01	// device reset
	#define ACC_DVRST_RST		0x3C	// device reset
	#define ACC_DVRST_EN		0xC3	// device enable
#define ACC_PDWN			0x02	// osc power down
	#define ACC_PWDN_RST		0x01	// device reset
	#define ACC_PWDN_EN		0x00	// device enable
#define ACC_CTRL0			0x03	// control 0
	#define ACC_CTRL0_CTSTR		0x40	// control 0 - temp sensor
	#define ACC_CTRL0_CGSTRNC	0x08	// control 0 - 3-axis/no tilt
	#define ACC_CTRL0_CGSTRC	0x04	// control 0 - 3-axis/tilt
	#define ACC_CTRL0_CGAUTO	0x01	// control 0 - auto
#define ACC_MODE0			0x05	// control 0
	#define ACC_MODE0_PDOFF		0x80	// mode 0 - disable auto power down
	#define ACC_MODE0_RVOFF		0x40	// mode 0 - disable temp compensation
	#define ACC_MODE0_TMPOFF	0x20	// mode 0 - disable temp measurement
	#define ACC_MODE0_AGCON		0x10	// mode 0 - enable auto mode pitch and roll
	#define ACC_MODE0_MAUTO		0x04	// mode 0 - enable auto termination
	#define ACC_MODE0_GDET00	0x00	// mode 0 - g detection threshold - see ML8953 data sheet
	#define ACC_MODE0_GDET01	0x01	// mode 0 - g detection threshold - see ML8953 data sheet
	#define ACC_MODE0_GDET10	0x02	// mode 0 - g detection threshold - see ML8953 data sheet
#define ACC_MODE1			0x06	// mode 1
	#define ACC_MODE1_MOFF		0x20	// mode 1 - disable 3-axis continuous mode
	#define ACC_MODE1_ZAXIS		0x03	// mode 1 - Z axis
	#define ACC_MODE1_YAXIS		0x02	// mode 1 - Y axis
	#define ACC_MODE1_XAXIS		0x01	// mode 1 - X axis
	#define ACC_MODE1_RAXIS		0x00	// mode 1 - Reference axis
#define ACC_INTRQ			0x07	// interrupt request (1 = request)
#define ACC_INTMSK			0x08	// interrupt mask (1 = masked)
	#define ACC_INT_TREQ		0x20	// interrupt - temperature
	#define ACC_INT_GREQ		0x08	// interrupt - acceleration/no tilt
	#define ACC_INT_GCREQ		0x04	// interrupt - acceleration/tilt
	#define ACC_INT_GAREQ		0x01	// interrupt - automatic
#define ACC_TMDL			0x09	// timer LSB = (1/6.2 MHz) x 2048 x TMD
#define ACC_TMDH			0x0A	// timer MSB
#define ACC_CFG				0x0C	// configuration
	#define ACC_CFG_REGMD		0x80	// address auto-increment
	#define ACC_CFG_SPI3M_3		0x40	// spi mode = 3-wire
	#define ACC_CFG_SPI3M_4		0x00	// spi mode = 4-wire
	#define ACC_CFG_SDOCFG_T	0x10	// sdo mode = totem-pole
	#define ACC_CFG_SDOCFG_OC	0x00	// sdo mode = open-drain
	#define ACC_CFG_INT1EN_G	0x08	// interrupt 1 mode = g only
	#define ACC_CFG_INT1EN_ALL	0x00	// interrupt 1 mode = all
	#define ACC_CFG_INTLVL		0x04	// interrupt level mode
	#define ACC_CFG_INT1CFG_T	0x02	// interrupt 1 mode = totem-pole
	#define ACC_CFG_INT1CFG_OC	0x00	// interrupt 1 mode = open-drain
	#define ACC_CFG_INT0CFG_T	0x01	// interrupt 0 mode = totem-pole
	#define ACC_CFG_INT0CFG_OC	0x00	// interrupt 0 mode = open-drain
#define ACC_INTOTM			0x0D	// interrupt output conditions
#define ACC_GAAVE			0x0E	// Data averaging - automatic mode
#define ACC_GNAVE			0x0F	// Data averaging - normal mode
#define ACC_GDTCT0L			0x11	// threshold 0 LSB
#define ACC_GDTCT0H			0x12	// threshold 0 MSB
#define ACC_GDTCT1L			0x13	// threshold 1 LSB
#define ACC_GDTCT1H			0x14	// threshold 1 MSB
#define ACC_CPURDY			0x15	// device ready status (ready = 0x01)
	// page 1
#define ACC_STATUS			0x01	// measurment status
	#define ACC_STATUS_ASTS		0x02	// acceleration measurement - automatic modes
	#define ACC_STATUS_STS		0x01	// acceleration measurement - non-automatic modes
#define ACC_GAXL			0x02	// g vector
#define ACC_GAXH			0x03	// g vector
#define ACC_GAYL			0x04	// g vector
#define ACC_GAYH			0x05	// g vector
#define ACC_GAZL			0x06	// g vector
#define ACC_GAZH			0x07	// g vector
#define ACC_GASVL			0x08	// g vector
#define ACC_GASVH			0x09	// g vector
#define ACC_GNXL                        0x0A    // g vector
#define ACC_GNXH                        0x0B    // g vector
#define ACC_GNYL                        0x0C    // g vector
#define ACC_GNYH                        0x0D    // g vector    
#define ACC_GNZL                        0x0E    // g vector
#define ACC_GNZH                        0x0F    // g vector
#define ACC_GNSVL			0x10	// g vector
#define ACC_GNSVH			0x11	// g vector
#define ACC_PITCHL			0x12	// pitch
#define ACC_PITCHH			0x13	// pitch
#define ACC_ROLLL			0x14	// roll
#define ACC_ROLLH			0x15	// roll 
#define ACC_TEMPL			0x19	// temperature
#define ACC_TEMPH			0x1A	// temperature

	// read byte from I2C IO expander
static int ReadByte_IOX(struct i2c_adapter *adap, unsigned char offset, unsigned char *data)
{
		int	ret = 0;
		struct i2c_msg rmsg[2];
		int	num_msgs;
		int retries = 0;
		
		/* Read Byte with Pointer */
				
		rmsg[0].addr = BMI_IOX_I2C_ADDRESS;
		rmsg[0].flags = 0;	  /* write */
		rmsg[0].len = 1;
		rmsg[0].buf = &offset;

		rmsg[1].addr = BMI_IOX_I2C_ADDRESS;
		rmsg[1].flags = I2C_M_RD;   /* read */ 
		rmsg[1].len = 1;
		rmsg[1].buf = data;

		num_msgs = 2;

		while (retries < 5)
		  {
		    ret = i2c_transfer (adap, rmsg, num_msgs);
		    if (ret == 2)
		      break;
		    else
		      retries++;
		  }

		if (ret == 2) {
			ret = 0;
		}
		else {
			printk (KERN_ERR "ReadByte_IOX() - i2c_transfer() failed.\n");
			ret = -1;
		}
		return ret;
}

	// write byte to I2C IO expander
static int WriteByte_IOX(struct i2c_adapter *adap, unsigned char offset, unsigned char data)
{
		int	ret = 0;
		struct i2c_msg wmsg[2];
		int	num_msgs;
		
		/* Write Byte with Pointer */

		wmsg[0].addr = BMI_IOX_I2C_ADDRESS;
		wmsg[0].flags = 0;	/* write */
		wmsg[0].len = 1;
		wmsg[0].buf = &offset;

		wmsg[1].addr = BMI_IOX_I2C_ADDRESS;
		wmsg[1].flags = 0;	/* write */ 
		wmsg[1].len = 1;
		wmsg[1].buf = &data;

		num_msgs = 2;

		ret = i2c_transfer (adap, wmsg, num_msgs);

		if (ret == 2) {
			ret = 0;
		}
		else {
			printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed.\n");
			ret = -1;
		}
		return ret;
}


#if defined ACCELEROMETER
	// read byte from I2C acceleromter
static int ReadByte_ACC(struct i2c_adapter *adap, unsigned char offset, unsigned char *data)
{
		int	ret = 0;
		struct i2c_msg rmsg[2];
		int	num_msgs;
		int retries = 0;

		/* Read Byte with Pointer */

		rmsg[0].addr = BMI_ACC_I2C_ADDRESS;
		rmsg[0].flags = 0;	  /* write */
		rmsg[0].len = 1;
		rmsg[0].buf = &offset;

		rmsg[1].addr = BMI_ACC_I2C_ADDRESS;
		rmsg[1].flags = I2C_M_RD;   /* read */ 
		rmsg[1].len = 1;
		rmsg[1].buf = data;

		num_msgs = 2;	

		while (retries < 5)
		  {
		    ret = i2c_transfer (adap, rmsg, num_msgs);
		    if (ret == 2)
		      break;
		    else
		      retries++;
		    mdelay(1);
		  }

		if (ret == 2) {
			ret = 0;
		}
		else {
			printk (KERN_ERR "ReadByte_ACC() - i2c_transfer() failed.\n");
			ret = -1;
		}
		return ret;
}

static int ReadByteLock_ACC(struct pbmi_lcd *priv, unsigned char offset, unsigned char *data, int slot)
{
		int	ret = 0;
		struct i2c_msg rmsg[2];
		int	num_msgs;
		int retries = 0;

		/* Read Byte with Pointer */

		rmsg[0].addr = BMI_ACC_I2C_ADDRESS;
		rmsg[0].flags = 0;	  /* write */
		rmsg[0].len = 1;
		rmsg[0].buf = &offset;

		rmsg[1].addr = BMI_ACC_I2C_ADDRESS;
		rmsg[1].flags = I2C_M_RD;   /* read */ 
		rmsg[1].len = 1;
		rmsg[1].buf = data;

		num_msgs = 2;	

		while (retries < 5)
		  {
		    ret = i2c_transfer (adap, rmsg, num_msgs);
		    if (ret == 2)
		      break;
		    else
		      retries++;
		    mdelay(1);
		  }

		if (ret == 2) {
			ret = 0;
		}
		else {
			printk (KERN_ERR "ReadByte_ACC() - i2c_transfer() failed.\n");
			ret = -1;
		}
		return ret;
}

	// write byte to I2C accelerometer
static int WriteByte_ACC(struct i2c_adapter *adap, unsigned char offset, unsigned char data)
{
		int	ret = 0;
		struct i2c_msg wmsg[2];
		int	num_msgs;
		
		/* Write Byte with Pointer */

		wmsg[0].addr = BMI_ACC_I2C_ADDRESS;
		wmsg[0].flags = 0;	/* write */
		wmsg[0].len = 1;
		wmsg[0].buf = &offset;

		wmsg[1].addr = BMI_ACC_I2C_ADDRESS;
		wmsg[1].flags = 0;	/* write */ 
		wmsg[1].len = 1;
		wmsg[1].buf = &data;

		num_msgs = 2;

		ret = i2c_transfer (adap, wmsg, num_msgs);

		if (ret == 2) {
			ret = 0;
		}
		else {
			printk (KERN_ERR "WriteByte_ACC() - i2c_transfer() failed.\n");
			ret = -1;
		}
		return ret;
}
#endif	// ACCELEROMETER

/*
 * 	SPI functions
 */
 
	// TSC2046 touch screen controller command register bit definitons
#define SPI_START	0x80	// command start
#define SPI_AT0		0x00	// read temperature - not supported
#define SPI_AY		0x10	// read Y
#define SPI_ABAT	0x20	// read battery - not supported
#define SPI_AZ1		0x30	// read Z1
#define SPI_AZ2		0x40	// read Z2
#define SPI_AX		0x50	// read X
#define SPI_AAUX	0x60	// read AUX - not supported
#define SPI_AT1		0x70	// read temperature - not supported
#define SPI_MODE_12	0x00	// 12-bit mode - Preferred
#define SPI_MODE_8	0x08	// 8-bit mode
#define SPI_MODE_DFR	0x00	// differential mode - Preferred
#define SPI_MODE_SER	0x04	// single ended mode
#define SPI_PD		0x00	// power down - PENIRQ enabled
#define SPI_ADC		0x01	// ADC enabled
#define SPI_REF		0x02	// Vref enabled - unused
#define SPI_REF_ADC	0x03	// Vref & ADC enabled - unused

	// spi access
static int spi_rw(struct spi_device *spi, u8 * buf, size_t len)
{
	struct spi_transfer t = {
		.tx_buf = (const void *)buf,
		.rx_buf = buf,
		.len = len,
		.cs_change = 0,
		.delay_usecs = 0,
	};
	struct spi_message m;

	spi_message_init(&m);

	spi_message_add_tail(&t, &m);
	if (spi_sync(spi, &m) != 0 || m.status != 0)
		return -1;

	return m.actual_length;
}

	// spi write register
static ssize_t spi_lcd_write_reg(struct pbmi_lcd *priv, char *buf, int len, int slot)
{
	int res = 0;

	down(&priv->sem[slot]);

	memset(priv->wbuf[slot], 0, BUF_MAX_SIZE);
	priv->wbuf[slot][0] = buf[0];
	priv->wbuf[slot][1] = buf[1];
	priv->wbuf[slot][2] = buf[2];
	priv->wbuf[slot][3] = buf[3];
	res = spi_rw(priv->spi[slot], priv->wbuf[slot], len);
	if (res != 1) {
		up(&priv->sem[slot]);
		return -EFAULT;
	}

	up(&priv->sem[slot]);

	return res;
}

	// spi read register
static ssize_t spi_lcd_read_reg(struct pbmi_lcd *priv, char *buf, int len, int slot)
{
	int res = 0;

	down(&priv->sem[slot]);

	memset(priv->wbuf[slot], 0, BUF_MAX_SIZE);
	priv->wbuf[slot][0] = buf[0];
	priv->wbuf[slot][1] = buf[1];
	priv->wbuf[slot][2] = buf[2];
	priv->wbuf[slot][3] = buf[3];
	res = spi_rw(priv->spi[slot], priv->wbuf[slot], len);
	if (res != 1) {
		up(&priv->sem[slot]);
		return -EFAULT;
	}

	memset(priv->rbuf[slot], 0, BUF_MAX_SIZE);
	buf[0] = priv->wbuf[slot][2];
	buf[1] = priv->wbuf[slot][1];

	up(&priv->sem[slot]);

	return res;
}

/*
 * 	BMI functions
 */

static irqreturn_t module_irq_handler(int irq, void *dummy);
void bmi_lcd_config(struct bmi_lcd *lcd, int disp);

	// probe
int bmi_lcd_probe(struct bmi_device *bdev)
{	
#if defined ACCELEROMETER
	unsigned char acc_data[1];
#endif	// ACCELEROMETER
	unsigned char iox_data[1];
	int slot = bdev->info->slot;
	struct i2c_adapter *adap;
	struct bmi_lcd *lcd;
	char buf[4];
	/*int first_time = 1;*/

	printk(KERN_INFO "bmi_lcd.c: probe slot %d\n", slot);

		// check for opposite side already active
	switch(slot) {	// opposite side
		case 0: 
			if(pbmi_lcd.activated[2] == 1) {
				printk(KERN_INFO "bmi_lcd.c: probe slot %d not allowed (slot 2 already active)\n", slot);
				bmi_slot_power_off(0);
				/*bmi_slot_power_off(2);*/
				pbmi_lcd.bdev[0] = bdev;
				/*bdev = pbmi_lcd.bdev[2];
				slot = 2;
				first_time = 0;*/
				return 0;
			}
			break;
		case 1: 
			if(pbmi_lcd.activated[3] == 1) {
				printk(KERN_INFO "bmi_lcd.c: probe slot %d not allowed (slot 3 already active)\n", slot);
				bmi_slot_power_off(1);
				/*bmi_slot_power_off(3);*/
				pbmi_lcd.bdev[1] = bdev;
				/*bdev = pbmi_lcd.bdev[3];
				slot = 3;
				first_time = 0;*/
				return 0;
			}
			break;
		case 2: 
			if(pbmi_lcd.activated[0] == 1) {
				printk(KERN_INFO "bmi_lcd.c: probe slot %d not allowed (slot 0 already active)\n", slot);
				bmi_slot_power_off(2);
				/*bmi_slot_power_off(0);*/
				pbmi_lcd.bdev[2] = bdev;
				/*bdev = pbmi_lcd.bdev[0];
				slot = 0;
				first_time = 0;*/
				return 0;
			}
			break;
		case 3: 
			if(pbmi_lcd.activated[1] == 1) {
				printk(KERN_INFO "bmi_lcd.c: probe slot %d not allowed (slot 1 already active)\n", slot);
				bmi_slot_power_off(3);
				/*bmi_slot_power_off(1);*/
				pbmi_lcd.bdev[3] = bdev;
				/*bdev = pbmi_lcd.bdev[1];
				slot = 1;
				first_time = 0;*/
				return 0;
			}
			break;
	}

	adap = &bdev->adap;
	bmi_slot_power_on(slot);

	mdelay(500);

		// configure IOX
		// [7:6]=interrupts, [5]=SER_PD*, [4]=LCD_RST*, [3]=VSYNC_OE*, [2]=ACC_RST*, [1:0]=backlight
	if(WriteByte_IOX(adap, IOX_OUTPUT_REG, 0xFF))	// normal - no accelerometer interrupts
		return -ENODEV;

		// normal operation - no accelerometer interrupts
	if(WriteByte_IOX(adap, IOX_CONTROL, 0x00))	// IOX[7:0]=OUT
		return -ENODEV;

		// clear interrupts
	if(ReadByte_IOX(adap, IOX_INPUT_REG, iox_data))
		return -ENODEV;

	printk(KERN_INFO "bmi_lcd.c: probe slot %d iox data =  %x\n", slot, *iox_data);

#if defined ACCELEROMETER
		// accelerometer
	printk(KERN_INFO "bmi_lcd.c: probe slot %d hardware version =  0x%x\n", slot, bdev->epraw.revision_msb);
	
		// check for PCB revision >= 1.2
	if(bdev->epraw.revision_msb >= 0x12) {

			// normal IOX operation - accelerometer interrupts
		if(WriteByte_IOX(adap, IOX_CONTROL, 0xC0))	// IOX[7:6]=IN, IOX[5:0]=OUT
			return -ENODEV;

		if(WriteByte_IOX(adap, IOX_OUTPUT_REG, 0xFB))	// reset OKI accelerometer
			return -ENODEV;

		mdelay(2);

		if(WriteByte_IOX(adap, IOX_OUTPUT_REG, 0xFF))	// enable OKI accelerometer
			return -ENODEV;

		mdelay(2);

			// write PAGESEL
		*acc_data = 0x0;
		if(WriteByte_ACC(adap, ACC_PAGESEL, *acc_data))
			return -ENODEV;

			// read device to verify existance
		if(ReadByte_ACC(adap, ACC_CPURDY, acc_data))
			return -ENODEV;
	
			// set TMD = 0x300 (~250 ms)
		*acc_data = 0x5;
		if(WriteByte_ACC(adap, ACC_TMDH, *acc_data))
			return -ENODEV;

		*acc_data = 0x0;
		if(WriteByte_ACC(adap, ACC_TMDL, *acc_data))
			return -ENODEV;

			// set INTOTM
		*acc_data = 0x00;
		if(WriteByte_ACC(adap, ACC_INTOTM, *acc_data))
			return -ENODEV;

			// set GxAVE
		*acc_data = 0x0;
		if(WriteByte_ACC(adap, ACC_GAAVE, *acc_data))
			return -ENODEV;

			// set GDTCT[01]
		*acc_data = 0x00;
		if(WriteByte_ACC(adap, ACC_GDTCT0L, *acc_data))
			return -ENODEV;

		*acc_data = 0x00;
		if(WriteByte_ACC(adap, ACC_GDTCT0H, *acc_data))
			return -ENODEV;

		*acc_data = 0x00;
		if(WriteByte_ACC(adap, ACC_GDTCT1L, *acc_data))
			return -ENODEV;

		*acc_data = 0x00;
		if(WriteByte_ACC(adap, ACC_GDTCT1H, *acc_data))
			return -ENODEV;

			// set MODE0
		*acc_data = ACC_MODE0_PDOFF | ACC_MODE0_TMPOFF | ACC_MODE0_AGCON | ACC_MODE0_MAUTO | ACC_MODE0_GDET10;
		if(WriteByte_ACC(adap, ACC_MODE0, *acc_data))
			return -ENODEV;

			// set CFG
		*acc_data = ACC_CFG_REGMD | ACC_CFG_INTLVL;
		if(WriteByte_ACC(adap, ACC_CFG, *acc_data))
			return -ENODEV;

			// set INTMSK
		*acc_data = 0xFE;
		if(WriteByte_ACC(adap, ACC_INTMSK, *acc_data))
			return -ENODEV;
	
			// set CTRL0
		*acc_data = ACC_CTRL0_CGAUTO;
		if(WriteByte_ACC(adap, ACC_CTRL0, *acc_data))
			return -ENODEV;

			// write PAGESEL
		*acc_data = 0x1;
		if(WriteByte_ACC(adap, ACC_PAGESEL, *acc_data))
			return -ENODEV;

		acc_probe(&pbmi_lcd.acc[slot], slot);

	} else {
		printk(KERN_INFO "bmi_lcd.c: probe slot %d hardware version =  0x%x (accelerometer not supported)\n", slot, bdev->epraw.revision_msb);
	}
#endif	// ACCELEROMETER

		// reset serial link (master)
	if((slot == 0) || (slot == 2)) {	  
	  bmi_lcd_inactive(0);
	} else {
	  bmi_lcd_inactive(1);
	}

		// configure GPIO
		// turn LED's on
	bmi_set_module_gpio_data(slot, 3, 0);	// Red LED=ON
	bmi_set_module_gpio_data(slot, 2, 0);	// Green LED=ON

		// assert reset
	bmi_set_module_gpio_data(slot, 1, 0);	// RST=0

		// set GPIO direction
	bmi_set_module_gpio_dir(slot, 3, BMI_GPIO_OUT);
	bmi_set_module_gpio_dir(slot, 2, BMI_GPIO_OUT);
	bmi_set_module_gpio_dir(slot, 1, BMI_GPIO_OUT);
	bmi_set_module_gpio_dir(slot, 0, BMI_GPIO_IN);	// real-time pen int state

	mdelay(200);

		// turn LED's off
	bmi_set_module_gpio_data(slot, 3, 1);	// Red LED=OFF
	bmi_set_module_gpio_data(slot, 2, 1);	// Green LED=OFF

		// deassert reset (module)
	bmi_set_module_gpio_data(slot, 1, 1);	// RST=1

	mdelay(500);

		// unreset serial link (master)
	if((slot == 0) || (slot == 2)) {
		mdelay(2);
		bmi_lcd_active(0, 0x0, LCD_MODE_I80);		
	} else {
		mdelay(2);
		bmi_lcd_active(1, 0x0, LCD_MODE_I80);
	}


			// set up bdev/pbmi_lcd pointers
	bmi_device_set_drvdata(bdev, &pbmi_lcd);
	pbmi_lcd.bdev[slot] = bdev;
	
	// spi set-up
	if (bmi_device_spi_setup(bdev, 2000000, SPI_MODE_2, 32)) {
	  printk(KERN_ERR "bmi_lcd.c: Unable to setup spi%d\n", slot);
	  bmi_device_set_drvdata(bdev, NULL);
	  pbmi_lcd.bdev[slot] = NULL;
	  bmi_slot_power_off(slot);
	  return -EFAULT;
	}
	
	bmi_slot_spi_enable(slot);
	pbmi_lcd.spi[slot] = bmi_device_get_spi(bdev);


		// check spi access and enable touch screen
	memset(buf, 0, 4);
	buf[3] = SPI_START | SPI_PD;
	if(spi_lcd_write_reg(&pbmi_lcd, buf, 1, slot) != 1) {
		printk(KERN_WARNING "bmi_lcd.c: Unable set-up spi for bmi_lcd %d\n", slot);
		bmi_device_set_drvdata(bdev, NULL);
		pbmi_lcd.bdev[slot] = NULL;
		pbmi_lcd.spi[slot] = NULL;
		bmi_device_spi_cleanup(bdev);
		bmi_slot_spi_disable(slot);
		bmi_slot_power_off(slot);
		return -EFAULT;
	}


	// complete pbmi_lcd set-up
	pbmi_lcd.lcd_cnt++;
	pbmi_lcd.active = 1;
	pbmi_lcd.activated[slot] = 1;
	

	mdelay(100);

	lcd = pbmi_lcd.blcd[slot];
	if((slot == 0) || (slot == 2)) {
		mdelay(2);
		bmi_lcd_config(lcd, 0);
		mdelay(2);
	} else {
		mdelay(2);
		bmi_lcd_config(lcd, 1);
		mdelay(2);
	}


			// request input event interrupt handler
	pbmi_lcd.interrupt[0] = M1_IRQ;
	pbmi_lcd.interrupt[1] = M2_IRQ;
	pbmi_lcd.interrupt[2] = M3_IRQ;
	pbmi_lcd.interrupt[3] = M4_IRQ;
	snprintf(pbmi_lcd.int_name, sizeof(pbmi_lcd.int_name), "bmi_lcd%d", slot);
	if (request_irq(pbmi_lcd.interrupt[slot], &module_irq_handler, 0, pbmi_lcd.int_name, &pbmi_lcd)) {
	  printk( KERN_ERR "bmi_lcd.c: Can't allocate irq %d or find lcd in slot %d\n", pbmi_lcd.interrupt[slot], slot); 
	  bmi_device_set_drvdata(bdev, NULL);
	  pbmi_lcd.bdev[slot] = NULL;
	  pbmi_lcd.spi[slot] = NULL;
	  bmi_device_spi_cleanup(bdev);
	  bmi_slot_power_off(slot);
	  return -EBUSY;
	}

		// check GPIO status
	printk(KERN_INFO "bmi_lcd.c: slot %d gpio = %x\n", slot, bmi_read_gpio_data_reg(slot));
	printk(KERN_INFO "bmi_lcd.c: LCD count = %d\n", pbmi_lcd.lcd_cnt);

	return 0;
}

extern struct delayed_work bmilcd_work0;
extern struct delayed_work bmilcd_work1;
extern struct delayed_work bmilcd_work2;
extern struct delayed_work bmilcd_work3;

	// remove
void bmi_lcd_remove(struct bmi_device *bdev)
{	
	int slot = bdev->info->slot;

	if(pbmi_lcd.activated[slot] == 0)
	    return;

	switch(slot) {
		case 0: 
			cancel_delayed_work(&bmilcd_work0);
			break;
		case 1: 
			cancel_delayed_work(&bmilcd_work1);
			break;
		case 2: 
			cancel_delayed_work(&bmilcd_work2);
			break;
		case 3: 
			cancel_delayed_work(&bmilcd_work3);
			break;
	}

	free_irq(pbmi_lcd.interrupt[slot], &pbmi_lcd);

	bmi_set_module_gpio_dir (slot, 3, BMI_GPIO_IN);
	bmi_set_module_gpio_dir (slot, 2, BMI_GPIO_IN);
	bmi_set_module_gpio_dir (slot, 1, BMI_GPIO_IN);
	bmi_set_module_gpio_dir (slot, 0, BMI_GPIO_IN);

		// bmi/spi clean-up
	bmi_device_spi_cleanup(bdev);
	pbmi_lcd.spi[slot] = NULL;
	bmi_slot_spi_disable(slot);

		//de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (&bdev[slot], 0);

		// deactivate
	pbmi_lcd.activated[slot] = 0;
	pbmi_lcd.bdev[slot] = 0;
	pbmi_lcd.lcd_cnt--;

	if((pbmi_lcd.activated[0] == 0) && (pbmi_lcd.activated[2] == 0)) {
		bmi_lcd_inactive(0); // disable serializer
	}

	if((pbmi_lcd.activated[1] == 0) && (pbmi_lcd.activated[3] == 0)) {
		bmi_lcd_inactive(1); // disable serializer
	}

	if((pbmi_lcd.activated[0] == 0) && (pbmi_lcd.activated[1] == 0) && 
		(pbmi_lcd.activated[2] == 0) && (pbmi_lcd.activated[3] == 0)) {
		pbmi_lcd.active = -1;
	}

		// enable LCD on opposite side
	switch(slot) {
		case 0: 
			if(pbmi_lcd.bdev[2] != 0)
				bmi_lcd_probe(pbmi_lcd.bdev[2]);
			break;
		case 1: 
			if(pbmi_lcd.bdev[3] != 0)
				bmi_lcd_probe(pbmi_lcd.bdev[3]);
			break;
		case 2: 
			if(pbmi_lcd.bdev[0] != 0)
				bmi_lcd_probe(pbmi_lcd.bdev[0]);
			break;
		case 3: 
			if(pbmi_lcd.bdev[1] != 0)
				bmi_lcd_probe(pbmi_lcd.bdev[1]);
			break;
	}

#ifdef ACCELEROMETER
	acc_remove(&pbmi_lcd.acc[slot], slot);
#endif
	printk(KERN_INFO "bmi_lcd.c: LCD count = %d\n", pbmi_lcd.lcd_cnt);

	return;
}

/*
 * Input interrupt handler and support routines
 */

static void update_pen_state(void *arg, int slot, int x, int y, int pressure)
{
  struct pbmi_lcd *pbmi_lcd = (struct pbmi_lcd *)arg;
  int sync = 0;

  if (pressure) 
    {      
      /*input_report_abs(pbmi_lcd->input_dev[slot], ABS_X, x);
      input_report_abs(pbmi_lcd->input_dev[slot], ABS_Y, y);
      input_report_abs(pbmi_lcd->input_dev[slot], ABS_PRESSURE, pressure);

      input_report_abs(pbmi_lcd->input_dev[BMI_TS_M1234], ABS_X, x);
      input_report_abs(pbmi_lcd->input_dev[BMI_TS_M1234], ABS_Y, y);
      input_report_abs(pbmi_lcd->input_dev[BMI_TS_M1234], ABS_PRESSURE, pressure);*/

      if((slot == 0) || (slot == 2)) 
	{	  
	  input_report_abs(pbmi_lcd->input_dev[BMI_TS_M13], ABS_Y, y);
	  input_report_abs(pbmi_lcd->input_dev[BMI_TS_M13], ABS_X, x);
	  input_report_abs(pbmi_lcd->input_dev[BMI_TS_M13], ABS_PRESSURE, pressure);
	} 
      else 
	{	  
	  input_report_abs(pbmi_lcd->input_dev[BMI_TS_M24], ABS_Y, y);
	  input_report_abs(pbmi_lcd->input_dev[BMI_TS_M24], ABS_X, x);	  
	  input_report_abs(pbmi_lcd->input_dev[BMI_TS_M24], ABS_PRESSURE, pressure);
	}

      if (!pbmi_lcd->pen_down[slot])
	{	  
	  /*input_report_key(pbmi_lcd->input_dev[slot], BTN_TOUCH, 1);
	    input_report_key(pbmi_lcd->input_dev[BMI_TS_M1234], BTN_TOUCH, 1);*/
	  if((slot == 0) || (slot == 2)) 
	    {
	      input_report_key(pbmi_lcd->input_dev[BMI_TS_M13], BTN_TOUCH, 1);	      
	    } 
	  else 
	    {
	      input_report_key(pbmi_lcd->input_dev[BMI_TS_M24], BTN_TOUCH, 1);   	      
	    }
	 	 
	}
      sync = 1;
    } 
  else if (pbmi_lcd->pen_down[slot]) 
    {
      /*input_report_key(pbmi_lcd->input_dev[slot], BTN_TOUCH, 0);
	input_report_abs(pbmi_lcd->input_dev[slot], ABS_PRESSURE, 0);

      input_report_key(pbmi_lcd->input_dev[BMI_TS_M1234], BTN_TOUCH, 0);
      input_report_abs(pbmi_lcd->input_dev[BMI_TS_M1234], ABS_PRESSURE, 0); */

      if((slot == 0) || (slot == 2)) 
	{
	  input_report_key(pbmi_lcd->input_dev[BMI_TS_M13], BTN_TOUCH, 0);
	  input_report_abs(pbmi_lcd->input_dev[BMI_TS_M13], ABS_PRESSURE, 0);
	} 
      else 
	{
	  input_report_key(pbmi_lcd->input_dev[BMI_TS_M24], BTN_TOUCH, 0);	      
	  input_report_abs(pbmi_lcd->input_dev[BMI_TS_M24], ABS_PRESSURE, 0);
	}
      sync = 1;
    }
  
  if (sync)
    {
      /*input_sync(pbmi_lcd->input_dev[slot]);
	input_sync(pbmi_lcd->input_dev[BMI_TS_M1234]);*/
      if((slot == 0) || (slot == 2)) 
	{
	  input_sync(pbmi_lcd->input_dev[BMI_TS_M13]);
	} 
      else 
	{
	  input_sync(pbmi_lcd->input_dev[BMI_TS_M24]);	  
	}
    }
  pbmi_lcd->pen_down[slot] = pressure ? 1 : 0;

}


void bmilcd_input_work(void *arg, int slot);

void bmilcd_input_work0(struct work_struct * work) {
	bmilcd_input_work(&pbmi_lcd, 0);
}

void bmilcd_input_work1(struct work_struct * work) {
	bmilcd_input_work(&pbmi_lcd, 1);
}

void bmilcd_input_work2(struct work_struct * work) {
	bmilcd_input_work(&pbmi_lcd, 2);
}

void bmilcd_input_work3(struct work_struct * work) {
  bmilcd_input_work(&pbmi_lcd, 3);
}

DECLARE_DELAYED_WORK(bmilcd_work0, bmilcd_input_work0);
DECLARE_DELAYED_WORK(bmilcd_work1, bmilcd_input_work1);
DECLARE_DELAYED_WORK(bmilcd_work2, bmilcd_input_work2);
DECLARE_DELAYED_WORK(bmilcd_work3, bmilcd_input_work3);

// work handler
void bmilcd_input_work(void *arg, int slot) {
	struct pbmi_lcd 	*pbmi_lcd = (struct pbmi_lcd *)arg;
#if defined ACCELEROMETER
	struct i2c_adapter	*adap = &pbmi_lcd->bdev[slot]->adap;
	unsigned char		acc_data[1];
	static int		pitch = 0;
	static int		roll = 0;
	static int gx = 0;
	static int gy = 0;
	 
#endif	// ACCELEROMETER
	unsigned char		buf[4];
	int			x = 0;
	int			y = 0;
	int			z1 = 0;
	int			z2 = 0;
	int			pressure = 0;
	int			debounce;
	int penirq;

#if defined DEBUG
	printk(KERN_INFO "bmi_lcd.c: bmi_lcd_input work (slot %d)\n", slot);
#endif

	if(pbmi_lcd->bdev[slot] == 0) {
		printk(KERN_INFO "bmi_lcd.c: bmi_lcd_input work called with no bdev active (slot %d)\n", slot);
		return;
	}

#if defined ACCELEROMETER 
	if(pbmi_lcd->bdev[slot]->epraw.revision_msb >= 0x12) {

			// orientation
			// read ROLL
		if(ReadByte_ACC(adap, ACC_ROLLH, acc_data))
			return;
		roll = (0x0000 | *acc_data) << 8;
	
		if(ReadByte_ACC(adap, ACC_ROLLL, acc_data))
			return;
		roll = roll | *acc_data;
			// read PITCH
		if(ReadByte_ACC(adap, ACC_PITCHH, acc_data))
			return;
		pitch = (0x0000 | *acc_data) << 8;
	
		if(ReadByte_ACC(adap, ACC_PITCHL, acc_data))
			return;
		pitch = pitch | *acc_data;




		if(ReadByte_ACC(adap, ACC_GAZH, acc_data))
		   return;
   	        pbmi_lcd->acc[slot].sample[0] = *acc_data;
		
		if(ReadByte_ACC(adap, ACC_GAZL, acc_data))
		   return;
   	        pbmi_lcd->acc[slot].sample[1] = *acc_data;
		  
		if(ReadByte_ACC(adap, ACC_GAYH, acc_data))
		   return;
   	        pbmi_lcd->acc[slot].sample[2] = *acc_data;
		gy = *acc_data << 8;

		if(ReadByte_ACC(adap, ACC_GAYL, acc_data))
		   return;
   	        pbmi_lcd->acc[slot].sample[3] = *acc_data;
		gy = gy | *acc_data;
		
		if(ReadByte_ACC(adap, ACC_GAXH, acc_data))
		   return;
   	        pbmi_lcd->acc[slot].sample[4] = *acc_data;
		gx = *acc_data << 8;

		if(ReadByte_ACC(adap, ACC_GAXL, acc_data))
		   return;
   	        pbmi_lcd->acc[slot].sample[5] = *acc_data;
		gx = gx | *acc_data;
		
		//wake up any read's
		pbmi_lcd->acc[slot].flag = 1;
		wake_up_interruptible(&pbmi_lcd->acc[slot].wq);

		// read STATUS
		if(ReadByte_ACC(adap, ACC_STATUS, acc_data))
			return;

		if((*acc_data & 0x1) == 0) {

				// write PAGESEL
			*acc_data = 0x0;
			if(WriteByte_ACC(adap, ACC_PAGESEL, *acc_data))
				return;

				// read INTRQ
			if(ReadByte_ACC(adap, ACC_INTRQ, acc_data))
				return;
		}

			// write PAGESEL
		*acc_data = 0x1;
		if(WriteByte_ACC(adap, ACC_PAGESEL, *acc_data))
			return;

			// report orientation
		// printk(KERN_INFO "bmi_lcd.c: bmi_lcd_input work (slot %d) pitch=0x%x, roll=0x%x, ABS_MISC=0x%x\n", 
			// slot, pitch, roll, pitch << 16 | roll);	//pjg - debug

		input_report_abs(pbmi_lcd->input_dev[slot], ABS_MISC, (pitch << 16) | roll);
		input_sync(pbmi_lcd->input_dev[slot]);
	}
#endif	// ACCELEROMETER


	// read touch screen - X, Y, TOUCH, PRESSURE

	penirq = bmi_slot_status_irq_state(slot);
	/*printk(KERN_INFO "bmi_lcd.c: IRQ Status %d (slot %d) %d\n", penirq, slot,msecs_to_jiffies(10));*/
	
	if (pbmi_lcd->activated[slot] && penirq)
	  { 

	    for(debounce = 0; debounce < DEBOUNCE; debounce++) 
	      {
		
		memset(buf, 0, 4);
		buf[3] = SPI_START | SPI_AY | SPI_MODE_12 | SPI_MODE_DFR | SPI_ADC;
		spi_lcd_read_reg(pbmi_lcd, buf, 1, slot);
		y = (((buf[0] << 5) | buf[1] >> 3)) & 0xFFF;
		
		memset(buf, 0, 4);
		buf[3] = SPI_START | SPI_AX | SPI_MODE_12 | SPI_MODE_DFR | SPI_ADC;
		spi_lcd_read_reg(pbmi_lcd, buf, 1, slot);
		x = (((buf[0] << 5) | buf[1]) >> 3) & 0xFFF;
		
		memset(buf, 0, 4);
		buf[3] = SPI_START | SPI_AZ1 | SPI_MODE_12 | SPI_MODE_DFR | SPI_ADC;
		spi_lcd_read_reg(pbmi_lcd, buf, 1, slot);
		z1 = (((buf[0] << 5) | buf[1]) >> 3) & 0xFFF;
		
		memset(buf, 0, 4);
		buf[3] = SPI_START | SPI_AZ2 | SPI_MODE_12 | SPI_MODE_DFR | SPI_ADC;
		spi_lcd_read_reg(pbmi_lcd, buf, 1, slot);
		z2 = (((buf[0] << 5) | buf[1]) >> 3) & 0xFFF;
		mdelay(1);
	      }
	    
	    if(x && y && z1 && z2)
	      pressure = (X_PLATE * x / 4096) * ((z2 / z1) - 1);
	    	    
	    x = 4096 - x;
	    y = 4096 - y;
	    
	    if (pressure < 70)
	      {	
		if (pbmi_lcd->scount)
		  update_pen_state(arg, slot, x, y, pressure);
		else
		  {
		    pbmi_lcd->scount[slot]++;
		    /*update_pen_state(arg, slot, 0, 0, pressure);*/
		  }
	      }
	    /* else
	      {
		update_pen_state(arg, slot, 0, 0, pressure);		
		}*/	     
	    
		switch(slot)
		  {
		  case BMI_TS_M1:
		    schedule_delayed_work(&bmilcd_work0, WORK_DELAY);
		    break;
		  case BMI_TS_M2:
		    schedule_delayed_work(&bmilcd_work1, WORK_DELAY);
		    break;
		  case BMI_TS_M3:
		    schedule_delayed_work(&bmilcd_work2, WORK_DELAY);
		    break;
		  case BMI_TS_M4:
		    schedule_delayed_work(&bmilcd_work3, WORK_DELAY);
		    break;
		  }	       
		/* printk(KERN_INFO "bmi_lcd.c: work scheduled on (slot %d)\n", slot); */
		/*buf[3] = SPI_START | SPI_PD;
		  spi_lcd_write_reg(pbmi_lcd, buf, 1, slot);*/
	  }
	
	else
	  {
	    /*printk(KERN_INFO "bmi_lcd.c: Pen up on (slot %d)\n", slot);*/
	    memset(buf, 0, 4);
	    buf[3] = SPI_START | SPI_PD;
	    spi_lcd_write_reg(pbmi_lcd, buf, 1, slot);		
	    update_pen_state(arg,slot, 0, 0, 0);		
	    enable_irq(pbmi_lcd->interrupt[slot]);	
	  }

}


// interrupt handler
static irqreturn_t module_irq_handler(int irq, void *dummy)
{
  disable_irq(irq);
  /*printk(KERN_INFO "bmi_lcd.c: Interupt on (slot %d)\n", irq);*/
  switch(irq) 
    {
    case M1_IRQ:            
      schedule_delayed_work(&bmilcd_work0, WORK_DELAY);
      pbmi_lcd.scount[BMI_TS_M1] = 0;
      break;
    case M2_IRQ:
      schedule_delayed_work(&bmilcd_work1, WORK_DELAY);
      pbmi_lcd.scount[BMI_TS_M2] = 0;
      break;
    case M3_IRQ:
      schedule_delayed_work(&bmilcd_work2, WORK_DELAY);
      pbmi_lcd.scount[BMI_TS_M3] = 0;
      break;
    case M4_IRQ:
      schedule_delayed_work(&bmilcd_work3, WORK_DELAY);
      pbmi_lcd.scount[BMI_TS_M4] = 0;
      break;
    }
  return IRQ_HANDLED;
}

/*
 * control device operations
 */

/*
 * control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *filp)
{	
	if(pbmi_lcd.open_flag) {
		return - EBUSY;
	}
	pbmi_lcd.open_flag = 1;
	filp->private_data = &pbmi_lcd;
	return 0;
}

// release
int cntl_release(struct inode *inode, struct file *filp)
{	
	pbmi_lcd.open_flag = 0;
	return 0;
}

// ioctl
int cntl_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, 
		   unsigned long arg)
{	
	struct i2c_adapter *adap;
	unsigned char iox_data[1];
	int slot = (__user arg) & 0xF;
	int bl = ((__user arg) & 0x70) >> 4;

		// error if no lcd active.
	if(pbmi_lcd.active == -1)
		return -ENODEV;
		
	if(cmd != BMI_LCD_GETSTAT) {

			// error if slot invalid
		if((slot < CPLD_M1) || (slot > CPLD_M4))
			return -ENODEV;
	
			// error if no lcd in chosen slot
		if(pbmi_lcd.bdev[slot] == 0)
			return -ENODEV;

			// i2c adapter
		adap = &pbmi_lcd.bdev[slot]->adap;
	}

		// ioctl's
	switch (cmd) {
		case BMI_LCD_RLEDOFF:
			bmi_set_module_gpio_data(slot, 3, 1);// Red LED=OFF
			break;
		case BMI_LCD_RLEDON:
			bmi_set_module_gpio_data(slot, 3, 0);// Red LED=ON
			break;
		case BMI_LCD_GLEDOFF:
			bmi_set_module_gpio_data(slot, 2, 1);// Green LED=OFF
			break;
		case BMI_LCD_GLEDON:
			bmi_set_module_gpio_data(slot, 2, 0);// Green LED=ON
			break;
		case BMI_LCD_VSYNC_DIS:	// enable VSYNC buffer tristate output
			if(ReadByte_IOX(adap, IOX_OUTPUT_REG, iox_data))
				return -ENODEV;
			*iox_data |= 0x08;
			if(WriteByte_IOX(adap, IOX_OUTPUT_REG, *iox_data))
				return -ENODEV;
			break;
		case BMI_LCD_VSYNC_EN:	// disable VSYNC buffer tristate output
			if(ReadByte_IOX(adap, IOX_OUTPUT_REG, iox_data))
				return -ENODEV;
			*iox_data &= ~0x08;
			if(WriteByte_IOX(adap, IOX_OUTPUT_REG, *iox_data))
				return -ENODEV;
			break;
		case BMI_LCD_EN:	// enable LCD component
			if(ReadByte_IOX(adap, IOX_OUTPUT_REG, iox_data))
				return -ENODEV;
			*iox_data &= ~0x10;
			if(WriteByte_IOX(adap, IOX_OUTPUT_REG, *iox_data))
				return -ENODEV;
			break;
		case BMI_LCD_DIS:	// disable LCD component only
			if(ReadByte_IOX(adap, IOX_OUTPUT_REG, iox_data))
				return -ENODEV;
			*iox_data |= 0x10;
			if(WriteByte_IOX(adap, IOX_OUTPUT_REG, *iox_data))
				return -ENODEV;
			break;
		case BMI_LCD_SER_EN:	// enable Serializer component
			if(ReadByte_IOX(adap, IOX_OUTPUT_REG, iox_data))
				return -ENODEV;
			*iox_data &= ~0x20;
			if(WriteByte_IOX(adap, IOX_OUTPUT_REG, *iox_data))
				return -ENODEV;
			break;
		case BMI_LCD_SER_DIS:	// disable Serializer component only
			if(ReadByte_IOX(adap, IOX_OUTPUT_REG, iox_data))
				return -ENODEV;
			*iox_data |= 0x20;
			if(WriteByte_IOX(adap, IOX_OUTPUT_REG, *iox_data))
				return -ENODEV;
			break;
		case BMI_LCD_SETRST:	// overall module reset
			bmi_set_module_gpio_data (slot, 1, 0);		// RST=0
			break;
		case BMI_LCD_CLRRST:	// overall module enable
			bmi_set_module_gpio_data (slot, 1, 1);		// RST=1
			break;
		case BMI_LCD_SET_BL:	// set backlight brightness
			if(ReadByte_IOX(adap, IOX_OUTPUT_REG, iox_data))
				return -ENODEV;
			*iox_data = (*iox_data & 0xF8) | bl;
			if(WriteByte_IOX(adap, IOX_OUTPUT_REG, *iox_data))
				return -ENODEV;
			break;
		case BMI_LCD_GETSTAT:
			{
				int *slot = ((int __user *) arg);
				int read_data;

				*slot &= 0xF;

					// error if slot invalid
				if((*slot < CPLD_M1) || (*slot > CPLD_M4))
					return -ENODEV;
	
					// error if no lcd in chosen slot
				if(pbmi_lcd.bdev[*slot] == 0)
					return -ENODEV;

					// i2c adapter
				adap = &pbmi_lcd.bdev[*slot]->adap;

				if(ReadByte_IOX(adap, IOX_INPUT_REG, iox_data))
					return -ENODEV;

				read_data = *iox_data | (bmi_read_gpio_data_reg(*slot) << 8);

				if(put_user(read_data, (int __user *) arg))
						return -EFAULT;
			}
			break;
		case BMI_LCD_ACTIVATE:	//pjg fix/test
				// check for opposite side already active
			switch(slot) {	// opposite side
				case 0: 
					if(pbmi_lcd.activated[2] == 1) {
						printk(KERN_INFO "bmi_lcd.c: probe slot %d not allowed (slot 2 already active)\n", slot);
						bmi_slot_power_off(0);
						return -ENODEV;
					}
					break;
				case 1: 
					if(pbmi_lcd.activated[3] == 1) {
						printk(KERN_INFO "bmi_lcd.c: probe slot %d not allowed (slot 3 already active)\n", slot);
						bmi_slot_power_off(1);
						return -ENODEV;
					}
					break;
				case 2: 
					if(pbmi_lcd.activated[0] == 1) {
						printk(KERN_INFO "bmi_lcd.c: probe slot %d not allowed (slot 0 already active)\n", slot);
						bmi_slot_power_off(2);
						return -ENODEV;
					}
					break;
				case 3: 
					if(pbmi_lcd.activated[1] == 1) {
						printk(KERN_INFO "bmi_lcd.c: probe slot %d not allowed (slot 1 already active)\n", slot);
						bmi_slot_power_off(3);
						return -ENODEV;
					}
					break;
			}
				// activate
			if((!pbmi_lcd.activated[slot]) && (pbmi_lcd.bdev[slot] != 0)) {
				bmi_lcd_probe(pbmi_lcd.bdev[slot]);
			}
			break;
		case BMI_LCD_DEACTIVATE:
			if(pbmi_lcd.activated[slot]) {
				disable_irq_nosync(pbmi_lcd.interrupt[slot]);			
				pbmi_lcd.activated[slot] = 0;
				if(ReadByte_IOX(adap, IOX_OUTPUT_REG, iox_data))
					return -ENODEV;
				*iox_data = (*iox_data & 0xF8);
				if(WriteByte_IOX(adap, IOX_OUTPUT_REG, *iox_data))
					return -ENODEV;
				bmi_slot_power_off(slot);
			}
			break;
		case BMI_LCD_SUSPEND:
			printk(KERN_ERR "BMI_LCD_SUSPEND NOT IMPLEMENTED\n");	//pjg
			break;
		case BMI_LCD_RESUME:
			printk(KERN_ERR "BMI_LCD_RESUME NOT IMPLEMENTED\n");	//pjg
			break;
		default:
			return -ENOTTY;
	}
	return 0;
}

	// control file operations
struct file_operations cntl_fops = {
	.owner = THIS_MODULE, 
	.ioctl = cntl_ioctl, 
	.open = cntl_open, 
	.release = cntl_release, 
};

	// BMI LCD fops
void bmi_lcd_config(struct bmi_lcd *lcd, int disp)
{
	if(pbmi_lcd.active == -1) {
		return;
	}
	
	if((lcd) && (lcd->lcd_ops.config)) {
		lcd->lcd_ops.config(disp);
	}
}

void bmi_lcd_reset(struct bmi_lcd *lcd, int slot)
{
	if(pbmi_lcd.active == -1) {
		return;
	}
	
	if((lcd) && (lcd->lcd_ops.reset)) {
		lcd->lcd_ops.reset(slot);
	}
}

int register_bmi_lcd(struct bmi_lcd *lcd, int slot)	//pjg - placeholder for multiple LCD types
{
	if(!lcd) {
		return -1;
	}
	if((slot < 0) || (slot > 3)) {
		return -1;
	}
	if(pbmi_lcd.blcd[slot]) {
		return -1;
	}
	else {
		pbmi_lcd.blcd[slot] = lcd;
	}

	if(lcd->lcd_ops.activate) {
		lcd->lcd_ops.activate(lcd, slot);
	}

	return 0;
}

int unregister_bmi_lcd(struct bmi_lcd *lcd, int slot)	//pjg - placeholder for multiple LCD types
{
	if (!lcd) {
		return -1;
	}
	if ((slot < 0) || (slot > 3)) {
		return -1;
	}
	if (pbmi_lcd.blcd[slot] != lcd) {
		return -1;
	}
	else {
		pbmi_lcd.blcd [slot] = 0;
		lcd->lcd_ops.deactivate(lcd, slot);
	}	
	return 0;
}

static struct miscdevice cntl_dev = {
	MISC_DYNAMIC_MINOR,
	"bmi_lcd_control",
	&cntl_fops
};

/*
 *	Module functions
 */

char const input_name0[MAX_STRG] = "bmi_lcd_ts0";
char const input_name1[MAX_STRG] = "bmi_lcd_ts1";
char const input_name2[MAX_STRG] = "bmi_lcd_ts2";
char const input_name3[MAX_STRG] = "bmi_lcd_ts3";
char const input_name4[MAX_STRG] = "bmi_lcd_ts4";
char const input_name5[MAX_STRG] = "bmi_lcd_ts5";
char const input_name6[MAX_STRG] = "bmi_lcd_ts6";

static __init int bmi_lcd_init(void)
{	
  int ts;
  int rc = 0;
  
  // No lcd is active.
  pbmi_lcd.active = -1;
  pbmi_lcd.activated[0] = 0;
  pbmi_lcd.activated[1] = 0;
  pbmi_lcd.activated[2] = 0;
  pbmi_lcd.activated[3] = 0;
  
  // set up control character device - bmi_lcd_control
  rc = misc_register(&cntl_dev);
  if(rc) {
    printk(KERN_ERR "bmi_lcd.c: Can't allocate bmi_lcd_control device\n");
    return rc;
  }
  
  // Allocate and Register input device. - bmi_lcd_ts[BMI_TS_M1:BMI_TS_M1234]
  for(ts = BMI_TS_M1; ts < BMI_TS_NUM; ts++) {
    pbmi_lcd.input_dev[ts] = input_allocate_device();
    if(!pbmi_lcd.input_dev[ts]) {
      printk(KERN_ERR "bmi_lcd_init: Can't allocate input_dev[ts]\n"); 
      return -ENOMEM;
    }
    
    // set up input device
    switch(ts) {
    case BMI_TS_M1:
      pbmi_lcd.input_dev[BMI_TS_M1]->name = input_name0;
      pbmi_lcd.input_dev[BMI_TS_M1]->phys = input_name0;
      break;
    case BMI_TS_M2:
      pbmi_lcd.input_dev[BMI_TS_M2]->name = input_name1;
      pbmi_lcd.input_dev[BMI_TS_M2]->phys = input_name1;
      break;
    case BMI_TS_M3:
      pbmi_lcd.input_dev[BMI_TS_M3]->name = input_name2;
      pbmi_lcd.input_dev[BMI_TS_M3]->phys = input_name2;
      break;
    case BMI_TS_M4:
      pbmi_lcd.input_dev[BMI_TS_M4]->name = input_name3;
      pbmi_lcd.input_dev[BMI_TS_M4]->phys = input_name3;
      break;
    case BMI_TS_M13:
      pbmi_lcd.input_dev[BMI_TS_M13]->name = input_name4;
      pbmi_lcd.input_dev[BMI_TS_M13]->phys = input_name4;
      break;
    case BMI_TS_M24:
      pbmi_lcd.input_dev[BMI_TS_M24]->name = input_name5;
      pbmi_lcd.input_dev[BMI_TS_M24]->phys = input_name5;
      break;
    case BMI_TS_M1234:
      pbmi_lcd.input_dev[BMI_TS_M1234]->name = input_name6;
      pbmi_lcd.input_dev[BMI_TS_M1234]->phys = input_name6;
      break;
    }
    pbmi_lcd.input_dev[ts]->id.bustype = BUS_BMI;
    pbmi_lcd.input_dev[ts]->private = &pbmi_lcd;
    pbmi_lcd.input_dev[ts]->evbit[BIT_WORD(EV_KEY)] |= BIT_MASK(EV_KEY);
    pbmi_lcd.input_dev[ts]->evbit[BIT_WORD(EV_ABS)] |= BIT_MASK(EV_ABS);
    pbmi_lcd.input_dev[ts]->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);
    pbmi_lcd.input_dev[ts]->absbit[BIT_WORD(ABS_X)] |= BIT_MASK(ABS_X);
    pbmi_lcd.input_dev[ts]->absbit[BIT_WORD(ABS_Y)] |= BIT_MASK(ABS_Y);
    pbmi_lcd.input_dev[ts]->absbit[BIT_WORD(ABS_PRESSURE)] |= BIT_MASK(ABS_PRESSURE);
    pbmi_lcd.input_dev[ts]->absbit[BIT_WORD(ABS_MISC)] |= BIT_MASK(ABS_MISC);
    input_set_abs_params(pbmi_lcd.input_dev[ts], ABS_X, BMI_LCD_MIN_XC, BMI_LCD_MAX_XC, 0, 0);
    input_set_abs_params(pbmi_lcd.input_dev[ts], ABS_Y, BMI_LCD_MIN_YC, BMI_LCD_MAX_YC, 0, 0);
    input_set_abs_params(pbmi_lcd.input_dev[ts], ABS_PRESSURE, 0, 1024, 0, 0);
    
    // register input device 
    if(input_register_device(pbmi_lcd.input_dev[ts])) {
      int tts;
      printk(KERN_ERR "bmi_lcd_init() - input_register_device failed.\n");
      
      for(tts = BMI_TS_M1; tts < ts; tts++) 
	input_unregister_device(pbmi_lcd.input_dev[tts]);
      
      misc_deregister(&cntl_dev);
      
      return -ENODEV;
    }
  }
  
  pbmi_lcd.lcd_cnt = 0;
  
  // hardware specfic set-up
  s320x240_bmi_lcd.interface = s320x240_lcd_interface;
  s320x240_bmi_lcd_ops.config = (void(*)) &s320x240_config;
  s320x240_bmi_lcd_ops.reset = NULL;	//pjg - placeholder for multiple LCD hardware types
  s320x240_bmi_lcd_ops.suspend = NULL;	//pjg - placeholder for multiple LCD hardware types
  s320x240_bmi_lcd_ops.resume = NULL;	//pjg - placeholder for multiple LCD hardware types
  s320x240_bmi_lcd_ops.disp_on = NULL;	//pjg - placeholder for multiple LCD hardware types
  s320x240_bmi_lcd_ops.disp_off = NULL;	//pjg - placeholder for multiple LCD hardware types
  s320x240_bmi_lcd_ops.activate = NULL;	//pjg - placeholder for multiple LCD hardware types
  s320x240_bmi_lcd_ops.deactivate = NULL;	//pjg - placeholder for multiple LCD hardware types
  s320x240_bmi_lcd.lcd_ops = s320x240_bmi_lcd_ops;
  pbmi_lcd.blcd[0] = &s320x240_bmi_lcd;
  pbmi_lcd.blcd[1] = &s320x240_bmi_lcd;
  pbmi_lcd.blcd[2] = &s320x240_bmi_lcd;
  pbmi_lcd.blcd[3] = &s320x240_bmi_lcd;
  
  sema_init(&pbmi_lcd.sem[0], 1);
  sema_init(&pbmi_lcd.sem[1], 1);
  sema_init(&pbmi_lcd.sem[2], 1);
  sema_init(&pbmi_lcd.sem[3], 1);	    
  
  
  acc_init();
  
  // register with BMI
  rc = bmi_register_driver(&bmi_lcd_driver); 
  if(rc) {
    printk(KERN_ERR "bmi_lcd.c: Can't register bmi_lcd_driver\n");
    
    for(ts = BMI_TS_M1; ts < BMI_TS_NUM; ts++) 
      input_unregister_device(pbmi_lcd.input_dev[ts]);
    
    misc_deregister(&cntl_dev);
    
    return rc;
  }
  
  printk("bmi_lcd.c: BMI_LCD Driver v%s \n", BMILCD_VERSION);
  
  return 0;
}	


static void __exit bmi_lcd_clean(void)
{	
  int ts;
  
  // remove input devices
  for(ts = BMI_TS_M1; ts < BMI_TS_NUM; ts++) 
    input_unregister_device(pbmi_lcd.input_dev[ts]);
  
  // remove control device
  misc_deregister(&cntl_dev);
  
  // remove bmi driver
  bmi_unregister_driver(&bmi_lcd_driver);
  acc_clean();
  return;
}	

module_init(bmi_lcd_init);
module_exit(bmi_lcd_clean);

// Exported symbols
EXPORT_SYMBOL(register_bmi_lcd);
EXPORT_SYMBOL(unregister_bmi_lcd);


MODULE_AUTHOR("Peter Giacomini <p.giacomini@encadis.com>");
MODULE_DESCRIPTION("BMI lcd device driver");
MODULE_SUPPORTED_DEVICE("bmi_lcd_control");
MODULE_SUPPORTED_DEVICE("bmi_lcd_ts");
MODULE_SUPPORTED_DEVICE("bmi_lcd_acc");
MODULE_LICENSE("GPL");
