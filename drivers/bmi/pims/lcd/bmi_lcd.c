/*
 * 	bmi_lcd.c
 *
 * 	BMI LCD device driver basic functionality
 *
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
 *	Include files
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/gpio.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/hardware.h>

#include <linux/i2c.h>
#include <linux/i2c/tsc2004.h>

#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/bmi/bmi_lcd.h>


/*
 * 	Global variables
 */


static struct tsc2004_platform_data tsc_platform_data= {
  .model = 2004,
  .x_plate_ohms = 180,
};

static struct i2c_board_info tsc_info = {
  I2C_BOARD_INFO("tsc2004", 0x48),
  .platform_data = &tsc_platform_data,
};


// private device structure
struct bmi_lcd
{
  struct bmi_device	*bdev;			// BMI device
  struct cdev		cdev;			// control device
  struct device	*class_dev;		// control class device
  int			open_flag;		// single open flag
  char			int_name[20];		// interrupt name
  struct i2c_client *tsc;
};

struct bmi_lcd bmi_lcd;

static int major;		// control device major

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
	{ 0, },	  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_lcd_tbl);

int	bmi_lcd_probe (struct bmi_device *bdev);
void	bmi_lcd_remove (struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_lcd_driver = 
{
	.name = "bmi_lcd", 
	.id_table = bmi_lcd_tbl, 
	.probe   = bmi_lcd_probe, 
	.remove  = bmi_lcd_remove, 
};

/*
 *	PIM functions
 */

// interrupt handler
static irqreturn_t module_irq_handler(int irq, void *dummy)
{

	return IRQ_HANDLED;
}

/*
 * 	BMI functions
 */

// probe - insert PIM
int bmi_lcd_probe(struct bmi_device *bdev)
{
	int err;
	int slot;
	struct bmi_lcd *lcd;
       	struct i2c_adapter *adap;
	struct class *bmi_class;
	dev_t dev_id;
	int irq;

	int gpio_int;

	err = 0;
	slot = bdev->slot->slotnum;
      	adap = bdev->slot->adap;
	lcd = &bmi_lcd;

	lcd->bdev = 0;
	lcd->open_flag = 0;
	
	// Create 1 minor device

	dev_id = MKDEV(major, slot); 

	// Create class device 
	bmi_class = bmi_get_class ();                            

	// bind driver and bmi_device 
	lcd->bdev = bdev;

	tsc_info.irq = gpio_to_irq(10);
	lcd->tsc = i2c_new_device(bdev->slot->adap, &tsc_info);
	if (lcd->tsc == NULL)
	  printk(KERN_ERR "TSC NULL...\n");

	bmi_device_set_drvdata (bdev, lcd);

	printk (KERN_INFO "bmi_lcd.c: probe...\n");	

	// request PIM interrupt
	irq = bdev->slot->status_irq;
	sprintf (lcd->int_name, "bmi_lcd%d", slot);

	return 0;

 err1:	
	bmi_device_set_drvdata (bdev, 0);
	lcd->bdev = 0;
	return -ENODEV;
}

// remove PIM
void bmi_lcd_remove(struct bmi_device *bdev)
{	
	int slot;
	struct bmi_lcd *lcd;
	struct class *bmi_class;
	int irq;
	int i;

	printk(KERN_INFO "bmi_lcd: Module Removed...\n");
	slot = bdev->slot->slotnum;
	lcd = &bmi_lcd;
	i2c_unregister_device(lcd->tsc);
	irq = bdev->slot->status_irq;

	for (i = 0; i < 4; i++)
	  bmi_slot_gpio_direction_in(slot, i);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	lcd->class_dev = 0;

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	lcd->bdev = 0;

	return;
}

/*
 *	module routines
 */

static void __exit bmi_lcd_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver (&bmi_lcd_driver);

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}

static int __init bmi_lcd_init(void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI LCD Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_lcd_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}

	printk (KERN_INFO "bmi_lcd.c: BMI_LCD Driver...\n");

	return 0;
}


module_init(bmi_lcd_init);
module_exit(bmi_lcd_cleanup);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Buglabs Inc.");
MODULE_DESCRIPTION("BMI von Hippel device driver");
