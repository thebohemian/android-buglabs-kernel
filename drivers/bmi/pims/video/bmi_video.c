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

static struct i2c_board_info tfp_info = {
  I2C_BOARD_INFO("tfp410p", 0x38),
};


// private device structure
struct bmi_video
{
  struct bmi_device	*bdev;			// BMI device
  struct cdev		cdev;			// control device
  struct device	*class_dev;		// control class device
  int			open_flag;		// single open flag
  char			int_name[20];		// interrupt name
  struct i2c_client *tfp;
};

struct bmi_video bmi_video;

static int major;		// control device major

/*
 * 	BMI set up
 */

// BMI device ID table
static struct bmi_device_id bmi_video_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_BUG_LABS, 
		.product  = BMI_PRODUCT_VIDEO, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },	  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_video_tbl);

int	bmi_video_probe (struct bmi_device *bdev);
void	bmi_video_remove (struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_video_driver = 
{
	.name = "bmi_video", 
	.id_table = bmi_video_tbl, 
	.probe   = bmi_video_probe, 
	.remove  = bmi_video_remove, 
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
int bmi_video_probe(struct bmi_device *bdev)
{
	int err;
	int slot;
	struct bmi_video *video;
       	struct i2c_adapter *adap;
	struct class *bmi_class;
	dev_t dev_id;
	int irq;

	int gpio_int;

	err = 0;
	slot = bdev->slot->slotnum;
      	adap = bdev->slot->adap;
	video = &bmi_video;

	video->bdev = 0;
	video->open_flag = 0;
	
	// Create 1 minor device

	dev_id = MKDEV(major, slot); 

	// Create class device 
	bmi_class = bmi_get_class ();                            

	// bind driver and bmi_device 
	video->bdev = bdev;

	tfp_info.irq = gpio_to_irq(10);
	video->tfp = i2c_new_device(bdev->slot->adap, &tfp_info);
	if (video->tfp == NULL)
	  printk(KERN_ERR "TFP NULL...\n");

	bmi_device_set_drvdata (bdev, video);

	printk (KERN_INFO "bmi_video.c: probe...\n");	

	// request PIM interrupt
	irq = bdev->slot->status_irq;
	sprintf (video->int_name, "bmi_video%d", slot);

	return 0;

 err1:	
	bmi_device_set_drvdata (bdev, 0);
	video->bdev = 0;
	return -ENODEV;
}

// remove PIM
void bmi_video_remove(struct bmi_device *bdev)
{	
	int slot;
	struct bmi_video *video;
	struct class *bmi_class;
	int irq;
	int i;

	printk(KERN_INFO "bmi_video: Module Removed...\n");
	slot = bdev->slot->slotnum;
	video = &bmi_video;
	i2c_unregister_device(video->tfp);
	irq = bdev->slot->status_irq;

	for (i = 0; i < 4; i++)
	  bmi_slot_gpio_direction_in(slot, i);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	video->class_dev = 0;

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	video->bdev = 0;

	return;
}

/*
 *	module routines
 */

static void __exit bmi_video_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver (&bmi_video_driver);

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}

static int __init bmi_video_init(void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI VIDEO Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_video_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}

	printk (KERN_INFO "bmi_video.c: BMI_VIDEO Driver...\n");

	return 0;
}


module_init(bmi_video_init);
module_exit(bmi_video_cleanup);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Buglabs Inc.");
MODULE_DESCRIPTION("BMI von Hippel device driver");
