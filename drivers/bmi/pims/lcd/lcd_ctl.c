/*
 * Copyright 2008 EnCADIS Designs, Inc. All Rights Reserved.
 * Copyright 2008 Bug-Labs, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*-----------------------------------------------------------------------------
 *
 *      Part of BMI Motion Detector Accelerometer (MDACC) Kernel Module
 *
 *-----------------------------------------------------------------------------
 */

#include <linux/ioctl.h>
#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include "lcd_ctl.h"

static int lcd_ctl_open (struct inode *, struct file *);
static int lcd_ctl_release (struct inode *, struct file *);
static int lcd_ctl_ioctl (struct inode *, struct file *, unsigned int, unsigned long);
static int ReadByte_IOX(struct i2c_adapter *, unsigned char, unsigned char *)


struct file_operations lcd_ctl_fops = {
	.owner = THIS_MODULE, 
	.ioctl = lcd_ctl_ioctl, 
	.open = lcd_ctl_open, 
	.release = lcd_ctl_release, 
};


	// read byte from I2C IO expander
static int ReadByte_IOX(struct i2c_adapter *adap, unsigned char offset, unsigned char *data)
{
		int	ret = 0;
		struct i2c_msg rmsg[2];
		int	num_msgs;

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
		ret = i2c_transfer (adap, rmsg, num_msgs);

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


static int lcd_ctl_major;

int lcd_ctl_init (void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers

	retval = alloc_chrdev_region(&dev_id, 0, 4, "BMI LCD Control Driver"); 
				     
	if (retval) {
		return -1;
	}
	lcd_ctl_major = MAJOR(dev_id);
	return 0;
}

void lcd_ctl_clean (void)
{
	dev_t dev_id;

	dev_id = MKDEV(lcd_ctl_major, 0); 
	unregister_chrdev_region(dev_id, 4);
	return;
}

int lcd_ctl_probe (struct lcd_ctl *lcd_ctl, int slot) 
{
	struct cdev *cdev;
	dev_t dev_id;
	int ret;
	struct class *bmi_class;

	cdev = &lcd_ctl->cdev;
	cdev_init (cdev, &lcd_ctl_fops);

	dev_id = MKDEV (lcd_ctl_major, slot); 
	ret = cdev_add (cdev, dev_id, 1);

	//Create class device 
	bmi_class = bmi_get_bmi_class ();                            

	lcd_ctl->class_dev = device_create (bmi_class, NULL, MKDEV(lcd_ctl_major, slot), lcd_ctl, "bmi_lcd_ctl_m%i", slot+1);  
								     
	if (IS_ERR(lcd_ctl->class_dev)) {                                
		printk(KERN_ERR "Unable to create "                  
		       "class_device for bmi_lcd_ctl_m%i; errno = %ld\n",
		       slot+1, PTR_ERR(lcd_ctl->class_dev));             
		lcd_ctl->class_dev = NULL;                               
	}
	lcd_ctl->slot = slot;

	return ret;
}

void lcd_ctl_remove (struct lcd_ctl *lcd_ctl, int slot) 
{
	struct class *bmi_class;

	bmi_class = bmi_get_bmi_class ();
	device_destroy (bmi_class, MKDEV(lcd_ctl_major, slot));

	lcd_ctl->class_dev = 0;

	cdev_del (&lcd_ctl->cdev);
	return;
}


static int lcd_ctl_open (struct inode *inode, struct file *file)
{
	struct lcd_ctl *lcd_ctl;
	
	lcd_ctl = container_of(inode->i_cdev, struct lcd_ctl, cdev);
	

	// Save ctl pointer for later.

	file->private_data = lcd_ctl;
	return 0;
}

static int lcd_ctl_release (struct inode *inode, struct file *file)
{
	struct lcd_ctl *lcd_ctl;

	lcd_ctl = container_of(inode->i_cdev, struct lcd_ctl, cdev);
	return 0;
}


/*// ioctl
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
			*iox_data = (*iox_data & 0xFC) | bl;
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
*/

static int lcd_ctl_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct lcd_ctl *lcd_ctl;
	int slot;
	
	lcd_ctl = container_of(inode->i_cdev, struct lcd_ctl, cdev);
	slot = lcd_ctl->slot;
	if (slot < 0) {
		return -ENODEV;
	}	

	
	switch (cmd) {

	case BMI_LCD_RLEDOFF:
		bmi_slot_gpio_write_bit (slot, 3, 1); //Red LED Off
		break;

	case BMI_LCD_RLEDON:
		bmi_slot_gpio_write_bit (slot, 3, 0); //Red LED On
		break;

	case BMI_LCD_GLEDOFF:
		bmi_slot_gpio_write_bit (slot, 2, 1); //Green LED Off
		break;

	case BMI_LCD_GLEDON:
		bmi_slot_gpio_write_bit (slot, 2, 0); //Green LED On
		break;
		
	default:
		printk (KERN_ERR "lcd_ctl_ioctl() - error exit\n");
		return -ENOTTY;
	}
	
	return 0;
}

