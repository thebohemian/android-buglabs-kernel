/*
 * drivers/bmi/pims/camera/bmi_li3m02cm.c
 *
 * Copyright (C) 2010 Lane Brooks
 *
 * Contact: Lane Brooks <dirjud@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#include <linux/bmi.h>
#include <linux/bmi/bmi-control.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/delay.h>

#include "../../../drivers/media/video/omap34xxcam.h"
#include "bmi_camera_mux.h"
#include "bmi_camera.h"
#include "mt9t111.h"

#define BMI_LI3M02CM_VERSION  "0.1.0.5"
extern struct platform_device omap3isp_device;

// BMI device ID table
static struct bmi_device_id bmi_li3m02cm_tbl[] = 
{ 
	{ .match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
	  .vendor   = BMI_VENDOR_BUG_LABS, 
	  .product  = BMI_PRODUCT_CAMERA_LI3M02CM, 
	  .revision = BMI_ANY, 
	}, 
	{ 0, },				      /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_li3m02cm_tbl);

int	bmi_li3m02cm_probe(struct bmi_device *bdev);
void 	bmi_li3m02cm_remove(struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_li3m02cm_driver = 
{
	.name = "bmi_li3m02cm", 
	.id_table = bmi_li3m02cm_tbl, 
	.probe   = bmi_li3m02cm_probe, 
	.remove  = bmi_li3m02cm_remove, 
	};


struct bmi_li3m02cm {
	struct bmi_device *bdev;		
	struct i2c_client *iox;
	struct i2c_client *mt9t111;
	struct v4l2_mbus_framefmt format;
	u8  sysfs_iox_i2c_addr;
	u16 sysfs_mt9t111_i2c_addr;
};

	// I2C Slave Address
#define BMI_IOX_I2C_ADDRESS	0x38	// 7-bit address

	// I2C IOX register addresses
#define IOX_INPUT_REG		0x0
#define IOX_OUTPUT_REG		0x1
#define IOX_POLARITY_REG	0x2
#define IOX_CONTROL		0x3

#define IOX_CAM_RESETB          0x80
#define IOX_GREEN_LED           0x40
#define IOX_SER_SYNC            0x20
#define IOX_FLASH_TORCHB        0x10
#define IOX_STROBE              0x08
#define IOX_CAM_STBY            0x04

#define GPIO_SER_EN             1
#define GPIO_FLASHON            2
#define GPIO_REDLED             3

// read byte from I2C IO expander

static struct i2c_board_info iox_info = {
	I2C_BOARD_INFO("CAM_IOX", BMI_IOX_I2C_ADDRESS),
};
static struct i2c_board_info mt9t111_info = {
	I2C_BOARD_INFO(MT9T111_NAME, MT9T111_I2C_ADDR),
};



static int ReadByte_IOX (struct i2c_client *client, unsigned char offset, unsigned char *data)
{
#ifdef REVB
        int    ret = 0;
        struct i2c_msg rmsg[2];

        /* Read Byte with Pointer */
        rmsg[0].addr = client->addr;
        rmsg[0].flags = 0;          /* write */
        rmsg[0].len = 1;
        rmsg[0].buf = &offset;

        rmsg[1].addr = client->addr;
        rmsg[1].flags = I2C_M_RD;   /* read */ 
        rmsg[1].len = 1;
        rmsg[1].buf = data;

        ret = i2c_transfer (client->adapter, rmsg, 2);

        if (ret == 2) {
		printk (KERN_ERR "ReadByte_IOX() - addr=0x%x data=0x%02X\n", offset, *data);
		
                ret = 0;
        }
        else {
                //Rework: add conditional debug messages here
		printk (KERN_ERR "ReadByte_IOX() - i2c_transfer failed\n");
                ret = -1;
        }
        return ret;
#else
	int     ret;
	if(offset == IOX_INPUT_REG || offset == IOX_OUTPUT_REG) {
		ret = i2c_master_recv(client, data, 1);
		if (ret < 0)
			printk (KERN_ERR "ReadByte_IOX() - i2c_transfer() failed...%d\n",ret);
		return ret;
	} else {
		*data = 0xFF;
		return 0;
	}
#endif
}

// write byte to I2C IO expander
static int WriteByte_IOX (struct i2c_client *client, unsigned char offset, unsigned char data)
{
#ifdef REVB
	int	ret = 0;
	unsigned char msg[2];
	printk (KERN_ERR "%s - addr=0x%x data=0x%02X\n", __func__, offset, data);
	
     	msg[0] = offset;
	msg[1] = data;
	ret = i2c_master_send(client, msg, sizeof(msg));
	
	if (ret < 0)
	  printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed...%d\n",ret);

	return ret;
#else
	int     ret = 0;
	if(offset == IOX_INPUT_REG || offset == IOX_OUTPUT_REG) {
		ret = i2c_master_send(client, &data, 1);
		if (ret < 0)
			printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed...%d\n",ret);
	}
	return ret;
#endif
}

static ssize_t show_iox_value(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	unsigned char iox_data;
	int ret;
	if(!cam)
		return sprintf(buf, "NULL camera handle\n");

	ret = ReadByte_IOX (cam->iox, cam->sysfs_iox_i2c_addr, &iox_data);
	if(ret < 0) 
		return sprintf(buf, "%d\n",   ret);
	else
		return sprintf(buf, "0x%02x\n", iox_data);
}
static ssize_t store_iox_value(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	long value;
	ssize_t status;
	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
		WriteByte_IOX(cam->iox, cam->sysfs_iox_i2c_addr, value);
	}
	return size;
}

static ssize_t show_iox_addr(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	return sprintf(buf, "0x%02x\n", cam->sysfs_iox_i2c_addr);
}

static ssize_t store_iox_addr(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	long value;
	ssize_t status;
	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
		cam->sysfs_iox_i2c_addr = value;
	}
	return size;
}

static ssize_t show_mt9t111_value(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	u16 val;
	int ret;
	if(!cam)
		return sprintf(buf, "NULL camera handle\n");

	ret = mt9t111_read_reg(cam->mt9t111, cam->sysfs_mt9t111_i2c_addr, &val);
	if(ret < 0) 
		return sprintf(buf, "%d\n",   ret);
	else
		return sprintf(buf, "0x%04x\n", val);
}
static ssize_t store_mt9t111_value(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	long value;
	ssize_t status;
	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
		mt9t111_write_reg(cam->mt9t111, cam->sysfs_mt9t111_i2c_addr, value);
	}
	return size;
}

static ssize_t show_mt9t111_addr(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	return sprintf(buf, "0x%04x\n", cam->sysfs_mt9t111_i2c_addr);
}

static ssize_t store_mt9t111_addr(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	long value;
	ssize_t status;
	status = strict_strtol(buf, 0, &value);
	if (status == 0) {
		cam->sysfs_mt9t111_i2c_addr = value;
	}
	return size;
}

static int li3m02cm_set_power(struct bmi_device *bdev, int on);

static ssize_t show_power(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "?\n");
}

static ssize_t store_power(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct bmi_li3m02cm *cam = dev_get_drvdata(dev);
	long value = 0;
	strict_strtol(buf, 0, &value);
	if(value)
		li3m02cm_set_power(cam->bdev, 1);
	else
		li3m02cm_set_power(cam->bdev, 0);
	return size;
}


static ssize_t show_serializer_locked(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", bmi_camera_mux_is_serializer_locked());
}

static DEVICE_ATTR(iox_value, S_IWUGO | S_IRUGO, show_iox_value, store_iox_value);
static DEVICE_ATTR(iox_addr,  S_IWUGO | S_IRUGO, show_iox_addr, store_iox_addr);
static DEVICE_ATTR(mt9t111_value, S_IWUGO | S_IRUGO, show_mt9t111_value, store_mt9t111_value);
static DEVICE_ATTR(mt9t111_addr,  S_IWUGO | S_IRUGO, show_mt9t111_addr, store_mt9t111_addr);
static DEVICE_ATTR(serializer_locked, S_IRUGO, show_serializer_locked, NULL);
static DEVICE_ATTR(set_power, S_IWUGO | S_IRUGO, show_power, store_power);

// configure IOX IO and states
void configure_IOX(struct bmi_li3m02cm *cam)
{	
	WriteByte_IOX(cam->iox, IOX_OUTPUT_REG, 0);
	WriteByte_IOX(cam->iox, IOX_CONTROL,    0x00); /* all outputs */
	WriteByte_IOX(cam->iox, IOX_OUTPUT_REG, 0);
}

// configure GPIO IO and states
void configure_GPIO(struct bmi_li3m02cm *cam)
{
	// set states before turning on outputs
	int slot = cam->bdev->slot->slotnum;
	bmi_slot_gpio_direction_out(slot, GPIO_REDLED,  0); // Red LED=OFF
	bmi_slot_gpio_direction_out(slot, GPIO_FLASHON, 0); // Flash LED=OFF
	bmi_slot_gpio_direction_out(slot, GPIO_SER_EN,  0); // SER_EN=OFF
}

// deconfigure IOX and GPIO
void deconfigure_module(struct bmi_li3m02cm *cam)
{
 	int slot = cam->bdev->slot->slotnum;
	bmi_slot_gpio_direction_in(slot, 3);
	bmi_slot_gpio_direction_in(slot, 2);
	bmi_slot_gpio_direction_in(slot, 1);
}

static int
li3m02cm_set_config(struct bmi_device *bdev, int irq, void *platform_data)
{
	struct bmi_li3m02cm *cam = bmi_device_get_drvdata(bdev);
		
	// configure GPIO and IOX
	configure_GPIO(cam);
	configure_IOX(cam);
	return 0;
}

static int li3m02cm_set_power(struct bmi_device *bdev, int on)
{
	struct bmi_li3m02cm *cam = bmi_device_get_drvdata(bdev);
	unsigned char iox_data;
	int ret;
	int slot = bdev->slot->slotnum;

	// setup the gpio pins
	bmi_slot_gpio_direction_out(slot, GPIO_FLASHON, 0);
	if(on) {
		bmi_slot_gpio_direction_out(slot, GPIO_SER_EN,  1);
		bmi_slot_gpio_direction_out(slot, GPIO_REDLED,  1);
	} else {
		bmi_slot_gpio_direction_out(slot, GPIO_SER_EN,  0);
		bmi_slot_gpio_direction_out(slot, GPIO_REDLED,  0);
	}

	// setup the io expander pins on the camera module board
	ret = ReadByte_IOX (cam->iox, IOX_OUTPUT_REG, &iox_data);
	if(ret < 0)
		return ret;

	// turn off green led. Turn it back on if turning on power
	// and everything goes OK.
	iox_data &= ~IOX_GREEN_LED;
	iox_data &= ~IOX_CAM_RESETB; // put part in reset regardless
	iox_data |=  IOX_CAM_STBY;   // and standby
	ret = WriteByte_IOX (cam->iox, IOX_OUTPUT_REG, iox_data);
	if(ret < 0)
		goto error;

	// now take part out reset and standby if a turn on is requested
	if(on) {
		iox_data |=  IOX_CAM_RESETB; // take out of reset 
		iox_data &= ~IOX_CAM_STBY;   // and standby
		iox_data |=  IOX_SER_SYNC;   // turn on serial sync
		ret = WriteByte_IOX (cam->iox, IOX_OUTPUT_REG, iox_data);
		if(ret < 0)
			goto error;
	}

	// turn on/off bug base camera stuff
	ret = bmi_camera_mux_set_power(on);
	if(ret < 0)
		goto error;

	// set power to the sensor
	ret = mt9t111_set_power(cam->mt9t111, on);
	if(ret < 0) {
		printk(KERN_ERR "%s error setting power to mt9t111 (err=%d)\n", __func__, ret);
		goto error;
	}
	ret = mt9t111_s_stream(cam->mt9t111, on);
	if(ret < 0)
		goto error;
	// initialize the current camera format
	cam->format.width  = 640;
	cam->format.height = 480;
	cam->format.code   = V4L2_MBUS_FMT_SGRBG10_1X10;
	cam->format.colorspace   = V4L2_COLORSPACE_SRGB;
	cam->format.field        = V4L2_FIELD_NONE;

	if(on) {
		// check if serializer/deserializer is locked
		ret = bmi_camera_mux_is_serializer_locked();
		if(ret < 0)
			goto error;
		if(!ret) {
			mdelay(100); // if not locked, wait and test again
			ret = bmi_camera_mux_is_serializer_locked();
			if(ret < 0)
				goto error;
			if(!ret) {
				printk(KERN_ERR "Camera serializer is not locked\n");
				ret = -EBUSY;
				goto error;
			}
		}

		// turn off the serial sync option and check if lock occurred
		iox_data &= ~IOX_SER_SYNC;  // turn off serial sync
		ret = WriteByte_IOX (cam->iox, IOX_OUTPUT_REG, iox_data);
		if(ret < 0)
			goto error;		

		printk(KERN_INFO "Camera serializer LOCKED\n");
	}
	return 0;

error:
	printk(KERN_ERR "%s error (err=%d)\n", __func__, ret);
	return ret;
}

static int li3m02cm_s_stream(struct bmi_device *bdev, int streaming)
{
	struct bmi_li3m02cm *cam = bmi_device_get_drvdata(bdev);
	unsigned char iox_data;
	int ret;
	int slot = bdev->slot->slotnum;

	ret = ReadByte_IOX (cam->iox, IOX_OUTPUT_REG, &iox_data);
	if(ret < 0)
		goto error;


	if(streaming) {
		// if we made it to here, then turn on the green LED saying
		// everything is good to go and turn off the red LED.
		iox_data |= IOX_GREEN_LED;
		ret = WriteByte_IOX (cam->iox, IOX_OUTPUT_REG, iox_data);
		bmi_slot_gpio_direction_out(slot, GPIO_REDLED,  0);
		if(ret < 0)
			goto error;
	} else {
		// otherwise they are turning off the stream, so we
		// turn on red led and off green
		iox_data &= ~IOX_GREEN_LED;
		ret = WriteByte_IOX (cam->iox, IOX_OUTPUT_REG, iox_data);
		bmi_slot_gpio_direction_out(slot, GPIO_REDLED,  1);
		if(ret < 0)
			goto error;
	}

error:
	return ret;
}

static int li3m02cm_enum_format(struct bmi_device *s, struct v4l2_fmtdesc *fmt)
{
	return 0;
}

static int li3m02cm_get_format(struct bmi_device *subdev,
			     struct v4l2_format *f)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int li3m02cm_try_format(struct bmi_device *s, struct v4l2_format *f)
{
	return 0;
}

static int li3m02cm_set_format(struct bmi_device *s, struct v4l2_format *f)
{
	return 0;
}

static int li3m02cm_get_param(struct bmi_device *subdev,
			    struct v4l2_streamparm *a)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int li3m02cm_set_param(struct bmi_device *subdev,
			    struct v4l2_streamparm *a)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int li3m02cm_enum_frame_sizes(struct bmi_device *s,
					struct v4l2_frmsizeenum *frms)
{
	return 0;
}

static int li3m02cm_enum_frame_intervals(struct bmi_device *s,
					struct v4l2_frmivalenum *frmi)
{
	return 0;
}

static int
li3m02cm_get_chip_ident(struct bmi_device *subdev,
		      struct v4l2_dbg_chip_ident *chip)
{
	return 0;
}


static int li3m02cm_query_ctrl(struct bmi_device *subdev,
				  struct v4l2_queryctrl *a)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int li3m02cm_query_menu(struct bmi_device *subdev,
				  struct v4l2_querymenu *qm)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int li3m02cm_get_ctrl(struct bmi_device *subdev,
			       struct v4l2_control *vc)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int li3m02cm_set_ctrl(struct bmi_device *subdev,
			       struct v4l2_control *vc)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}


static int li3m02cm_enum_frame_size(struct bmi_device *subdev,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int li3m02cm_enum_frame_ival(struct bmi_device *subdev,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_interval_enum *fie)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}
static int li3m02cm_enum_mbus_code(struct bmi_device *subdev,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_pad_mbus_code_enum *code)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}


static struct v4l2_mbus_framefmt *
__li3m02cm_get_pad_format(struct bmi_li3m02cm *cam, struct v4l2_subdev_fh *fh,
			unsigned int pad, enum v4l2_subdev_format which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_PROBE:
		return v4l2_subdev_get_probe_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &cam->format;
	default: 
		return NULL;
	}
}

static int li3m02cm_get_pad_format(struct bmi_device *bdev,
				 struct v4l2_subdev_fh *fh, unsigned int pad,
				 struct v4l2_mbus_framefmt *fmt,
				 enum v4l2_subdev_format which)
{
	struct bmi_li3m02cm *cam = bmi_device_get_drvdata(bdev);
	struct v4l2_mbus_framefmt *format;
	printk(KERN_INFO "%s enter\n", __func__);
		
	format = __li3m02cm_get_pad_format(cam, fh, pad, which);
	if (format == NULL)
		return -EINVAL;

	*fmt = *format;
	return 0;
}

static int li3m02cm_set_pad_format(struct bmi_device *bdev,
				 struct v4l2_subdev_fh *fh, unsigned int pad,
				 struct v4l2_mbus_framefmt *fmt,
				 enum v4l2_subdev_format which)
{
	struct bmi_li3m02cm *cam = bmi_device_get_drvdata(bdev);
	struct v4l2_mbus_framefmt *format;
	int ret;
	printk(KERN_INFO "%s enter\n", __func__);
	ret = mt9t111_set_format(cam->mt9t111, &fmt->width, &fmt->height);
	if(ret < 0)
		return ret;
	cam->format.width  = fmt->width;
	cam->format.height = fmt->height;
	return 0;
}

static const struct bmi_camera_video_ops li3m02cm_video_ops = {
	.s_stream            = li3m02cm_s_stream,
	.enum_fmt            = li3m02cm_enum_format,
	.g_fmt               = li3m02cm_get_format,
	.try_fmt             = li3m02cm_try_format,
	.s_fmt               = li3m02cm_set_format,
	.g_parm              = li3m02cm_get_param,
	.s_parm              = li3m02cm_set_param,
        .enum_framesizes     = li3m02cm_enum_frame_sizes,
        .enum_frameintervals = li3m02cm_enum_frame_intervals,
};

static const struct bmi_camera_core_ops li3m02cm_core_ops = {
	.g_chip_ident = li3m02cm_get_chip_ident,
	.s_config     = li3m02cm_set_config,
	.queryctrl    = li3m02cm_query_ctrl,
	.querymenu    = li3m02cm_query_menu,
	.g_ctrl       = li3m02cm_get_ctrl,
	.s_ctrl       = li3m02cm_set_ctrl,
	.s_power      = li3m02cm_set_power,
};

static const struct bmi_camera_pad_ops li3m02cm_pad_ops = {
	.enum_mbus_code      = li3m02cm_enum_mbus_code,
        .enum_frame_size     = li3m02cm_enum_frame_size,
        .enum_frame_interval = li3m02cm_enum_frame_ival,
	.get_fmt             = li3m02cm_get_pad_format,
	.set_fmt             = li3m02cm_set_pad_format,
};

static struct bmi_camera_ops li3m02cm_ops = {
	.core  = &li3m02cm_core_ops,
	.video = &li3m02cm_video_ops,
	.pad   = &li3m02cm_pad_ops,
};


int bmi_li3m02cm_probe(struct bmi_device *bdev)
{	
	struct bmi_li3m02cm *bmi_li3m02cm;
	int ret=0;

	bmi_li3m02cm = kzalloc(sizeof(*bmi_li3m02cm), GFP_KERNEL);
	if (!bmi_li3m02cm) {
	     return -1;
	}
	
	bmi_device_set_drvdata(bdev, bmi_li3m02cm);
	bmi_li3m02cm->bdev = bdev;
	bmi_li3m02cm->iox = i2c_new_device(bdev->slot->adap, &iox_info);
	bmi_li3m02cm->mt9t111 = i2c_new_device(bdev->slot->adap, &mt9t111_info);

	bmi_register_camera(bdev, &li3m02cm_ops);

	// These can be removed after driver stabalizes. They are for debug now.
	ret = device_create_file(&bdev->dev, &dev_attr_serializer_locked);
	ret = device_create_file(&bdev->dev, &dev_attr_iox_value);
	ret = device_create_file(&bdev->dev, &dev_attr_iox_addr);
	ret = device_create_file(&bdev->dev, &dev_attr_mt9t111_value);
	ret = device_create_file(&bdev->dev, &dev_attr_mt9t111_addr);
	ret = device_create_file(&bdev->dev, &dev_attr_set_power);
	return 0;
}

void bmi_li3m02cm_remove(struct bmi_device *bdev)
{	
	int slot;
	struct bmi_li3m02cm *bmi_li3m02cm;

	bmi_li3m02cm = (struct bmi_li3m02cm*)(bmi_device_get_drvdata (bdev));
	slot = bdev->slot->slotnum;
	
	bmi_unregister_camera(bdev);

	i2c_unregister_device(bmi_li3m02cm->iox);
	i2c_unregister_device(bmi_li3m02cm->mt9t111);

	kfree (bmi_li3m02cm);
	return;
}

static __init int bmi_li3m02cm_init(void)
{	
	printk(KERN_INFO "BMI LI3M02CM Camera Sensor Driver v%s \n", BMI_LI3M02CM_VERSION);
//	Register with BMI bus.
	return  bmi_register_driver(&bmi_li3m02cm_driver); 

}

static void __exit bmi_li3m02cm_cleanup(void)
{	
//	UnRegister with BMI bus.
	bmi_unregister_driver(&bmi_li3m02cm_driver);
	return;
}


module_init(bmi_li3m02cm_init);
module_exit(bmi_li3m02cm_cleanup);

MODULE_AUTHOR("Ubixum, Inc.");
MODULE_DESCRIPTION("LI3M02CM Camera Driver");
MODULE_LICENSE("GPL");
