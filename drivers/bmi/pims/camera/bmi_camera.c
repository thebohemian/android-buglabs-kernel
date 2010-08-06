/*
 * drivers/bmi/pims/camera/bmi_camera.c
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
#include <linux/platform_device.h>
#include <linux/mm.h>
#include "../../../drivers/media/video/isp/isp.h"
#include "../../../drivers/media/video/isp/ispreg.h"
#include <linux/gpio.h>

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/v4l2-mediabus.h>

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "bmi_camera.h"

extern struct platform_device omap3isp_device;


/* We declare this module as an i2c device simply because the
 * omap3-isp requires a i2c subdev. In reality this module is
 * simply a mux between any bug camera modules that plug
 * and register as such. It registers a 'fake' i2c address
 * only because it has to. */
static struct i2c_board_info bmi_camera_i2c_devices[] = {
	{
		I2C_BOARD_INFO(BMI_CAMERA_NAME, BMI_CAMERA_I2C_ADDR),
		.platform_data = NULL,
	},
};

static struct v4l2_subdev_i2c_board_info bmi_camera_primary_subdevs[] = {
	{
		.board_info = &bmi_camera_i2c_devices[0],
		.i2c_adapter_id = 3,
		.module_name = BMI_CAMERA_NAME,
	},
	{ NULL, 0, NULL, },
};

static struct isp_subdevs_group bmi_camera_subdevs[] = {
	{ bmi_camera_primary_subdevs, ISP_INTERFACE_PARALLEL, },
	{ NULL, 0, },
};


static struct isp_platform_data bmi_isp_platform_data = {
	.parallel = {
		.data_lane_shift	= 3,
		.clk_pol		= 0,
		.bridge                 = ISPCTRL_PAR_BRIDGE_DISABLE,
		//.bridge                 = ISPCTRL_PAR_BRIDGE_LENDIAN, 
		//.bridge                 = ISPCTRL_PAR_BRIDGE_BENDIAN, 
	},
	.subdevs = bmi_camera_subdevs,
};


static struct bmi_camera_selector bmi_camera_sel;

static int bmi_camera_get_selected_ops(struct v4l2_subdev_ops **ops, 
				       struct bmi_device **bdev) {
	int ret=0;
	mutex_lock(&bmi_camera_sel.mutex);
	if(bmi_camera_sel.selected < 0) {
		ret = -EINVAL;
		goto out;
	}
	*ops  = bmi_camera_sel.ops[bmi_camera_sel.selected];
	*bdev = bmi_camera_sel.bdev[bmi_camera_sel.selected];
	if(*ops == NULL || *bdev == NULL)
		ret = -EINVAL;
out:
	mutex_unlock(&bmi_camera_sel.mutex);
	return ret;
}

static int bmi_camera_get_selected_slot(void) {
	int ret=0;
	mutex_lock(&bmi_camera_sel.mutex);
	ret = bmi_camera_sel.selected;
	mutex_unlock(&bmi_camera_sel.mutex);
	return ret;
}

static int bmi_camera_set_selected_slot(int slotnum) {
	int ret;
	if(slotnum < 0 || slotnum > 3)
		return -EINVAL;
	mutex_lock(&bmi_camera_sel.mutex);
	if(!bmi_camera_sel.bdev[slotnum]) {
		ret = -EINVAL;
	} else {
		bmi_camera_sel.selected = slotnum;
		ret = 0;
	}
	mutex_unlock(&bmi_camera_sel.mutex);
	return ret;
}

static int bmi_camera_select_available_slot(void) {
	int ret = -EINVAL, slotnum;
	mutex_lock(&bmi_camera_sel.mutex);
	for(slotnum=0; slotnum<4; slotnum++) { // cycle to find next avail slot
		if(bmi_camera_sel.bdev[slotnum]) {
			bmi_camera_sel.selected = slotnum;
			ret = 0;
			break;
		}
	}
	if(ret == -EINVAL) {
		bmi_camera_sel.selected = -1;
	}
	mutex_unlock(&bmi_camera_sel.mutex);
	return ret;
}

int bmi_register_camera(struct bmi_device *bdev, struct v4l2_subdev_ops *ops)
{
	int rval = 0;
	int slotnum = bdev->slot->slotnum;

	mutex_lock(&bmi_camera_sel.mutex);
	bmi_camera_sel.bdev[slotnum] = bdev;
	bmi_camera_sel.ops[slotnum]  = ops;

	// first camera plugged in gets selected
	if(bmi_camera_sel.selected == -1)
		bmi_camera_sel.selected = slotnum;
	bmi_camera_sel.count++;

	mutex_unlock(&bmi_camera_sel.mutex);
	printk(KERN_INFO "%s registering camera in slot %d: return cord = %d\n", __func__, slotnum, rval);
	return rval;
}
EXPORT_SYMBOL(bmi_register_camera);

int bmi_unregister_camera(struct bmi_device *bdev)
{
	int slotnum = bdev->slot->slotnum;
	mutex_lock(&bmi_camera_sel.mutex);
	bmi_camera_sel.bdev[slotnum] = NULL;
	bmi_camera_sel.ops[slotnum]  = NULL;
	bmi_camera_sel.count--;
	bmi_camera_sel.selected = -1;
	mutex_unlock(&bmi_camera_sel.mutex);
	bmi_camera_select_available_slot();
	return 0;
}
EXPORT_SYMBOL(bmi_unregister_camera);




#define CAM_OSC_EN  37
#define CAM_REN     34
#define CAM_RCLK_RF 38
#define CAM_BUF_OEN 98
#define CAM_LOCKB  167

int bmi_camera_is_serdes_locked() {
	int val;
	val = gpio_get_value(CAM_LOCKB);
	if(val < 0)
		return val;
	else
		return !val;
}
EXPORT_SYMBOL(bmi_camera_is_serdes_locked);

int bmi_camera_set_power_bugbase(int on) {
	if(on) {
		gpio_set_value(CAM_OSC_EN, 1);
		gpio_set_value(CAM_REN,    1);
	} else {
		gpio_set_value(CAM_OSC_EN, 0);
		gpio_set_value(CAM_REN,    0);
	}		
	return 0;
}

static int setup_gpio(unsigned gpio, int value) {
	int ret;
	ret = gpio_request(gpio,  BMI_CAMERA_NAME);
	if(ret < 0)
		return ret;
	return gpio_direction_output(gpio, value);
}


static ssize_t show_slot(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", bmi_camera_get_selected_slot());
}

static ssize_t store_slot(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	sscanf(buf, "%d", &value);
	bmi_camera_set_selected_slot(value);
	return size;
}

static ssize_t show_serdes_locked(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", bmi_camera_is_serdes_locked());
}

static DEVICE_ATTR(slot, S_IWUGO | S_IRUGO, show_slot, store_slot);
static DEVICE_ATTR(serdes_locked, S_IRUGO, show_serdes_locked, NULL);

#define GET_SELECTED_OPS                                                 \
	int ret;                                                         \
	struct v4l2_subdev_ops *ops = NULL;                               \
	struct bmi_device *bdev = NULL;                                  \
	struct bmi_camera_sensor *sensor = to_bmi_camera_sensor(subdev); \
	ret = bmi_camera_get_selected_ops(&ops, &bdev);                  \
	if(ret < 0)                                                      \
		return ret;  						\
	sensor->bdev = bdev

static int bmi_camera_s_stream(struct v4l2_subdev *subdev, int streaming)
{
	GET_SELECTED_OPS;
	if(ops->video && ops->video->s_stream)
		return ops->video->s_stream(subdev, streaming);
	return 0;
}

static int bmi_camera_enum_format(struct v4l2_subdev *s, struct v4l2_fmtdesc *fmt)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int bmi_camera_get_format(struct v4l2_subdev *subdev,
			     struct v4l2_format *f)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int bmi_camera_try_format(struct v4l2_subdev *s, struct v4l2_format *f)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int bmi_camera_set_format(struct v4l2_subdev *s, struct v4l2_format *f)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int bmi_camera_get_param(struct v4l2_subdev *subdev,
			    struct v4l2_streamparm *a)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int bmi_camera_set_param(struct v4l2_subdev *subdev,
			    struct v4l2_streamparm *a)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int bmi_camera_enum_frame_sizes(struct v4l2_subdev *s,
					struct v4l2_frmsizeenum *frms)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int bmi_camera_enum_frame_intervals(struct v4l2_subdev *s,
					struct v4l2_frmivalenum *frmi)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int bmi_camera_get_chip_ident(struct v4l2_subdev *subdev,
				     struct v4l2_dbg_chip_ident *chip)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int
bmi_camera_set_config(struct v4l2_subdev *subdev, int irq, void *platform_data)
{
	return 0;
}

static int bmi_camera_query_ctrl(struct v4l2_subdev *subdev,
				  struct v4l2_queryctrl *a)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int bmi_camera_query_menu(struct v4l2_subdev *subdev,
				  struct v4l2_querymenu *qm)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int bmi_camera_get_ctrl(struct v4l2_subdev *subdev,
			       struct v4l2_control *vc)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int bmi_camera_set_ctrl(struct v4l2_subdev *subdev,
			       struct v4l2_control *vc)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int bmi_camera_set_power(struct v4l2_subdev *subdev, int on)
{
	GET_SELECTED_OPS;
	if(ops->core && ops->core->s_power)
		return ops->core->s_power(subdev, on);
	return 0;
}

#define MAX_FMTS 1
static int bmi_camera_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	GET_SELECTED_OPS;
	if(ops->pad && ops->pad->enum_frame_size)
		return ops->pad->enum_frame_size(bdev, fh, fse);
	return -EINVAL;
}

static int bmi_camera_enum_frame_interval(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_interval_enum *fie)
{
	GET_SELECTED_OPS;
	if(ops->pad && ops->pad->enum_frame_interval)
		return ops->pad->enum_frame_interval(subdev, fh, fie);
	return -EINVAL;
}


static int bmi_camera_enum_mbus_code(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_pad_mbus_code_enum *code)
{
	GET_SELECTED_OPS;
	if(ops->pad && ops->pad->enum_mbus_code)
		return ops->pad->enum_mbus_code(subdev, fh, code);
	return -EINVAL;
}

static int bmi_camera_get_pad_format(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_fh *fh, unsigned int pad,
				 struct v4l2_mbus_framefmt *fmt,
				 enum v4l2_subdev_format which)
{
	GET_SELECTED_OPS;
	if(ops->pad && ops->pad->get_fmt) {
		return 	ops->pad->get_fmt(subdev, fh, pad, fmt, which);
	}
	return -EINVAL;
}

static int bmi_camera_set_pad_format(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_fh *fh, unsigned int pad,
				 struct v4l2_mbus_framefmt *fmt,
				 enum v4l2_subdev_format which)
{
	GET_SELECTED_OPS;
	if(ops->pad && ops->pad->set_fmt)
		return ops->pad->set_fmt(subdev, fh, pad, fmt, which);
	return -EINVAL;
}

static int bmi_camera_get_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_frame_interval *fi)
{
	printk(KERN_INFO "%s enter\n", __func__);
//	BUG_ON(1);
//	memset(fi, 0, sizeof(*fi));
//	fi->interval.numerator=1;
//	fi->interval.denominator=15;
	return 0;
}

static int bmi_camera_set_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_frame_interval *fi)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static const struct v4l2_subdev_video_ops bmi_camera_video_ops = {
	.s_stream            = bmi_camera_s_stream,
	.g_frame_interval    = bmi_camera_get_frame_interval,
	.s_frame_interval    = bmi_camera_set_frame_interval,
	.enum_fmt            = bmi_camera_enum_format,
	.g_fmt               = bmi_camera_get_format,
	.try_fmt             = bmi_camera_try_format,
	.s_fmt               = bmi_camera_set_format,
	.g_parm              = bmi_camera_get_param,
	.s_parm              = bmi_camera_set_param,
        .enum_framesizes     = bmi_camera_enum_frame_sizes,
        .enum_frameintervals = bmi_camera_enum_frame_intervals,
};

static const struct v4l2_subdev_core_ops bmi_camera_core_ops = {
	.g_chip_ident = bmi_camera_get_chip_ident,
	.s_config     = bmi_camera_set_config,
	.queryctrl    = bmi_camera_query_ctrl,
	.querymenu    = bmi_camera_query_menu,
	.g_ctrl       = bmi_camera_get_ctrl,
	.s_ctrl       = bmi_camera_set_ctrl,
	.s_power      = bmi_camera_set_power,
};

static const struct v4l2_subdev_pad_ops bmi_camera_pad_ops = {
	.enum_mbus_code      = bmi_camera_enum_mbus_code,
        .enum_frame_size     = bmi_camera_enum_frame_size,
        .enum_frame_interval = bmi_camera_enum_frame_interval,
	.get_fmt             = bmi_camera_get_pad_format,
	.set_fmt             = bmi_camera_set_pad_format,
};

static const struct v4l2_subdev_ops bmi_camera_ops = {
	.core  = &bmi_camera_core_ops,
	.video = &bmi_camera_video_ops,
	.pad   = &bmi_camera_pad_ops,
};

static const struct media_entity_operations bmi_camera_entity_ops = {
	.set_power = v4l2_subdev_set_power,
};

#define bmi_camera_suspend	NULL
#define bmi_camera_resume	NULL


static int bmi_camera_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct bmi_camera_sensor *sensor;
	int ret;

	sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
	if (sensor == NULL)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&sensor->subdev, client, &bmi_camera_ops);

	sensor->pad.type = MEDIA_PAD_TYPE_OUTPUT;
	sensor->subdev.entity.ops = &bmi_camera_entity_ops;
	sensor->bdev = NULL;
	ret = media_entity_init(&sensor->subdev.entity, 1, &sensor->pad, 0);
	if (ret < 0) {
		kfree(sensor);
		return ret;
	}

	ret = device_create_file(&client->dev, &dev_attr_slot);
	ret |= device_create_file(&client->dev, &dev_attr_serdes_locked);
	return ret;
}

static int __exit bmi_camera_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct bmi_camera_sensor *sensor = to_bmi_camera_sensor(subdev);

	v4l2_device_unregister_subdev(&sensor->subdev);
	kfree(sensor);
	return 0;
}

static const struct i2c_device_id bmi_camera_id_table[] = {
	{ BMI_CAMERA_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bmi_camera_id_table);

static struct i2c_driver bmi_camera_i2c_driver = {
	.driver		= {
		.name	= BMI_CAMERA_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= bmi_camera_probe,
	.remove		= __exit_p(bmi_camera_remove),
	.suspend	= bmi_camera_suspend,
	.resume		= bmi_camera_resume,
	.id_table	= bmi_camera_id_table,
};

static __init int bmi_camera_init(void)
{	
	int ret, i;
	// initialize slot selection code
	mutex_init(&bmi_camera_sel.mutex);
	bmi_camera_sel.selected = -1;
	bmi_camera_sel.count = 0;
	for(i=0; i<4; i++) {
		bmi_camera_sel.bdev[i] = NULL;
		bmi_camera_sel.ops[i] = NULL;
	}

	ret = i2c_add_driver(&bmi_camera_i2c_driver);
	if (ret) {
		printk(KERN_ERR "%s: failed registering i2c driver" BMI_CAMERA_NAME "\n",
		       __func__);
		return ret;
	}

	omap3isp_device.dev.platform_data = &bmi_isp_platform_data;
	// unlock the mutex to prevent nested locking while registering
	ret = platform_device_register(&omap3isp_device);
	if(ret < 0) {
		printk(KERN_ERR "Error register omap3-isp\n");
		return ret;
	}
	printk(KERN_INFO "Successfully registered omap3-isp\n");

	ret = setup_gpio(CAM_OSC_EN, 1);
	if(ret < 0)
		return ret;
	ret = setup_gpio(CAM_REN, 1);
	if(ret < 0)
		return ret;
	ret = setup_gpio(CAM_RCLK_RF, 1);
	if(ret < 0)
		return ret;
	ret = setup_gpio(CAM_BUF_OEN, 0);
	if(ret < 0)
		return ret;
	ret = gpio_request(CAM_LOCKB, BMI_CAMERA_NAME);
	if(ret < 0)
		return ret;
	gpio_direction_input(CAM_LOCKB);

	bmi_camera_set_power_bugbase(1);

	return 0;
}

static void __exit bmi_camera_cleanup(void)
{	
	bmi_camera_set_power_bugbase(0);
	gpio_direction_output(98, 1); // CAM_OE#
	gpio_set_value(CAM_OSC_EN, 0);
	gpio_set_value(CAM_REN,    0);
	gpio_free(CAM_OSC_EN);
	gpio_free(CAM_REN);
	gpio_free(CAM_RCLK_RF);
	gpio_free(CAM_BUF_OEN);
	gpio_free(CAM_LOCKB);

	platform_device_unregister(&omap3isp_device);
	i2c_del_driver(&bmi_camera_i2c_driver);
}




module_init(bmi_camera_init);
module_exit(bmi_camera_cleanup);

MODULE_AUTHOR("Lane Brooks");
MODULE_DESCRIPTION("BMI Camera Driver");
MODULE_LICENSE("GPL");
