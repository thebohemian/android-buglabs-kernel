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

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/v4l2-mediabus.h>

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "bmi_camera_mux.h"
#include "bmi_camera.h"

static int bmi_camera_s_stream(struct v4l2_subdev *subdev, int streaming)
{
	int ret;
	struct bmi_camera_ops *ops = NULL;
	struct bmi_device *bdev = NULL;
	ret = bmi_camera_mux_get_selected(&ops, &bdev);
	if(ret < 0) 
		return ret;
	if(ops->video && ops->video->s_stream)
		return ops->video->s_stream(bdev, streaming);
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

/* --------------------------------------------------------------------------
 * sysfs attributes
 */
static ssize_t
bmi_camera_priv_mem_read(struct device *dev, struct device_attribute *attr,
		     char *buf)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 100; //BMI_CAMERA_PRIV_MEM_SIZE;
}
static DEVICE_ATTR(priv_mem, S_IRUGO, bmi_camera_priv_mem_read, NULL);

static int
bmi_camera_get_chip_ident(struct v4l2_subdev *subdev,
		      struct v4l2_dbg_chip_ident *chip)
{
	printk(KERN_INFO "%s enter\n", __func__);
	return 0;
}

static int
bmi_camera_set_config(struct v4l2_subdev *subdev, int irq, void *platform_data)
{
	int ret;
	struct bmi_camera_ops *ops = NULL;
	struct bmi_device *bdev = NULL;
	printk(KERN_INFO "%s enter\n", __func__);
	ret = bmi_camera_mux_get_selected(&ops, &bdev);
	if(ret < 0) 
		return ret;
	if(ops->core && ops->core->s_config)
		return ops->core->s_config(bdev, irq, platform_data);
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
	int ret;
	struct bmi_camera_ops *ops = NULL;
	struct bmi_device *bdev = NULL;
	printk(KERN_INFO "%s enter on=%d\n", __func__, on);
	ret = bmi_camera_mux_get_selected(&ops, &bdev);
	if(ret < 0) 
		return ret;
	if(ops->core && ops->core->s_power)
		return ops->core->s_power(bdev, on);
	return 0;
}

#define MAX_FMTS 1
static int bmi_camera_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	printk(KERN_INFO "%s enter index=%d\n", __func__, fse->index);
	if (fse->index >= MAX_FMTS)
		return -EINVAL;
	fse->min_width  = 640; //2036;
	fse->min_height = 480; //1536;
	fse->max_width  = 640; //2036;
	fse->max_height = 480; //1536;
	return 0;
}

static int bmi_camera_enum_frame_ival(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_interval_enum *fie)
{

	printk(KERN_INFO "%s enter index=%d\n", __func__, fie->index);
	if (fie->index >= MAX_FMTS)
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator= 15;
	return 0;
}


static int bmi_camera_enum_mbus_code(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_pad_mbus_code_enum *code)
{
	printk(KERN_INFO "%s enter index=%d\n", __func__, code->index);

	if (code->index >= MAX_FMTS)
		return -EINVAL;
	
	code->code = V4L2_MBUS_FMT_SGRBG10_1X10;
	return 0;
}

static int bmi_camera_get_pad_format(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_fh *fh, unsigned int pad,
				 struct v4l2_mbus_framefmt *fmt,
				 enum v4l2_subdev_format which)
{
	int ret;
	struct bmi_camera_ops *ops = NULL;
	struct bmi_device *bdev = NULL;
	printk(KERN_INFO "%s enter which=%d\n", __func__, which);
	ret = bmi_camera_mux_get_selected(&ops, &bdev);
	if(ret < 0) 
		return ret;
	if(!(ops->pad && ops->pad->get_fmt))
		return -EINVAL;

	ret = ops->pad->get_fmt(bdev, fh, pad, fmt, which);
	if(ret < 0) {
		printk(KERN_INFO "%s error\n", __func__);
		return ret;
	}
	return 0;
}

static int bmi_camera_set_pad_format(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_fh *fh, unsigned int pad,
				 struct v4l2_mbus_framefmt *fmt,
				 enum v4l2_subdev_format which)
{
	int ret;
	struct bmi_camera_ops *ops = NULL;
	struct bmi_device *bdev = NULL;
	printk(KERN_INFO "%s enter which=%d (%d x %d)\n", __func__, which, fmt->width, fmt->height);
	ret = bmi_camera_mux_get_selected(&ops, &bdev);
	if(ret < 0) 
		return ret;
	if(!(ops->pad && ops->pad->set_fmt))
		return -EINVAL;

	ret = ops->pad->set_fmt(bdev, fh, pad, fmt, which);
	if(ret < 0) {
		printk(KERN_INFO "%s error\n", __func__);
		return ret;
	}
	return 0;
}

static int bmi_camera_get_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_frame_interval *fi)
{
	printk(KERN_INFO "%s enter\n", __func__);
	memset(fi, 0, sizeof(*fi));
	fi->interval.numerator=1;
	fi->interval.denominator=15;

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
        .enum_frame_interval = bmi_camera_enum_frame_ival,
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

#define to_bmi_camera_sensor(sd) container_of(sd, struct bmi_camera_sensor, subdev)
struct bmi_camera_sensor {
	struct v4l2_subdev subdev;
	struct media_entity_pad pad;
};

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
	ret = media_entity_init(&sensor->subdev.entity, 1, &sensor->pad, 0);
	if (ret < 0)
		kfree(sensor);
	return ret;
}

static int __exit bmi_camera_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct bmi_camera_sensor *sensor = to_bmi_camera_sensor(subdev);

	v4l2_device_unregister_subdev(&sensor->subdev);
	device_remove_file(&client->dev, &dev_attr_priv_mem);
	kfree(sensor);
	return 0;
}

#define BMI_CAMERA_NAME "bmi_camera"
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

static int __init bmi_camera_init(void)
{
	int rval;
	rval = i2c_add_driver(&bmi_camera_i2c_driver);
	if (rval)
		printk(KERN_INFO "%s: failed registering " BMI_CAMERA_NAME "\n",
		       __func__);

	return rval;
}

static void __exit bmi_camera_exit(void)
{
	i2c_del_driver(&bmi_camera_i2c_driver);
}

module_init(bmi_camera_init);
module_exit(bmi_camera_exit);

MODULE_AUTHOR("Lane Brooks");
MODULE_DESCRIPTION("BMI_CAMERA camera I2C sensor driver");
MODULE_LICENSE("GPL");
