/*
 * drivers/bmi/pims/camera/bmi_camera_mux.c
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
#include "bmi_camera_mux.h"
#include "bmi_camera.h"
#include <linux/gpio.h>

extern struct platform_device omap3isp_device;

static struct i2c_board_info bmi_camera_mux_i2c_devices[] = {
	{
		I2C_BOARD_INFO("bmi_camera", BMI_CAMERA_MUX_I2C_ADDR),
		.platform_data = NULL,
	},
};

static struct v4l2_subdev_i2c_board_info bmi_camera_primary_subdevs[] = {
	{
		.board_info = &bmi_camera_mux_i2c_devices[0],
		.i2c_adapter_id = 3,
		.module_name = "bmi_camera_mux",
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
	},
	.subdevs = bmi_camera_subdevs,
};

static void platform_camera_release(struct device *dev)
{
}

static struct bmi_camera_selector bmi_camera_sel;

int bmi_register_camera(struct bmi_device *bdev, struct bmi_camera_ops *ops)
{
	int rval = 0;
	int slotnum = bdev->slot->slotnum;

	mutex_lock(&bmi_camera_sel.mutex);
	bmi_camera_sel.bdev[slotnum] = bdev;
	bmi_camera_sel.ops[slotnum]  = ops;

	/* FIXME: For now the first camera plugged in is selected with
	   no method to select the other cameras plugged */
	if(bmi_camera_sel.selected == -1)
		bmi_camera_sel.selected = slotnum;
	bmi_camera_sel.count++;
	
	/* We register with the omap34xxcam driver only after the
	   first device has been plugged in. */
	if(!bmi_camera_sel.initialized) {
		omap3isp_device.dev.platform_data = &bmi_isp_platform_data;
		rval = platform_device_register(&omap3isp_device);
		if(rval < 0) {
			printk(KERN_INFO "Error register omap34xxcam");
			goto out;
		}
		bmi_camera_sel.initialized = 1;
	}

	if(ops->core && ops->core->s_config) {
		rval = ops->core->s_config(bdev, 0, NULL);
		if(rval < 0)
			goto out;
	}

out:
	mutex_unlock(&bmi_camera_sel.mutex);
	printk(KERN_INFO "%s exit\n", __func__);
	return rval;
}
EXPORT_SYMBOL(bmi_register_camera);

int bmi_unregister_camera(struct bmi_device *bdev)
{
	int rval;
	int slotnum = bdev->slot->slotnum;
	
	mutex_lock(&bmi_camera_sel.mutex);

	/* FIXME: I don't know which module to select, so deselect everything */
	bmi_camera_sel.selected = -1;
	bmi_camera_sel.bdev[slotnum] = NULL;
	bmi_camera_sel.ops[slotnum]  = NULL;
	bmi_camera_sel.count--;
	
	rval = 0;
	mutex_unlock(&bmi_camera_sel.mutex);
	return rval;
}
EXPORT_SYMBOL(bmi_unregister_camera);

int bmi_camera_mux_get_selected(struct bmi_camera_ops **ops, struct bmi_device **bdev) {
	if(bmi_camera_sel.selected < 0) 
		return -EINVAL;
	*ops  = bmi_camera_sel.ops[bmi_camera_sel.selected];
	*bdev = bmi_camera_sel.bdev[bmi_camera_sel.selected];
	if(*ops == NULL || *bdev == NULL)
		return -EINVAL;
	return 0;
}
EXPORT_SYMBOL(bmi_camera_mux_get_selected);

#define CAM_OSC_EN  37
#define CAM_REN     34
#define CAM_RCLK_RF 38
#define CAM_BUF_OEN 98
#define CAM_LOCKB  167

int bmi_camera_mux_is_serializer_locked() {
	int val;
	val = gpio_get_value(CAM_LOCKB);
	if(val < 0)
		return val;
	else
		return !val;
}
EXPORT_SYMBOL(bmi_camera_mux_is_serializer_locked);

int bmi_camera_mux_set_power(int on) {
	if(on) {
		gpio_set_value(CAM_OSC_EN, 1);
		gpio_set_value(CAM_REN,    1);
	} else {
		gpio_set_value(CAM_OSC_EN, 0);
		gpio_set_value(CAM_REN,    0);
	}		
	return 0;
}
EXPORT_SYMBOL(bmi_camera_mux_set_power);

static int setup_gpio(unsigned gpio, int value) {
	int ret;
	ret = gpio_request(gpio,  "bmi_camera_mux");
	if(ret < 0)
		return ret;
	return gpio_direction_output(gpio, value);
}

static __init int bmi_camera_mux_init(void)
{	
	int ret;
	mutex_init(&bmi_camera_sel.mutex);
	bmi_camera_sel.selected = -1;
	bmi_camera_sel.count = 0;
	bmi_camera_sel.initialized = 0;
	bmi_camera_sel.bdev[0] = NULL;
	bmi_camera_sel.bdev[1] = NULL;
	bmi_camera_sel.bdev[2] = NULL;
	bmi_camera_sel.bdev[3] = NULL;

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
	ret = gpio_request(CAM_LOCKB, "bmi_camera_mux");
	if(ret < 0)
		return ret;
	gpio_direction_input(CAM_LOCKB);
	return 0;
}

static void __exit bmi_camera_mux_cleanup(void)
{	
	gpio_direction_output(98, 1); // CAM_OE#
	gpio_set_value(CAM_OSC_EN, 0);
	gpio_set_value(CAM_REN,    0);
	gpio_free(CAM_OSC_EN);
	gpio_free(CAM_REN);
	gpio_free(CAM_RCLK_RF);
	gpio_free(CAM_BUF_OEN);
	gpio_free(CAM_LOCKB);

	if(bmi_camera_sel.initialized) {
		bmi_camera_sel.initialized = 0;
		platform_device_unregister(&omap3isp_device);
	}

}

module_init(bmi_camera_mux_init);
module_exit(bmi_camera_mux_cleanup);

MODULE_AUTHOR("Lane Brooks");
MODULE_DESCRIPTION("BMI Camera Mux Driver");
MODULE_LICENSE("GPL");
