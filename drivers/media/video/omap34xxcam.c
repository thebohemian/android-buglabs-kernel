/*
 * omap34xxcam.c
 *
 * Copyright (C) 2006--2010 Nokia Corporation
 * Copyright (C) 2007--2009 Texas Instruments
 *
 * Contact: Sakari Ailus <sakari.ailus@nokia.com>
 *          Tuukka Toivonen <tuukka.o.toivonen@nokia.com>
 *
 * Originally based on the OMAP 2 camera driver.
 *
 * Written by Sakari Ailus <sakari.ailus@nokia.com>
 *            Tuukka Toivonen <tuukka.o.toivonen@nokia.com>
 *            Sergio Aguirre <saaguirre@ti.com>
 *            Mohit Jalori
 *            Sameer Venkatraman
 *            Leonides Martinez
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

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <media/v4l2-common.h>
#include <media/v4l2-device.h>

#include "omap34xxcam.h"
#include "isp/isp.h"

/*
 * omap34xxcam_register_subdevs - Register a group of subdevices
 * @cam: OMAP34xx camera device
 * @board_info: I2C subdevs board information array
 *
 * Register all I2C subdevices in the board_info array. The array must be
 * terminated by a NULL entry, and the first entry must be the sensor.
 *
 * Return a pointer to the sensor media entity if it has been successfully
 * registered, or NULL otherwise.
 */
static struct media_entity *
omap34xxcam_register_subdevs(struct omap34xxcam_device *cam,
			     struct v4l2_subdev_i2c_board_info *board_info)
{
	struct media_entity *sensor = NULL;
	unsigned int first;

	if (board_info->board_info == NULL)
		return NULL;

	for (first = 1; board_info->board_info; ++board_info, first = 0) {
		struct v4l2_subdev *subdev;
		struct i2c_adapter *adapter;

		adapter = i2c_get_adapter(board_info->i2c_adapter_id);
		if (adapter == NULL) {
			printk(KERN_ERR "%s: Unable to get I2C adapter %d for "
				"device %s\n", __func__,
				board_info->i2c_adapter_id,
				board_info->board_info->type);
			continue;
		}

		subdev = v4l2_i2c_new_subdev_board(&cam->v4l2_dev,
				adapter, board_info->module_name,
				board_info->board_info, NULL);
		if (subdev == NULL) {
			printk(KERN_ERR "%s: Unable to register subdev %s\n",
				__func__, board_info->board_info->type);
			continue;
		}

		if (sensor == NULL && first)
			sensor = &subdev->entity;
	}

	return sensor;
}

static int omap34xxcam_remove(struct platform_device *pdev)
{
	struct omap34xxcam_platform_data *pdata = pdev->dev.platform_data;
	struct v4l2_device *vdev = platform_get_drvdata(pdev);
	struct omap34xxcam_device *cam = to_omap34xxcam_device(vdev);

	omap3isp_unregister_entities(pdata->isp);
	platform_device_unregister(pdata->isp);

	v4l2_device_unregister(&cam->v4l2_dev);
	media_device_unregister(&cam->media_dev);

	kfree(cam);
	return 0;
}

static int omap34xxcam_probe(struct platform_device *pdev)
{
	struct omap34xxcam_platform_data *pdata = pdev->dev.platform_data;
	struct omap34xxcam_subdevs_group *subdevs;
	struct omap34xxcam_device *cam;
	struct isp_device *isp = NULL;
	int ret;

	if (pdata == NULL)
		return -EINVAL;

	cam = kzalloc(sizeof(*cam), GFP_KERNEL);
	if (!cam) {
		printk(KERN_ERR "%s: could not allocate memory\n", __func__);
		return -ENOMEM;
	}

	cam->pdata = pdata;

	cam->media_dev.dev = &pdev->dev;
	ret = media_device_register(&cam->media_dev);
	if (ret < 0) {
		printk(KERN_ERR "%s: Media device registration failed (%d)\n",
			__func__, ret);
		goto err;
	}

	cam->v4l2_dev.mdev = &cam->media_dev;
	ret = v4l2_device_register(&pdev->dev, &cam->v4l2_dev);
	if (ret < 0) {
		printk(KERN_ERR "%s: V4L2 device registration failed (%d)\n",
			__func__, ret);
		goto err;
	}

	/* We need to check rval just once. The place is here. */
	platform_device_register(pdata->isp);

	cam->isp = platform_get_drvdata(pdata->isp);
	isp = isp_get(cam->isp);
	if (isp == NULL) {
		printk(KERN_ERR "%s: can't get ISP\n", __func__);
		ret = -EBUSY;
		goto err;
	}

	for (subdevs = pdata->subdevs; subdevs->subdevs; ++subdevs) {
		struct media_entity *sensor;
		struct media_entity *input;
		unsigned int flags;
		unsigned int pad;

		sensor = omap34xxcam_register_subdevs(cam, subdevs->subdevs);
		if (sensor == NULL)
			continue;

		/* Connect the sensor to the correct interface module. Parallel
		 * sensors are connected directly to the CCDC, while serial
		 * sensors are connected to the CSI2a, CCP2b or CSI2c receiver
		 * through CSIPHY1 or CSIPHY2.
		 */
		switch (subdevs->interface) {
		case ISP_INTERFACE_PARALLEL:
			input = &isp->isp_ccdc.subdev.entity;
			pad = CCDC_PAD_SINK;
			flags = 0;
			break;

		case ISP_INTERFACE_CSI2A_PHY2:
			input = &isp->isp_csi2a.subdev.entity;
			pad = CSI2_PAD_SINK;
			flags = MEDIA_LINK_FLAG_IMMUTABLE
			      | MEDIA_LINK_FLAG_ACTIVE;
			break;

		case ISP_INTERFACE_CCP2B_PHY1:
		case ISP_INTERFACE_CCP2B_PHY2:
			input = &isp->isp_ccp2.subdev.entity;
			pad = CCP2_PAD_SINK;
			flags = 0;
		break;

		case ISP_INTERFACE_CSI2C_PHY1:
			input = &isp->isp_csi2c.subdev.entity;
			pad = CSI2_PAD_SINK;
			flags = MEDIA_LINK_FLAG_IMMUTABLE
			      | MEDIA_LINK_FLAG_ACTIVE;
			break;

		default:
			printk(KERN_ERR "%s: invalid interface type %u\n",
			       __func__, subdevs->interface);
			ret = -EINVAL;
			goto err;
		}

		ret = media_entity_create_link(sensor, 0, input, pad, flags);
		if (ret < 0)
			goto err;
	}

	ret = omap3isp_register_entities(pdata->isp, &cam->v4l2_dev);
	if (ret < 0) {
		printk(KERN_ERR "%s: Can't register ISP subdevices (%d)\n",
			__func__, ret);
		goto err;
	}

	isp_put(isp);
	return 0;

err:
	isp_put(isp);
	omap34xxcam_remove(pdev);
	return ret;
}

#ifdef CONFIG_PM
static int omap34xxcam_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int omap34xxcam_resume(struct platform_device *pdev)
{
	return 0;
}
#else

#define omap34xxcam_suspend	NULL
#define omap34xxcam_resume	NULL

#endif /* CONFIG_PM */

static struct platform_driver omap34xxcam_driver = {
	.probe = omap34xxcam_probe,
	.remove = omap34xxcam_remove,
	.suspend = omap34xxcam_suspend,
	.resume = omap34xxcam_resume,
	.driver = {
		.name = CAM_NAME,
		.owner = THIS_MODULE,
	},
};

/*
 * Module initialisation and cleanup
 */

static int __init omap34xxcam_init(void)
{
	return platform_driver_register(&omap34xxcam_driver);
}

static void omap34xxcam_exit(void)
{
	platform_driver_unregister(&omap34xxcam_driver);
}

MODULE_AUTHOR("Sakari Ailus <sakari.ailus@nokia.com>");
MODULE_DESCRIPTION("OMAP34xx Video for Linux camera driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("omap34xxcam-mod");

module_init(omap34xxcam_init);
module_exit(omap34xxcam_exit);

