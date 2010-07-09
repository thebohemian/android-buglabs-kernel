/*
 * omap34xxcam.h
 *
 * Copyright (C) 2006--2009 Nokia Corporation
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

#ifndef OMAP34XXCAM_H
#define OMAP34XXCAM_H

#include <media/media-device.h>
#include <media/v4l2-device.h>
#include "isp/isp.h"

#define CAM_NAME			"omap34xxcam"

#define OMAP34XXCAM_XCLK_NONE		-1
#define OMAP34XXCAM_XCLK_A		0
#define OMAP34XXCAM_XCLK_B		1

struct omap34xxcam_subdevs_group {
	struct v4l2_subdev_i2c_board_info *subdevs;
	enum isp_interface_type interface;
};

struct omap34xxcam_platform_data {
	struct platform_device *isp;
	struct omap34xxcam_subdevs_group *subdevs;
};

/*
 * struct omap34xxcam_device - per-device data structure
 */
struct omap34xxcam_device {
	struct omap34xxcam_platform_data *pdata;
	struct v4l2_device v4l2_dev;
	struct media_device media_dev;
	struct isp_device *isp;
};

#define to_omap34xxcam_device(dev) \
	container_of(dev, struct omap34xxcam_device, v4l2_dev)

#endif /* ifndef OMAP34XXCAM_H */
