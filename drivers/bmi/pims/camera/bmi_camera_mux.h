/*
 * drivers/bmi/pims/camera/bmi_camera_mux.h
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

#ifndef __BMI_CAMERA_MUX_H
#define __BMI_CAMERA_MUX_H

struct mutex;
struct bmi_device;
struct bmi_camera_ops;

struct bmi_camera_selector {
	struct mutex mutex; /* atomic access to this structure */
	struct bmi_device *bdev[4];
	struct bmi_camera_ops *ops[4];
	struct i2c_client *stream_sel;
	int initialized;
	int selected;
	int count;
};
#define BMI_CAMERA_MUX_NAME     "BMI Camera"
#define BMI_CAMERA_MUX_I2C_ADDR 0x38

extern int bmi_camera_mux_get_selected(struct bmi_camera_ops **ops, struct bmi_device **bdev);
extern int bmi_register_camera(struct bmi_device *bdev, struct bmi_camera_ops *ops);
extern int bmi_unregister_camera(struct bmi_device *bdev);

extern int bmi_camera_mux_is_serializer_locked(void);
extern int bmi_camera_mux_set_power(int on);

#endif
