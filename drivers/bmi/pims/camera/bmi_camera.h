/*
 * drivers/bmi/pims/camera/bmi_camera.h
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
#ifndef _BMI_CAMERA_H
#define _BMI_CAMERA_H


struct bmi_camera_core_ops {
	int (*g_chip_ident)(struct bmi_device *sd, struct v4l2_dbg_chip_ident *chip);
	int (*log_status)(struct bmi_device *sd);
	int (*s_config)(struct bmi_device *sd, int irq, void *platform_data);
	int (*init)(struct bmi_device *sd, u32 val);
	int (*load_fw)(struct bmi_device *sd);
	int (*reset)(struct bmi_device *sd, u32 val);
	int (*s_gpio)(struct bmi_device *sd, u32 val);
	int (*queryctrl)(struct bmi_device *sd, struct v4l2_queryctrl *qc);
	int (*g_ctrl)(struct bmi_device *sd, struct v4l2_control *ctrl);
	int (*s_ctrl)(struct bmi_device *sd, struct v4l2_control *ctrl);
	int (*g_ext_ctrls)(struct bmi_device *sd, struct v4l2_ext_controls *ctrls);
	int (*s_ext_ctrls)(struct bmi_device *sd, struct v4l2_ext_controls *ctrls);
	int (*try_ext_ctrls)(struct bmi_device *sd, struct v4l2_ext_controls *ctrls);
	int (*querymenu)(struct bmi_device *sd, struct v4l2_querymenu *qm);
	int (*s_std)(struct bmi_device *sd, v4l2_std_id norm);
	long (*ioctl)(struct bmi_device *sd, unsigned int cmd, void *arg);
#ifdef CONFIG_VIDEO_ADV_DEBUG
	int (*g_register)(struct bmi_device *sd, struct v4l2_dbg_register *reg);
	int (*s_register)(struct bmi_device *sd, struct v4l2_dbg_register *reg);
#endif
	int (*s_power)(struct bmi_device *sd, int on);
	int (*subscribe_event)(struct bmi_device *sd, struct v4l2_fh *fh,
			       struct v4l2_event_subscription *sub);
	int (*unsubscribe_event)(struct bmi_device *sd, struct v4l2_fh *fh,
				 struct v4l2_event_subscription *sub);
};

struct bmi_camera_video_ops {
	int (*s_routing)(struct bmi_device *sd, u32 input, u32 output, u32 config);
	int (*s_crystal_freq)(struct bmi_device *sd, u32 freq, u32 flags);
	int (*s_std_output)(struct bmi_device *sd, v4l2_std_id std);
	int (*querystd)(struct bmi_device *sd, v4l2_std_id *std);
	int (*g_input_status)(struct bmi_device *sd, u32 *status);
	int (*s_stream)(struct bmi_device *sd, int enable);
	int (*enum_fmt)(struct bmi_device *sd, struct v4l2_fmtdesc *fmtdesc);
	int (*g_fmt)(struct bmi_device *sd, struct v4l2_format *fmt);
	int (*try_fmt)(struct bmi_device *sd, struct v4l2_format *fmt);
	int (*s_fmt)(struct bmi_device *sd, struct v4l2_format *fmt);
	int (*cropcap)(struct bmi_device *sd, struct v4l2_cropcap *cc);
	int (*g_crop)(struct bmi_device *sd, struct v4l2_crop *crop);
	int (*s_crop)(struct bmi_device *sd, struct v4l2_crop *crop);
	int (*g_parm)(struct bmi_device *sd, struct v4l2_streamparm *param);
	int (*s_parm)(struct bmi_device *sd, struct v4l2_streamparm *param);
	int (*g_frame_interval)(struct bmi_device *sd,
				struct v4l2_subdev_frame_interval *interval);
	int (*s_frame_interval)(struct bmi_device *sd,
				struct v4l2_subdev_frame_interval *interval);
	int (*enum_framesizes)(struct bmi_device *sd, struct v4l2_frmsizeenum *fsize);
	int (*enum_frameintervals)(struct bmi_device *sd, struct v4l2_frmivalenum *fival);
	int (*enum_dv_presets) (struct bmi_device *sd,
			struct v4l2_dv_enum_preset *preset);
	int (*s_dv_preset)(struct bmi_device *sd,
			struct v4l2_dv_preset *preset);
	int (*query_dv_preset)(struct bmi_device *sd,
			struct v4l2_dv_preset *preset);
	int (*s_dv_timings)(struct bmi_device *sd,
			struct v4l2_dv_timings *timings);
	int (*g_dv_timings)(struct bmi_device *sd,
			struct v4l2_dv_timings *timings);
	int (*enum_mbus_fmt)(struct bmi_device *sd, unsigned int index,
			     enum v4l2_mbus_pixelcode *code);
	int (*g_mbus_fmt)(struct bmi_device *sd,
			  struct v4l2_mbus_framefmt *fmt);
	int (*try_mbus_fmt)(struct bmi_device *sd,
			    struct v4l2_mbus_framefmt *fmt);
	int (*s_mbus_fmt)(struct bmi_device *sd,
			  struct v4l2_mbus_framefmt *fmt);
};

struct bmi_camera_pad_ops {
	int (*enum_mbus_code)(struct bmi_device *sd, struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_pad_mbus_code_enum *code);
	int (*enum_frame_size)(struct bmi_device *sd,
			       struct v4l2_subdev_fh *fh,
			       struct v4l2_subdev_frame_size_enum *fse);
	int (*enum_frame_interval)(struct bmi_device *sd,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_frame_interval_enum *fie);
	int (*get_fmt)(struct bmi_device *sd, struct v4l2_subdev_fh *fh,
		       unsigned int pad, struct v4l2_mbus_framefmt *fmt,
		       enum v4l2_subdev_format which);
	int (*set_fmt)(struct bmi_device *sd, struct v4l2_subdev_fh *fh,
		       unsigned int pad, struct v4l2_mbus_framefmt *fmt,
		       enum v4l2_subdev_format which);
};

struct bmi_camera_ops {
	const struct bmi_camera_core_ops	*core;
	const struct bmi_camera_video_ops	*video;
	const struct bmi_camera_pad_ops	        *pad;
};


#endif
