/*
 * File:         include/linux/bmi/bmi_camera.h
 * Author:       Peter Giacomini <p.giacomini@encadis.com>
 *
 * 		This is the application header file for the BMI bus camera plug-in
 * 		module on the MX31 BUG platform.
 */

#ifndef BMI_CAMERA_A_H
#define BMI_CAMERA_A_H

#include <linux/input.h>
#include <linux/bmi/bmi_ioctl.h>

	// IOCTL commands for BMI Camera driver

#define BMI_CAM_FLASH_HIGH_BEAM  _IOW(BMI_CAMERA_IOCTL, 0x1, __u32)
#define BMI_CAM_FLASH_LOW_BEAM   _IOW(BMI_CAMERA_IOCTL, 0x2, __u32)
#define BMI_CAM_FLASH_LED_OFF    _IOW(BMI_CAMERA_IOCTL, 0x3, __u32)
#define BMI_CAM_FLASH_LED_ON     _IOW(BMI_CAMERA_IOCTL, 0x4, __u32)

#define BMI_CAM_RED_LED_OFF      _IOW(BMI_CAMERA_IOCTL, 0x5, __u32)		// Turn off red LED
#define BMI_CAM_RED_LED_ON       _IOW(BMI_CAMERA_IOCTL, 0x6, __u32)		// Turn on red LED
#define BMI_CAM_GREEN_LED_OFF    _IOW(BMI_CAMERA_IOCTL, 0x7, __u32)		// Turn off green LED
#define BMI_CAM_GREEN_LED_ON     _IOW(BMI_CAMERA_IOCTL, 0x8, __u32)		// Turn on green LED

#define BMI_CAM_SELECT           _IOW(BMI_CAMERA_IOCTL, 0x9, __u32)		// Select camera module
#define BMI_CAM_GET_SELECTED     _IOR(BMI_CAMERA_IOCTL, 0xA, __u32)		// return selected camera module
#define BMI_CAM_SUSPEND          _IOW(BMI_CAMERA_IOCTL, 0xB, __u32)
#define BMI_CAM_RESUME           _IOW(BMI_CAMERA_IOCTL, 0xC, __u32)

	// input event definitions
#define BN_SHUTTER	BTN_0
#define BN_ZOOMIN	BTN_1
#define BN_ZOOMOUT	BTN_2

#endif	/* BMI_CAMERA_A_H */

