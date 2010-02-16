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
 *      Part of BMI LCD Kernel Module
 *
 *-----------------------------------------------------------------------------
 */

#ifndef LCD_CTL_H
#define LCD_CTL_H

#include <linux/kernel.h>

#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/bmi/bmi_ioctl.h>

	// IOCTL commands for BMI LCD driver
#define BMI_LCD_RLEDOFF		_IOW(BMI_LCD_IOCTL, 0x1, __u32)		// turn off Red LED
#define BMI_LCD_RLEDON		_IOW(BMI_LCD_IOCTL, 0x2, __u32)		// turn on Red LED
#define BMI_LCD_GLEDOFF		_IOW(BMI_LCD_IOCTL, 0x3, __u32)		// turn off Green LED
#define BMI_LCD_GLEDON		_IOW(BMI_LCD_IOCTL, 0x4, __u32)		// turn on Green LED
#define BMI_LCD_VSYNC_DIS	_IOW(BMI_LCD_IOCTL, 0x5, __u32)		// Enable VSYNC output buffer
#define BMI_LCD_VSYNC_EN	_IOW(BMI_LCD_IOCTL, 0x6, __u32)		// Disable VSYNC output buffer
#define BMI_LCD_EN		_IOW(BMI_LCD_IOCTL, 0x7, __u32)		// Enable LCD component
#define BMI_LCD_DIS		_IOW(BMI_LCD_IOCTL, 0x8, __u32)		// Disable LCD component
#define BMI_LCD_SER_EN		_IOW(BMI_LCD_IOCTL, 0x9, __u32)		// Enable Seriallizer component
#define BMI_LCD_SER_DIS		_IOW(BMI_LCD_IOCTL, 0xa, __u32)		// Disable Seriallizer component
#define BMI_LCD_SETRST		_IOW(BMI_LCD_IOCTL, 0xb, __u32)		// Disable entire module
#define BMI_LCD_CLRRST		_IOW(BMI_LCD_IOCTL, 0xc, __u32)		// Enable entire module
#define BMI_LCD_SET_BL		_IOW(BMI_LCD_IOCTL, 0xd, __u32)		// Set IOX backlight bits [2:0]
#define BMI_LCD_GETSTAT		_IOR(BMI_LCD_IOCTL, 0xe, __u32)		// Get IOX state
#define BMI_LCD_ACTIVATE	_IOW(BMI_LCD_IOCTL, 0xf, __u32)		// Activate SER, TS, ACCEL
#define BMI_LCD_DEACTIVATE	_IOW(BMI_LCD_IOCTL, 0x10, __u32)	// Deactivate SER, TS, ACCEL
#define BMI_LCD_SUSPEND		_IOW(BMI_LCD_IOCTL, 0x11, __u32)	// Power down module
#define BMI_LCD_RESUME		_IOW(BMI_LCD_IOCTL, 0x12, __u32)	// Power up module

/*
 * 	I2C set up
 */

	// I2C Slave Address
#define BMI_IOX_I2C_ADDRESS	0x71	// 7-bit address
#define BMI_ACC_I2C_ADDRESS	0x17	// 7-bit address

	// I2C IOX register addresses
#define IOX_INPUT_REG		0x0	// IOX input data register
#define IOX_OUTPUT_REG		0x1	// IOX output data register
#define IOX_POLARITY_REG	0x2	// IOX polarity data register
#define IOX_CONTROL		0x3	// IOX direction control register
#define IOX_B1			(0)	// bit 0 - backlight control
#define IOX_A1_A2		(1)	// bit 1 - backlight control
#define IOX_ACC_RST_N		(2)	// bit 2 - acceleromter reset
#define IOX_VSYNC_EN_N		(3)	// bit 3 - VSYNC output buffer enable
#define IOX_LCD_RST_N		(4)	// bit 4 - LCD reset
#define IOX_SERDES_PD_N		(5)	// bit 5 - SERDES power down
#define IOX_X_INT		(6)	// bit 6 - accelerometer interrupt
#define IOX_Y_INT		(7)	// bit 7 - accelerometer interrupt

struct lcd_ctl
{
  int slot;
  struct cdev cdev;
  struct device *class_dev;
};

extern int  lcd_ctl_init (void);
extern void lcd_ctl_clean(void);
extern int  lcd_ctl_probe (struct lcd_ctl *lcd_ctl, int slot); 
extern void lcd_ctl_remove(struct lcd_ctl *lcd_ctl, int slot); 


#endif


