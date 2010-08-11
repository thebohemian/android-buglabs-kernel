/*
 * drivers/bmi/pims/camera/mt9t111.c
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

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "mt9t111.h"
#include "mt9t111_reg.h"

struct mt9t111_sensor {
	struct i2c_client *client;
	struct v4l2_mbus_framefmt format[2];
	u8 active_context;
	u8 test_pat_id;
	u8 colorfx_id;
};

int mt9t111_read_reg(struct i2c_client *client, u16 reg, u16 *val)
{
	int ret;
	u8 data[2];
	data[0] = (reg >> 8) & 0x00FF;
	data[1] = (reg >> 0) & 0x00FF;
	ret = i2c_master_send(client, data, 2); // send register addr first
	if (ret >= 0)
		ret = i2c_master_recv(client, data, 2); // read register value
	if (ret < 0) {
		printk(KERN_ERR "%s: i2c_transfer() failed...%d\n", __func__, ret);
	} else {
		*val = ((data[0] & 0x00ff) << 8) | (data[1] & 0x00ff);	
		//printk(KERN_DEBUG "%s: Read register 0x%x. value = 0x%x\n", __func__, reg, *val);
	}
	return ret;
}
EXPORT_SYMBOL(mt9t111_read_reg);

/**
 * mt9t111_write_reg - Write a value to a register in an mt9t111 sensor device
 * @client: i2c driver client structure
 * @data_length: length of data to be read
 * @reg: register address / offset
 * @val: value to be written to specified register
 *
 * Write a value to a register in an mt9t111 sensor device.
 * Returns zero if successful, or non-zero otherwise.
 */
int mt9t111_write_reg(struct i2c_client *client, u16 reg, u16 val)
{
	struct i2c_msg msg[1];
	u8 data[20];
	int err;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 4;
	msg->buf = data;
	data[0] = (u8)((reg & 0xff00) >> 8);
	data[1] = (u8)(reg & 0x00ff);	
	data[2] = (u8)((val & 0xff00) >> 8);
	data[3] = (u8)(val & 0x00ff);
	err = i2c_transfer(client->adapter, msg, 1);
	if(err < 0)
		printk(KERN_ERR "%s error writing to addr=0x%x data=0x%x err=%d\n", __func__, reg, val, err);
	//else
		//printk(KERN_DEBUG "%s succeed writing to addr=0x%x data=0x%x err=%d\n", __func__, reg, val, err);


	return err;
}
EXPORT_SYMBOL(mt9t111_write_reg);


int mt9t111_write_bits(struct i2c_client *client, u16 reg, u16 val, u16 mask)
{
	u16 tmp;
	int err = mt9t111_read_reg(client, reg, &tmp);
	if (err < 0)
		return err;
	tmp = (tmp & ~mask) | (val & mask);
	return mt9t111_write_reg(client, reg, tmp);
}

/**
 * mt9t111_write_regs - Write registers to an mt9t111 sensor device
 * @client: i2c driver client structure
 * @reg_in: pointer to registers to write
 * @cnt: the number of registers 
 *
 * Write registers .
 * Returns zero if successful, or non-zero otherwise.
 */
static int
mt9t111_write_regs(struct i2c_client *client, struct mt9t111_regs *r, int cnt)
{
	int err = 0, i;
	u16 val;
	struct mt9t111_regs *reg = r;
	
	for (i=0;i<cnt;i++) {
		if(reg->mask != 0x0000 && reg->mask != 0xFFFF) { 
			// in this case, the user is applying a write mask,
			// so we first read the register in question to 
			// generate the correct data to write based on the mask.
			err = mt9t111_read_reg(client, reg->addr, &val);
			if (err < 0)
				return err;
			val = (val & ~reg->mask) | (reg->data & reg->mask);
		} else {
			val = reg->data;
		}
			
		if (reg->delay_time == 0) {
			err = mt9t111_write_reg(client, reg->addr, val);
		} else if (reg->addr != 0 || reg->data != 0) {
			err = mt9t111_write_reg(client, reg->addr, val);
			mdelay(reg->delay_time);
		} else 
			mdelay(reg->delay_time);
			
		if (err < 0)
			return err;
		reg++;
	}
	return err;
}

/* The mt9t111_write_var() and mt9t111_read_var() functions are
 * convenience methods for writing to the extended variable registers
 * on the mt9t111. */
static int mt9t111_write_var(struct i2c_client *client, u16 addr, u16 val) 
{
	int ret;
	ret = mt9t111_write_reg(client, 0x098E, addr);
	if(ret < 0)
		return ret;
	return mt9t111_write_reg(client, 0x0990, val);
}

static int mt9t111_read_var(struct i2c_client *client, u16 addr, u16 *val)
{
	int ret;
	ret = mt9t111_write_reg(client, 0x098E, addr);
	if(ret < 0)
		return ret;
	return mt9t111_read_reg(client, 0x0990, val);
}

static int mt9t111_detect(struct i2c_client *client) 
{
	u16 val;
	/* chip ID is at address 0 */
	if (mt9t111_read_reg(client, MT9T111_CHIP_ID, &val) < 0) {
		printk(KERN_ERR "%s: Error reading MT9T111 chip id.\n", __func__);
		return -ENODEV;
	}
	if (val != MT9T111_CHIP_ID_VALUE) {
		printk(KERN_ERR "%s: Chip ID mismatch received 0x%x expecting 0x%x\n",
		       __func__, val, MT9T111_CHIP_ID_VALUE);
		return -ENODEV;
	}
	printk(KERN_INFO "%s: Read MT9T111 CHIP ID = 0x%x", __func__, val);
	return 0;
}

#define MT9T111_APPLY_PATCH(client, x) mt9t111_write_regs(client, x, sizeof(x)/sizeof(x[0]));

static int mt9t111_refresh(struct i2c_client *client){	
	int i, err;	
	unsigned short value;		
	err = mt9t111_write_var(client, 0x8400, 0x0006);//Refresh Sequencer Mode
	if(err < 0)
		return err;
	//err = mt9t111_write_var(client, 0x8400, 0x0005);//Refresh Mode
	//if(err < 0)
	//	return err;

	for (i=0;i<100;i++){
		err = mt9t111_read_var(client, 0x8400, &value);
		if(err < 0)
			return err;
		if (value == 0)			
			break;		
		mdelay(5);	
	}
	return 0;
}

static int mt9t111_enable_pll(struct i2c_client *client)
{
	int i, err;
	unsigned short value; 

	err = MT9T111_APPLY_PATCH(client, pll_regs1);
	if(err < 0) {
		printk(KERN_ERR "%s error applying pll_regs1 patch (err=%d)\n", __func__, err);
		return err;
	}
	i=0;
	while(1) { // wait for MT9T111 to report that PLL is locked
		err = mt9t111_read_reg(client,0x0014,&value);
		if(err < 0) {
			printk(KERN_ERR "%s: error reading pll lock state\n", __func__);
			return err;
		}
		if (( value & 0x8000) != 0) {
			printk(KERN_INFO "%s: PLL locked\n", __func__);
			break;
		}
		if(i++ > 100) {
			printk(KERN_ERR "%s: can't get pll lock\n", __func__);
			return -EBUSY;
		}
		mdelay(2);
	}
	err = MT9T111_APPLY_PATCH(client, pll_regs2);
	if(err < 0) {
		printk(KERN_ERR "%s: error applying pll_regs2 patch (err=%d)\n", __func__, err);
		return err;
	}
	return 0;
}

static int mt9t111_sw_reset(struct i2c_client *client)
{
	int err;
	u16 value;
	err = mt9t111_read_reg(client, 0x001A, &value);
	if(err < 0)
		return err;
	err = mt9t111_write_reg(client, 0x001A, value | 0x1);
	if(err < 0)
		return err;
	err = mt9t111_write_reg(client, 0x001A, value & ~0x1);
	if(err < 0)
		return err;
	return 0;
}

#if 0
static int mt9t111_soft_standby(struct i2c_client *client, int on)
{
	int err,i;
	unsigned short value;

	err = mt9t111_read_reg(client, 0x0018, &value);
	if(on) 
		value |= 0x1;
	else
		value &= ~0x1;

	err = mt9t111_write_reg(client, 0x0018, value);
	
	// now wait until the standby_done state indicator switches
	i=0;
	while(1) {
		err = mt9t111_read_reg(client, 0x0018, &value);
		if(err < 0)
			return err;
		if ( (value & 0x4000) != (on) ? 0x4000 : 0x0000 ) 
			break;
		if(i++ > 100) {
			return -EBUSY;
		}
		mdelay(2);
	}
	return 0;
}
#endif

static int mt9t111_loaddefault(struct i2c_client *client)
{
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	int err;
	sensor->active_context = 0;
	sensor->test_pat_id = 0;
	sensor->colorfx_id = 0;

	err = mt9t111_enable_pll(client);
	if(err < 0) 
		return err;

	err = MT9T111_APPLY_PATCH(client, def_regs1);
	if(err < 0)
		return err;

	err = MT9T111_APPLY_PATCH(client, patch_rev6);
	if(err < 0)
		return err;

	err = MT9T111_APPLY_PATCH(client, def_regs2);
	if(err < 0)
		return err;

	err = MT9T111_APPLY_PATCH(client, fmt_GBRG_regs);
	if(err < 0)
		return err;
	//mt9t111_color_bar(client);

	mt9t111_refresh(client);

	return 0;
}

int mt9t111_set_power(struct i2c_client *client, int on)
{
	int ret = 0;
	if(on) {
		ret = mt9t111_detect(client);
		if(ret < 0)
			return ret;

		ret = mt9t111_sw_reset(client);
		if(ret < 0)
			return ret;
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_set_power);

int mt9t111_s_stream(struct i2c_client *client, int streaming)
{
	int ret;
	if(streaming) {
		ret = mt9t111_loaddefault(client);
		if(ret < 0)
			return ret;
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_s_stream);

int mt9t111_set_format(struct i2c_client *client, struct v4l2_mbus_framefmt *fmt)
{
	int ret, timeout=0;
	u16 val;
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);

	switch(fmt->code) {
	//case V4L2_MBUS_FMT_JPEG8:
	//	printk(KERN_INFO "%s applying JPEG mode\n", __func__);
	//	ret = MT9T111_APPLY_PATCH(client, fmt_JPEG_regs);
	//	if(ret < 0)
	//		return ret;
	//	break;
	case V4L2_MBUS_FMT_YUYV8_2X8_LE:
	case V4L2_MBUS_FMT_YVYU8_2X8_LE:
	case V4L2_MBUS_FMT_YUYV8_2X8_BE:
	case V4L2_MBUS_FMT_YVYU8_2X8_BE:
	case V4L2_MBUS_FMT_YUYV16_1X16:
	case V4L2_MBUS_FMT_UYVY16_1X16:
	case V4L2_MBUS_FMT_YVYU16_1X16:
	case V4L2_MBUS_FMT_VYUY16_1X16:
		printk(KERN_INFO "%s applying YUV mode\n", __func__);
		ret = MT9T111_APPLY_PATCH(client, fmt_YCrCb_regs);
		if(ret < 0)
			return ret;
		break;
	default:
		printk(KERN_INFO "%s applying GBRG mode\n", __func__);
		ret = MT9T111_APPLY_PATCH(client, fmt_GBRG_regs);
		if(ret < 0)
			return ret;
		fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;
		break;
	}

//	if(fmt->height <= 768 && fmt->width <= 1024) {
//		sensor->active_context = 1;
//	} else {
//		sensor->active_context = 1;
//	}
	sensor->active_context = 0;

	while(1) {
		
		mt9t111_write_var(client, 0x8400, sensor->active_context ? 2:1);
		ret = mt9t111_read_var(client, 0x8401, &val);
		printk(KERN_INFO "%s Context status register = %d\n", __func__, val);
		if(val == (sensor->active_context ? 7 : 3)) {
			break;
		}
		if(timeout == 100) {
			printk(KERN_ERR "%s context switch failed\n", __func__);
			ret = -EBUSY;
			break;
		}
		timeout++;
		mdelay(50);
	}

	memcpy(&(sensor->format[sensor->active_context]), fmt, sizeof(*fmt));
	return mt9t111_refresh(client);
}
EXPORT_SYMBOL(mt9t111_set_format);


int mt9t111_get_format(struct i2c_client *client, struct v4l2_mbus_framefmt *fmt)
{
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	struct v4l2_mbus_framefmt *tmp = &(sensor->format[sensor->active_context]);
	memcpy(fmt, tmp, sizeof(*fmt));
	return 0;
}
EXPORT_SYMBOL(mt9t111_get_format);

static char *test_pats[] = {
	"Disabled",         
	"Walking 1's",      
	"Solid White",      
	"Grey Ramp",        
	"Color Bars",       
	"Black/White Bars", 
	"Pseudo Random",    
};
static char *fx[] = {
	"Disabled",         
	"Black & White",
	"Sepia",
	"Negative",
	"Solarize",
};

static int mt9t111_set_test_pattern(struct i2c_client *client, int id) {
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	int err = 0;
	sensor->test_pat_id = id;
	if(id == 0) { // disable test pattern
		printk(KERN_INFO "%s Disabling Test Pattern\n", __func__);
		err |= mt9t111_write_var(client, 0x6025, 0x0000); //select pat 0
		err |= mt9t111_write_var(client, 0x6003, 0x0000); //disable patt
                //enable lens correction, gamma, etc.
		err |= mt9t111_write_reg(client, 0x3210, 0x01B8); 

		// disable 8bit walking 1's test pattern
		err |= mt9t111_write_bits(client, 0x3C20, 0x00, 0x0030);

	} else if(id == 1) { // walking 1's test pattern

		printk(KERN_INFO "%s Enabling Walking 1's Test Pattern\n", __func__);
		err |= mt9t111_write_var(client, 0x6025, 0x0000); //select pat 0
		err |= mt9t111_write_var(client, 0x6003, 0x0000); //disable patt

		// enable 8bit walking 1's test pattern
		err |= mt9t111_write_bits(client, 0x3C20, 0x20, 0x0032);
		
		// Note the user must be in Bayer mode for this to work.

	} else { // all other test patterns
		int code;
		switch(id) {
		case 2: code = 1; break; // solid white
		case 3: code = 4; break; // gray ramp
		case 4: code = 6; break; // color bars
		case 5: code = 8; break; // black/white bars
		case 6: code = 10; break; // random
		default: code = 6; break;
		}
		printk(KERN_INFO "%s Enabling Test Pattern %d\n", __func__, code);
		// disable 8bit walking 1's test pattern
		err |= mt9t111_write_bits(client, 0x3C20, 0x00, 0x0030);

		err |= mt9t111_write_var(client, 0x6003, 0x100); //enable patt.
		err |= mt9t111_write_var(client, 0xE025, code);  //select patt.
		{
			u16 val;
			mt9t111_read_var(client, 0xE025, &val);
			printk(KERN_INFO "%s test pat code=0x%x\n", __func__, val);
		}
		// disable lens correction, gamma, etc
		err |= mt9t111_write_reg(client, 0x3210, 0x0000);
	}
	mt9t111_refresh(client);
	return err;
}

static int mt9t111_set_colorfx(struct i2c_client *client, int fx) {
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	printk(KERN_INFO "%s fx=%d\n", __func__, fx);
	switch (fx) {
	case V4L2_COLORFX_NONE:
		mt9t111_write_var(client, 0xE883, 0x0000);
		mt9t111_write_var(client, 0xEC83, 0x0000);
		break;
	case V4L2_COLORFX_BW:
		mt9t111_write_var(client, 0xE883, 0x0001);
		//mt9t111_write_var(client, 0xEC83, 0x0001);
		break;
	case V4L2_COLORFX_SEPIA:
		mt9t111_write_var(client, 0xE883, 0x0002);
		mt9t111_write_var(client, 0xEC83, 0x0002);
		break;
 	case V4L2_COLORFX_NEGATIVE:
		mt9t111_write_var(client, 0xE883, 0x0003);
		mt9t111_write_var(client, 0xEC83, 0x0003);
		break;
	case 4:
		//[Special Effect – Solarize w/ Strength Control]
		mt9t111_write_var(client, 0xE883, 0x0004);
		//mt9t111_write_var(client, 0xE884, 0x08);// SOLARIZATION_TH
		mt9t111_write_var(client, 0xEC83, 0x0004);
		//mt9t111_write_var(client, 0xEC84, 0x08);// SOLARIZATION_TH
		break;
	default:
		return -EINVAL;
	}
	sensor->colorfx_id = fx;
	return mt9t111_refresh(client);
}

int mt9t111_query_ctrl(struct i2c_client *client, struct v4l2_queryctrl *a)
{
	switch (a->id) {
	case V4L2_CID_TEST_PATTERN:
		a->type = V4L2_CTRL_TYPE_MENU;
		sprintf(a->name, "Test Pattern");
		a->minimum = 0;
		a->maximum = ARRAY_SIZE(test_pats)-1;
		a->default_value = 0;
		a->flags = 0;
		break;
	case V4L2_CID_COLORFX:
		a->type = V4L2_CTRL_TYPE_MENU;
		sprintf(a->name, "Color Effects");
		a->minimum = 0;
		a->maximum = 4;
		a->default_value = 0;
		a->flags = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_query_ctrl);

int mt9t111_query_menu(struct i2c_client *client, struct v4l2_querymenu *qm)
{
	switch (qm->id) {
	case V4L2_CID_TEST_PATTERN:
		if(qm->index < ARRAY_SIZE(test_pats)) {
			strcpy(qm->name, test_pats[qm->index]);
		} else {
			return -EINVAL;
		}
		break;
	case V4L2_CID_COLORFX:
		if(qm->index < 10) {
			strcpy(qm->name, fx[qm->index]);
		} else {
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_query_menu);

int mt9t111_get_ctrl(struct i2c_client *client, struct v4l2_control *vc)
{
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	switch (vc->id) {
	case V4L2_CID_TEST_PATTERN:
		vc->value = sensor->test_pat_id;
		break;
	case V4L2_CID_COLORFX:
		vc->value = sensor->colorfx_id;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_get_ctrl);

int mt9t111_set_ctrl(struct i2c_client *client, struct v4l2_control *vc)
{
	switch (vc->id) {
	case V4L2_CID_TEST_PATTERN:
		return mt9t111_set_test_pattern(client, vc->value);
	case V4L2_CID_COLORFX:
		return mt9t111_set_colorfx(client, vc->value);
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(mt9t111_set_ctrl);



/**
 * mt9t111_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int mt9t111_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct mt9t111_sensor *sensor;
	sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
	     return -1;

	i2c_set_clientdata(client, sensor);
	sensor->client = client;
	sensor->format[0].width        = 640;
	sensor->format[0].height       = 480;
	sensor->format[0].code         = V4L2_MBUS_FMT_YUYV16_1X16;
	sensor->format[0].colorspace   = V4L2_COLORSPACE_SRGB;
	sensor->format[0].field        = V4L2_FIELD_NONE;
	sensor->format[1].width        = 2048;
	sensor->format[1].height       = 1536;
	sensor->format[1].code         = V4L2_MBUS_FMT_YUYV16_1X16;
	sensor->format[1].colorspace   = V4L2_COLORSPACE_SRGB;
	sensor->format[1].field        = V4L2_FIELD_NONE;
	sensor->active_context = 0;
	sensor->test_pat_id = 0;
	sensor->colorfx_id = 0;
	return 0;
}


/**
 * mt9t111_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of mt9t111_probe().
 */
static int __exit mt9t111_remove(struct i2c_client *client)
{
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);
	kfree(sensor);
	return 0;
}


static const struct i2c_device_id mt9t111_id_table[] = {
	{ MT9T111_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9t111_id_table);

static struct i2c_driver mt9t111_i2c_driver = {
	.driver		= {
		.name	= MT9T111_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= mt9t111_probe,
	.remove		= __exit_p(mt9t111_remove),
	.suspend	= NULL,
	.resume		= NULL,
	.id_table	= mt9t111_id_table,
};

/**
 * mt9t111sensor_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init mt9t111_init(void)
{
	int rval;

	rval = i2c_add_driver(&mt9t111_i2c_driver);
	if (rval)
		printk(KERN_ERR "%s: failed registering " MT9T111_NAME "\n",
		       __func__);
	return rval;
}


/**
 * mt9t111sensor_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of mt9t111sensor_init.
 */
static void __exit mt9t111_exit(void)
{
	i2c_del_driver(&mt9t111_i2c_driver);
}

module_init(mt9t111_init);
module_exit(mt9t111_exit);

MODULE_AUTHOR("Lane Brooks <dirjud@gmail.com>");
MODULE_DESCRIPTION("Aptina MT9T111 camera I2C sensor driver");
MODULE_LICENSE("GPL");
