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

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "mt9t111.h"
#include "mt9t111_reg.h"

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
		printk(KERN_DEBUG "%s: Read register 0x%x. value = 0x%x\n", __func__, reg, *val);
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
		printk(KERN_INFO "%s error writing to addr=0x%x data=0x%x err=%d\n", __func__, reg, val, err);
	else
		printk(KERN_DEBUG "%s succeed writing to addr=0x%x data=0x%x err=%d\n", __func__, reg, val, err);


	return err;
}
EXPORT_SYMBOL(mt9t111_write_reg);


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
	struct mt9t111_regs *reg = r;
	
	for (i=0;i<cnt;i++) {
		if (reg->delay_time == 0) {
			err = mt9t111_write_reg(client, reg->addr, reg->data);
		} else if (reg->addr != 0 || reg->data != 0) {
			err = mt9t111_write_reg(client, reg->addr, reg->data);
			mdelay(reg->delay_time);
		} else 
			mdelay(reg->delay_time);
			
		if (err < 0)
			return err;
		reg++;
	}
	return err;
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

static void mt9t111_refresh(struct i2c_client *client){	
	int i, err;	
	unsigned short value;		
	// MCU_ADDRESS [SEQ_CMD] -- refresh	
	mt9t111_write_reg(client, 0x098E, 0x8400);	
	mt9t111_write_reg(client, 0x0990, 0x0006);	
	for (i=0;i<100;i++){
		err = mt9t111_write_reg(client, 0x098E, 0x8400);
		if(err < 0) {
			printk(KERN_INFO "%s write failed %d\n", __func__, i);
			continue;
		}
		err = mt9t111_read_reg(client,  0x0990, &value);
		if(err < 0) {
			printk(KERN_INFO "%s read failed %d\n", __func__, i);
			continue;
		}
		if (value == 0)			
			break;		
		mdelay(5);	
	}
//	mt9t111_write_reg(client, 0x098E, 0x8400);	
//	mt9t111_write_reg(client, 0x0990, 0x0002);	
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
	while(0) {
		err = mt9t111_read_reg(client,0x0014,&value);
		if(err < 0) {
			printk(KERN_INFO "%s: error readign pll lock state\n", __func__);
			return err;
		}
		if (( value & 0x8000) != 0)
			break;
		if(i++ > 100) {
			printk(KERN_INFO "%s: can't get pll lock\n", __func__);
			return -EBUSY;
		}
		mdelay(2);
		printk(KERN_INFO "%s waiting for pll lock %d\n", __func__, i);
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
	int err;
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

	err = MT9T111_APPLY_PATCH(client, bayer_pattern_regs);
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

int mt9t111_set_format(struct i2c_client *client, int *cols, int *rows)
{
	int ret;
	ret = mt9t111_write_reg(client, 0x6800, *cols);
	if(ret < 0) 
		return ret;
	ret = mt9t111_write_reg(client, 0x6802, *rows);
	if(ret < 0) 
		return ret;
	return 0;
}
EXPORT_SYMBOL(mt9t111_set_format);




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
		printk(KERN_INFO "%s: failed registering " MT9T111_NAME "\n",
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
