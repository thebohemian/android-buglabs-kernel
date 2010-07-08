/*
 * A hwmon driver for the Analog Devices ADXL345
 *
 * Copyright (c) 2009 Cyber Switching, Inc.
 * Author: Robert Mehranfar <robertme@earthlink.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon-vid.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/kernel.h>

#define DRV_NAME	"adxl345"
#define DRV_VERSION	"0.1"

#define SIMULATE 1

/*
 * The ADXL345 registers
 */
#define ADXL345_REG_DEV_ID		0x00
#define ADXL345_REG_OFS(nr)		(0x1E + (nr))
#define ADXL345_REG_DATA(nr)		(0x32 + (nr))
#define ADXL345_REG_DATA_FORMAT		0x31

/*
 * See ADXL345 specification
 */
#define OFFSET_SCALE_FACTOR_WHOLE 15
#define OFFSET_SCALE_FACTOR_FRACTIONAL 6

#define MAX_OFFSET 1988
#define MIN_OFFSET -1998

/*
 * Based on 10-bit resolution +/-16 range set in the DATA FORMAT register
 */
#define DATA_SCALE_FACTOR_WHOLE 15
#define DATA_SCALE_FACTOR_FRACTIONAL 6
#define NUMBER_OF_AXES 3

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

/* Position in the sysfs attribute array */
#define X_AXIS_ATTR 0
#define Y_AXIS_ATTR 1
#define Z_AXIS_ATTR 2
#define DATA_ATTR 3

/*
 * Functions declaration
 */
static void adxl345_init_client(struct i2c_client *client);
static struct adxl345_data *adxl345_update_device(struct device *dev);
static void convert_to_fraction(s16 data, u8 whole_scale, u8 fraction_scale,
					int *decimal, int *fraction);

/*
 * Client data (each client gets its own)
 */
struct adxl345_data {
	struct i2c_client	client;
	struct device		*hwmon_dev;
	struct mutex		update_lock;	/* lock on the structure */
	char			valid;		/* 0 until below are valid */
	unsigned long		last_updated;	/* in jiffies */

	/*
	 * ADXL345 Data
	 * In two's complement, data coming in from or going out to user-space
	 * must be converted.
	 */
	u8			offset[NUMBER_OF_AXES];
	u16			data[NUMBER_OF_AXES];
};


/*
 * Converts internal data to a decimal-fraction value.
 * Notes: Must already be converted out of twos complement
 *
 * @param data  Data to be converted
 * @param whole_scale Scale factor for whole number part. (Set to 1 for no scaling).
 * @param fraction_scale Scale factor for fractional number part. (Set to 1 for no scaling).
 * @param decimal Pointer to location to store decimal part.
 * @param fraction Pointer to location to store fractional part.
 */
static void convert_to_fraction(s16 data, u8 whole_scale, u8 fraction_scale,
					int *decimal, int *fraction)
{
	int temp_decimal, temp_fraction, temp;
	int sign = 1;

	/* Scale the decimal and fractional parts */
	temp_decimal = data * whole_scale * 10;
	temp_fraction = data * fraction_scale;

	/* get rid of the sign for negative fractions */
	if (temp_fraction < 0) {
		sign = -1;
		temp_fraction *= sign;
	}

	/* If necessary, carry */
	if (temp_fraction >= 10) {
		/* Add to the decimal part */
		temp_decimal += sign * temp_fraction;

		/* Amount to be subtracted from the fractional part */
		temp = temp_fraction / 10;

		temp_fraction -= temp * 10;
	}

	temp_decimal /= 10;

	/*
	 * If at least 10 still remains in the fractional part, one
	 * last carry
	 */
	if (temp_fraction >= 10) {
		temp_decimal += sign;
		temp_fraction -= 10;
	}

	/* Pass the values up */
	*decimal = temp_decimal;
	*fraction = temp_fraction;
}

/*
 * Called in response to cat ofs(x,y,z) in sysfs
 *
 * @param dev Pointer to device
 * @param attr Pointer to the device attributes
 * @param buf Pointer to string shown in user space
 * @return Number of chars copied to buffer
*/
static ssize_t show_offset(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int index = to_sensor_dev_attr(attr)->index;
	struct adxl345_data *data = adxl345_update_device(dev);
	int decimal, fraction;
	s8 temp_data;

	dev_dbg(dev, "%s\n", __func__);

	/* Convert from 2's complement */
	temp_data = ~(data->offset[index] - 1);

	dev_dbg(dev, "temp_data=%d\n", temp_data);

	convert_to_fraction(temp_data,
			OFFSET_SCALE_FACTOR_WHOLE,
			OFFSET_SCALE_FACTOR_FRACTIONAL,
			&decimal,
			&fraction);

	return snprintf(buf, PAGE_SIZE,
			"%4d.%1d\n", decimal, fraction);
}

/*
 * Called in response to echoing data to ofs(x,y,z) in sysfs
 * Note: Input range is -1998 to 1998 milli-g's
 *
 * @param dev Pointer to device
 * @param attr Pointer to the device attributes
 * @param buf Pointer to string passed in from user space
 * @param count Number of chars passed in from user space..
 * @return Number of chars passed in from user space.
 */
static ssize_t set_offset(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	int index = to_sensor_dev_attr(attr)->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct adxl345_data *data = i2c_get_clientdata(client);
	long val;
	long temp_val;

	dev_dbg(dev, "%s\n", __func__);

	if (!strict_strtol(buf, 10, &val)) {
		dev_dbg(dev, "error in converting string '%s' to long\n",
				buf);
		return 0;
	}

	/* If outside offset range, clip to max or min value */
	if (val < MIN_OFFSET)
		val = MIN_OFFSET;
	else if (val > MAX_OFFSET)
		val = MAX_OFFSET;

	temp_val = (val * 100) %
			((OFFSET_SCALE_FACTOR_WHOLE * 100) +
			(OFFSET_SCALE_FACTOR_FRACTIONAL * 10));
	if (temp_val < 0)
		temp_val *= -1;

	/* Get rid of scale for internal storage */
	val = (val * 100) /
			((OFFSET_SCALE_FACTOR_WHOLE * 100) +
			(OFFSET_SCALE_FACTOR_FRACTIONAL * 10));

	if (temp_val > (OFFSET_SCALE_FACTOR_WHOLE * 100) / 2 && val > 0)
		++val;
	else if (temp_val > (OFFSET_SCALE_FACTOR_WHOLE * 100) / 2 && val < 0)
		--val;

	val = ~val + 1;  /* convert to two's complement */

	mutex_lock(&data->update_lock);

	data->offset[index] = val;

	dev_dbg(dev, "offset[%d]=%d\n", index, data->offset[index]);

	/* Write the avlue to the chip via I2C */
	i2c_smbus_write_byte_data(client,
			ADXL345_REG_OFS(index),
			data->offset[index]);

	mutex_unlock(&data->update_lock);

	return count;
}

/*
 * Called in response to cat data in sysfs
 *
 * @param dev Pointer to device
 * @param attr Pointer to the device attributes
 * @param buf Pointer to string shown in user space
 * @return Number of chars copied to buffer
*/
static ssize_t show_data(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	int i;
	s16 temp_data;
	int decimal[NUMBER_OF_AXES], fraction[NUMBER_OF_AXES];

	struct adxl345_data *data = adxl345_update_device(dev);

	/* Convert x,y,z values */
	for (i = 0; i < NUMBER_OF_AXES; i++) {
		/* Convert from 2's complement */
		temp_data = ~(data->data[i] - 1);

		convert_to_fraction(temp_data,
				DATA_SCALE_FACTOR_WHOLE,
				DATA_SCALE_FACTOR_FRACTIONAL,
				&decimal[i],
				&fraction[i]);
	}

	return snprintf(buf, PAGE_SIZE,
			"x:%5d.%1d y:%5d.%1d z:%5d.%1d\n",
			decimal[X_AXIS], fraction[X_AXIS],
			decimal[Y_AXIS], fraction[Y_AXIS],
			decimal[Z_AXIS], fraction[Z_AXIS]);
}

/*  Attributes of the sysfs entries */
static SENSOR_DEVICE_ATTR(ofsx, S_IWUSR | S_IRUGO,
				show_offset, set_offset, X_AXIS_ATTR);
static SENSOR_DEVICE_ATTR(ofsy, S_IWUSR | S_IRUGO,
				show_offset, set_offset, Y_AXIS_ATTR);
static SENSOR_DEVICE_ATTR(ofsz, S_IWUSR | S_IRUGO,
				show_offset, set_offset, Z_AXIS_ATTR);
static SENSOR_DEVICE_ATTR(data, S_IRUGO,
				show_data, NULL, DATA_ATTR);

static struct attribute *adxl345_attributes[] = {
	&sensor_dev_attr_ofsx.dev_attr.attr,
	&sensor_dev_attr_ofsy.dev_attr.attr,
	&sensor_dev_attr_ofsz.dev_attr.attr,
	&sensor_dev_attr_data.dev_attr.attr,
	NULL
};

static const struct attribute_group adxl345_group = {
	.attrs = adxl345_attributes,
};

/*
 * Initialize the chip
 *
 * @param client Pointer to the client structure
 */
static void adxl345_init_client(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s\n", __func__);

	/*
	 * Set up the device, No self test, Dont care about SPI or
	 * interrupt, 10 bit resoltion, +/- 16g range
	 */
	i2c_smbus_write_byte_data(client, ADXL345_REG_DATA_FORMAT, 0x03);
}

/*
 * Does more than just detection. If detection succeeds, it also
 * registers the new chip.
 *
 * @param adapter Pointer to the adapter
 * @param address I2C address
 * @return Zero, upon success
 */
static int adxl345_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct adxl345_data *data;
	int err = 0;
	u8 dev_id;

	dev_dbg(&client->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	data = kzalloc(sizeof(struct adxl345_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);

	#ifdef SIMULATE
	dev_id = 0xE5; /* Spoof the Analog Devices device ID */
	dev_dbg(&client->dev, "Simulating ADXL345 device\n");
	#else
	dev_id = i2c_smbus_read_byte_data(client, ADXL345_REG_DEV_ID);
	#endif

	/* If the chip is not from Analog Devices, report an error */
	if (dev_id != 0xE5) {
		dev_err(&client->dev, "Unsupported chip (dev_id=0x%02X)\n",
				dev_id);
		err = -EINVAL;
		goto err_free_mem;
	}

	dev_info(&client->dev, "chip found, driver version "
			DRV_VERSION "\n");

	/* We can fill in the remaining client fields */
	mutex_init(&data->update_lock);

	/* Initialize the ADXL345 chip */
	adxl345_init_client(client);

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &adxl345_group);
	if (err)
		goto err_free_mem;

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto err_remove;
	}

	return 0;

err_remove:
	sysfs_remove_group(&client->dev.kobj, &adxl345_group);
err_free_mem:
	kfree(data);

	return err;
}

/*
 * Unregister device, remove the sysfs entries, and detach the client
 * from I2C bus.
 *
 * @param client Pointer to the client structure
 * @return Zero, upon success.
 */
static int adxl345_remove(struct i2c_client *client)
{
	struct adxl345_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &adxl345_group);

	i2c_unregister_device(client);

	kfree(data);

	return 0;
}

/*
 * Gets the data from the chip.
 *
 * @param client Pointer to the device
 * @return Pointer to structure containing the data
 */
static struct adxl345_data *adxl345_update_device(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adxl345_data *data = i2c_get_clientdata(client);
	int i;

	dev_dbg(dev, "%s\n", __func__);

	mutex_lock(&data->update_lock);

	/*
	 * This delay is 500ms, based on a default value of 100 for HZ
	 * in ARM kernels
	 */
	if (time_after(jiffies, data->last_updated + HZ * 50) ||
		!data->valid) {
		for (i = 0; i < NUMBER_OF_AXES; i++) {
			data->offset[i] = i2c_smbus_read_byte_data(client,
						ADXL345_REG_OFS(i));
			dev_dbg(dev, "offset[%d]=%d\n", i, data->offset[i]);
		}

		/*
		 * Concatenate the data from each register pair
		 * Indexing logic is needed as per ADXL345 spec, LSB is first
		 * INDEX(reg)  LSB  MSB
		 *     0        0    1
		 *     1        2    3
		 *     2        4    5
		 */
		for (i = 0; i < NUMBER_OF_AXES; i++) {
			/* Get the MSB, shift by 8, and then get the LSB */
			data->data[i] = i2c_smbus_read_byte_data(client,
						ADXL345_REG_DATA(i*2+1)) << 8;
			data->data[i] |= i2c_smbus_read_byte_data(client,
						ADXL345_REG_DATA(i*2));

			dev_dbg(dev, "data[%d]=%d\n", i, data->data[i]);
		}

		data->last_updated = jiffies;
		data->valid = 1;
	}

	mutex_unlock(&data->update_lock);

	return data;
}

/*
 * Driver data (common to all clients)
 */

static const struct i2c_device_id adxl345_id[] = {
	{ "adxl345", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adxl345_id);

static struct i2c_driver adxl345_driver = {
	.driver = {
		.name	= DRV_NAME,
	},
	.probe		= adxl345_probe,
	.remove		= adxl345_remove,
	.id_table	= adxl345_id,
};

/*
 * Initialize the module
 *
 * @return Zero, upon success
 */
static int __init adxl345_init(void)
{
	return i2c_add_driver(&adxl345_driver);
}

/*
 * Remove the module
 */
static void __exit adxl345_exit(void)
{
	i2c_del_driver(&adxl345_driver);
}

MODULE_AUTHOR("Robert Mehranfar <robertme@earthlink.net>");
MODULE_DESCRIPTION("Analog Devices ADXL345 driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(adxl345_init);
module_exit(adxl345_exit);
