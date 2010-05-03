#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/i2c.h>


static int ths_read(struct i2c_client *client, unsigned char offset, unsigned char data)
{
	int	ret = 0;
	unsigned char msg[2];
	
     	msg[0] = offset;
	msg[1] = data;
	ret = i2c_master_send(client, msg, sizeof(msg));
	
	if (ret < 0)
	  printk (KERN_ERR "ths_read() - i2c_transfer() failed...%d\n",ret);

	return ret;
}

static int ths_write(struct i2c_client *client, unsigned char offset, unsigned char data)
{
	int	ret = 0;
	unsigned char msg[2];
	
     	msg[0] = offset;
	msg[1] = data;
	ret = i2c_master_send(client, msg, sizeof(msg));
	
	if (ret < 0)
	  printk (KERN_ERR "ths_read() - i2c_transfer() failed...%d\n",ret);

	return ret;
}

static int tfp410p_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
  int err = 0;
  u8 value;

  gpio_direction_output(10, 0);
  gpio_set_value (10, 1);
  mdelay (1);
  gpio_set_value (10, 0);
  mdelay (1);
  //  gpio_set_value (10, 1);


  value = 0xbd;
  err |= WriteByte_TFP(client, 0x08, value);
  mdelay (1);
  
  value = 0x98;
  err |= WriteByte_TFP(client, 0x09, value);
  mdelay (1);
  value = 0x30;
  err |= WriteByte_TFP(client, 0x33, value);
  mdelay (1);
  if (err < 0) {
    dev_err(&client->dev, "%s: Error during init\n", __func__);
    return -EINVAL;
  }
  return 0;
}

static int tfp410p_remove(struct i2c_client *client)
{

	return 0;
}

static const struct i2c_device_id tfp410p_id[] = {
	{"tfp410p", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, tfp410p_id);

static struct i2c_driver tfp410p_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tfp410p",
	},
	.probe		= tfp410p_probe,
	.remove		= tfp410p_remove,
	.id_table	= tfp410p_id,
};

static __init int init_tfp410p(void)
{
	return i2c_add_driver(&tfp410p_driver);
}

static __exit void exit_tfp410p(void)
{
	i2c_del_driver(&tfp410p_driver);
}

module_init(init_tfp410p);
module_exit(exit_tfp410p);
