#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/sc16is7x.h>


#include <asm/gpio.h>

#define SC16IS7X_DIRECTION 0xA
#define SC16IS7X_STATE 0XB

struct sc16is7x_chip {
  unsigned base;
  unsigned char reg_state;
  unsigned char reg_direction;
  
  struct spi_device *spi_dev;
  struct gpio_chip gpio_chip;
  char **names;
};

static int sc16is7x_write_reg(struct sc16is7x_chip *chip, unsigned char address, unsigned char data)
{
  int ret;
  unsigned char buf[2];

  buf[0] = address << 3;
  buf[1] = data;
  ret = spi_write_then_read(chip->spi_dev,buf,2,NULL,0);
  
  if (ret < 0) {
    dev_err(&chip->spi_dev->dev, "failed writing register\n");
    return ret;
  }
  return 0;
}

static int sc16is7x_read_reg(struct sc16is7x_chip *chip, unsigned char address, unsigned char *data)
{
  int ret;

  *data = 0x80 | (address << 3);
  ret = spi_write_then_read(chip->spi_dev,data,1,data,1);

  if (ret < 0) {
    dev_err(&chip->spi_dev->dev, "failed reading register\n");
    return ret;
  }
  return 0;
}

static void sc16is7x_dump_regs(struct sc16is7x_chip *chip)
{
  unsigned char tmp;
  int i;
  int res;

  for (i =0; i <= 0xf; i++)
    {
      res = sc16is7x_read_reg(chip, i, &tmp);
      if (res != 0) {
	printk(KERN_ERR "%d\n",res);
	return;
      }
      printk(KERN_INFO "0x%x : 0x%02x \n", i, tmp);
    }
  return;
}

static int sc16is7x_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
  struct sc16is7x_chip *chip;
  uint8_t reg_val;
  int ret;

  chip = container_of(gc, struct sc16is7x_chip, gpio_chip);

  reg_val = chip->reg_direction & ~(1u << off);
  ret = sc16is7x_write_reg(chip, SC16IS7X_DIRECTION, reg_val);
  if (ret)
    return ret;

  chip->reg_direction = reg_val;
  return 0;
}

static int sc16is7x_gpio_direction_output(struct gpio_chip *gc, unsigned off, int val)
{
  struct sc16is7x_chip *chip;
  uint8_t reg_val;
  int ret;

  chip = container_of(gc, struct sc16is7x_chip, gpio_chip);


  /* Set direction direction */
  reg_val = chip->reg_direction | (1u << off);
  ret = sc16is7x_write_reg(chip, SC16IS7X_DIRECTION, reg_val);
  if (ret)
    return ret;

  chip->reg_direction = reg_val;

  /* set output level */
  if (val)
    reg_val = chip->reg_state | (1u << off);
  else
    reg_val = chip->reg_state & ~(1u << off);

  ret = sc16is7x_write_reg(chip, SC16IS7X_STATE, reg_val);
  if (ret)
    return ret;

  chip->reg_state = reg_val;

  return 0;
}

static int sc16is7x_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
  struct sc16is7x_chip *chip;
  uint8_t reg_val;
  int ret;

  chip = container_of(gc, struct sc16is7x_chip, gpio_chip);

  ret = sc16is7x_read_reg(chip, SC16IS7X_STATE, &reg_val);
  if (ret < 0)
    return ret;

  return (reg_val & (1u << off)) ? 1 : 0;
}

static void sc16is7x_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
  struct sc16is7x_chip *chip;
  uint8_t reg_val;
  int ret;

  chip = container_of(gc, struct sc16is7x_chip, gpio_chip);

  if (val)
    reg_val = chip->reg_state | (1u << off);
  else
    reg_val = chip->reg_state & ~(1u << off);

  ret = sc16is7x_write_reg(chip, SC16IS7X_STATE, reg_val);
  if (ret)
    return;

  chip->reg_state = reg_val;
}

static void sc16is7x_setup_gpio(struct sc16is7x_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = sc16is7x_gpio_direction_input;
	gc->direction_output = sc16is7x_gpio_direction_output;
	gc->get = sc16is7x_gpio_get_value;
	gc->set = sc16is7x_gpio_set_value;
	gc->can_sleep = 1;

	gc->base = chip->base;
	gc->ngpio = gpios;
	gc->label = dev_name(&chip->spi_dev->dev);
	gc->dev = &chip->spi_dev->dev;
	gc->owner = THIS_MODULE;
	gc->names = chip->names;
}

static int __devinit sc16is7x_probe(struct spi_device *spi)
{
  struct sc16is7x_platform_data *pdata;
  struct sc16is7x_chip *chip;
  int res = 0;
  
  spi->mode = SPI_MODE_0;
  spi->bits_per_word = 8;
  spi_setup(spi);

  printk(KERN_INFO "SC16IS7X: Probe Called...\n");

  pdata = spi->dev.platform_data;
  if (pdata == NULL) { 
    dev_err(&spi->dev, "No platform data...\n");
    return -EINVAL;
  }

  chip = kzalloc(sizeof(struct sc16is7x_chip), GFP_KERNEL);

  if (!chip)
    return -ENOMEM;
  spi_set_drvdata(spi, chip);

  chip->spi_dev = spi;
  chip->base = pdata->gpio_base;
  chip->names = pdata->names;

  sc16is7x_setup_gpio(chip,8);

  res = sc16is7x_read_reg(chip, SC16IS7X_STATE, &chip->reg_state);
  if (res)
    goto failed;
  
  res = sc16is7x_read_reg(chip, SC16IS7X_DIRECTION, &chip->reg_direction);
  if (res)
    goto failed;

  res = gpiochip_add(&chip->gpio_chip);
  if (res) {
    dev_err(&chip->spi_dev->dev, "failed adding gpios\n");
    goto failed;
  }
  if (pdata->setup) {
    res = pdata->setup(spi,chip->gpio_chip.base, chip->gpio_chip.ngpio, pdata->context);
  }
  return 0;

 failed:
  kfree(chip);
  return res;
}

static int __devexit sc16is7x_remove(struct spi_device *spi)
{
  return 0;
}

static struct spi_driver sc16is7x_driver = {
	.driver = {
		.name	 = "sc16is7x",
		.owner	= THIS_MODULE,
	},
	.probe	 = sc16is7x_probe,
	.remove = __devexit_p(sc16is7x_remove),
};

static __init int sc16is7x_init(void)
{
	return spi_register_driver(&sc16is7x_driver);
}
module_init(sc16is7x_init);

static __exit void sc16is7x_exit(void)
{
	spi_unregister_driver(&sc16is7x_driver);
}
module_exit(sc16is7x_exit);

MODULE_DESCRIPTION("SC16IS7X SPI driver");
MODULE_AUTHOR("Matt Isaacs <izzy@buglabs.net>");
MODULE_LICENSE("GPL");
