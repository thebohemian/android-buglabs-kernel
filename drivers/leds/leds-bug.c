/* drivers/leds/leds-bug.c
 * (C) 2010 Bug Labs, Inc.
 *
 * Based on drivers/leds/leds-omap.c by Kyungmin Park<kyungmin.park@samsung.com>
 *
 * BUG - LEDs GPIO driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <mach/led.h>

/* our context */

static void bug_set_led_gpio(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	struct omap_led_config *led_dev;

	led_dev = container_of(led_cdev, struct omap_led_config, cdev);
	gpio_set_value(led_dev->gpio, value);
}

static int bug_led_probe(struct platform_device *dev)
{
	struct omap_led_platform_data *pdata = dev->dev.platform_data;
	struct omap_led_config *leds = pdata->leds;
	int i, ret = 0;

	for (i = 0; ret >= 0 && i < pdata->nr_leds; i++) {
		ret = gpio_request(leds[i].gpio, leds[i].cdev.name);
		if (ret < 0)
			break;
		gpio_direction_output(leds[i].gpio, 0);
		if (!leds[i].cdev.brightness_set)
			leds[i].cdev.brightness_set = bug_set_led_gpio;

		ret = led_classdev_register(&dev->dev, &leds[i].cdev);
	}

	if (ret < 0 && i > 1) {
		for (i = i - 2; i >= 0; i--) {
			led_classdev_unregister(&leds[i].cdev);
			gpio_free(leds[i].gpio);
		}
	}

	return ret;
}

static int bug_led_remove(struct platform_device *dev)
{
	struct omap_led_platform_data *pdata = dev->dev.platform_data;
	struct omap_led_config *leds = pdata->leds;
	int i;

	for (i = 0; i < pdata->nr_leds; i++) {
		led_classdev_unregister(&leds[i].cdev);
		gpio_free(leds[i].gpio);
	}

	return 0;
}

#ifdef CONFIG_PM
static int bug_led_suspend(struct platform_device *dev, pm_message_t state)
{
	struct omap_led_platform_data *pdata = dev->dev.platform_data;
	struct omap_led_config *leds = pdata->leds;
	int i;

	for (i = 0; i < pdata->nr_leds; i++)
		led_classdev_suspend(&leds[i].cdev);

	return 0;
}

static int bug_led_resume(struct platform_device *dev)
{
	struct omap_led_platform_data *pdata = dev->dev.platform_data;
	struct omap_led_config *leds = pdata->leds;
	int i;

	for (i = 0; i < pdata->nr_leds; i++)
		led_classdev_resume(&leds[i].cdev);

	return 0;
}
#else
#define bug_led_suspend	NULL
#define bug_led_resume		NULL
#endif

static struct platform_driver bug_led_driver = {
	.probe		= bug_led_probe,
	.remove		= bug_led_remove,
	.suspend	= bug_led_suspend,
	.resume		= bug_led_resume,
	.driver		= {
		.name		= "bug-led",
		.owner		= THIS_MODULE,
	},
};

static int __init bug_led_init(void)
{
	return platform_driver_register(&bug_led_driver);
}

static void __exit bug_led_exit(void)
{
 	platform_driver_unregister(&bug_led_driver);
}

module_init(bug_led_init);
module_exit(bug_led_exit);

MODULE_AUTHOR("Ken Gilmer (ken@buglabs.net)");
MODULE_DESCRIPTION("BUG LED driver");
MODULE_LICENSE("GPL");
