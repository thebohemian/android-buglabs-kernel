
/*#define DEBUG*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>

#include <mach/display.h>
#include <mach/dma.h>
#include <mach/gpio.h>

static struct omap_video_timings sharp_spi_timings = {
  .x_res = 320,
  .y_res = 240,
  
  .pixel_clock	= 5000,
  .hsw		= 8, //3
  .hfp		= 4, //3
  .hbp		= 4, //7
  
  .vsw		= 2, //0
  .vfp		= 1, //0
  .vbp		= 1, //1
};


u16 panel_init_seq [] = {
	0x0028,
	0x0100,
	0x0106,

	0x002D,
	0x017F,
	0x0106,

	0x0001,
	0x010A,
	0x01EF,

	0x0002,
	0x0103,
	0x0100,

	0x0003,
	0x010A,
	0x010E,

	0x000B,
	0x01DC,
	0x0100,

	0x000C,
	0x0100,
	0x0105,

	0x000D,
	0x0100,
	0x0102,

	0x000E,
	0x012C,
	0x0100,

	0x000F,
	0x0100,
	0x0100,

	0x0016,
	0x019F,
	0x0188,

	0x0017,
	0x0100,
	0x0102,

	0x001E,
	0x0100,
	0x0100,

	0x0028,
	0x0100,
	0x0106,

	0x002C,
	0x01C8,
	0x018C,

	0x002E,
	0x01B9,
	0x0145,

	0x0030,
	0x0100,
	0x0104,

	0x0031,
	0x0104,
	0x0107,

	0x0032,
	0x0100,
	0x0102,

	0x0033,
	0x0101,
	0x0107,

	0x0034,
	0x0105,
	0x0107,

	0x0035,
	0x0100,
	0x0103,

	0x0036,
	0x0103,
	0x0107,

	0x0037,
	0x0107,
	0x0104,

	0x003A,
	0x011F,
	0x0109,

	0x003B,
	0x0109,
	0x010E,


	0x002D,
	0x017F,
	0x0104,	
};

struct sharp_panel_device {
	struct backlight_device *bl_dev;
	int		enabled;
	unsigned int	saved_bklight_level;

	struct spi_device	*spi;
	struct mutex		mutex;
	struct omap_dss_device	*dssdev;
};


static inline void panel_write(struct spi_device *spi,
			       const u8 *buf, int len)
{
	struct spi_transfer t;

	t.bits_per_word = 9;	
	t.tx_buf = buf;
	t.len = len;

	if (spi_write (spi, buf, len) < 0)
	{
		printk ("SPI transfer failed for LCD\n");
		return ;
	}
	return ;

}

static int sharp_spi_panel_probe(struct omap_dss_device *dssdev)
{

	dssdev->panel.config = OMAP_DSS_LCD_TFT|OMAP_DSS_LCD_IVS|OMAP_DSS_LCD_IHS|OMAP_DSS_LCD_IPC;
	dssdev->panel.acb = 0x0;
	dssdev->panel.recommended_bpp = 18;
	dssdev->panel.timings = sharp_spi_timings;

	return 0;
}

static void sharp_spi_panel_remove(struct omap_dss_device *dssdev)
{
}

static int sharp_spi_panel_enable(struct omap_dss_device *dssdev)
{
  int r = 0;
  int i;
  u16 data = 0x00;
  struct lcd_device *md = dev_get_drvdata(dssdev->panel->priv);

  dev_dbg("spi_lcd_panel_enable\n");

  mutex_lock(&md->mutex);

  if (dssdev->platform_enable)
    {
      r = dssdev->platform_enable(dssdev);
      if (r)
	{
	  goto exit;
	}
      md->spi->bits_per_word = 9;
      spi_setup (md->spi);
      //gpio_set_value (93, 1);
      mdelay (1);
      gpio_set_value (90, 0);
      for (i =0; i<81; i++)
	{
	  data = lcd_init_seq[i];
	  lcd_write (md, (u8 *)&data, 2);
	}
      mdelay (1);

    }

  msleep(50); // wait for power up

  if (md->enabled) {
    mutex_unlock(&md->mutex);
    return 0;
  }

  md->enabled = 1;

 exit:
  mutex_unlock(&md->mutex);
  return 0;
}

static void sharp_spi_lcd_disable(struct omap_dss_device *dssdev)
{
	struct sharp_panel_device *md =
		(struct lcd_device *)dssdev->panel->priv;

	DBG("sharp_spi_lcd_disable\n");

	mutex_lock(&md->mutex);

	if (!md->enabled) {
		mutex_unlock(&md->mutex);
		return;
	}

	md->enabled = 0;


	if (dssdev->hw_config.panel_disable)
		dssdev->hw_config.panel_disable(dssdev);

	mutex_unlock(&md->mutex);
}

static int sharp_spi_lcd_init(struct omap_dss_device *dssdev)
{
	struct sharp_panel_device *md =
		(struct lcd_device *)dssdev->panel->priv;

	DBG("sharp_spi_lcd_init\n");

	mutex_init(&md->mutex);
	md->dssdev = dssdev;
#if 0
	md->spi->bits_per_word = 9;
	spi_setup (md->spi);
	//gpio_set_value (93, 1);
	mdelay (1);
	gpio_set_value (90, 0);
	for (i =0; i<81; i++)
	{
		data = lcd_init_seq[i];
		lcd_write (md, (u8 *)&data, 2);
	}
	mdelay (1);
#endif
	return 0;
}

static int sharp_spi_lcd_suspend (struct omap_dss_device *dssdev)
{
	return 0;
}

static int sharp_spi_lcd_resume (struct omap_dss_device *dssdev)
{

	return 0;
}
static int sharp_spi_lcd_run_tests(struct omap_dss_device *dssdev, int test_num)
{
	return 0;
}

static struct omap_dss_driver sharp_spi_lcd_driver = {
	.probe		= sharp_spi_panel_probe,
	.remove		= sharp_spi_panel_remove,

	.enable		= sharp_spi_panel_enable,
	.disable	= sharp_spi_panel_disable,
	.suspend	= sharp_spi_panel_suspend,
	.resume		= sharp_spi_panel_resume,

	.driver         = {
		.name   = "sharp_spi_panel",
		.owner  = THIS_MODULE,
	},  
};

static int spi_lcd_probe(struct spi_device *spi)
{
	struct sharp_panel_device *md;

	DBG("spi_lcd_probe\n");

	md = kzalloc(sizeof(*md), GFP_KERNEL);
	if (md == NULL) {
		dev_err(&spi->dev, "out of memory\n");
		return -ENOMEM;
	}

	md->spi = spi;
	dev_set_drvdata(&spi->dev, md);
	md->panel = spi_lcd_panel;
	spi_lcd_panel.priv = md;

	omap_dss_register_driver(&sharp_panel_driver);

	return 0;
}

static int spi_lcd_remove(struct spi_device *spi)
{
	struct sharp_panel_device *md = dev_get_drvdata(&spi->dev);

	DBG("spi_lcd_remove\n");

	omap_dss_unregister_driver(&sharp_panel_driver);

	kfree(md);

	return 0;
}

static struct spi_driver lcd_spi_driver = {
	.driver = {
		.name	= "spi-lcd",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= spi_lcd_probe,
	.remove	= __devexit_p(spi_lcd_remove),
};

static int __init spi_lcd_init(void)
{
	DBG("spi_lcd_init\n");
	return spi_register_driver(&lcd_spi_driver);
}

static void __exit spi_lcd_exit(void)
{
	DBG("spi_lcd_exit\n");
	spi_unregister_driver(&lcd_spi_driver);
}

module_init(spi_lcd_init);
module_exit(spi_lcd_exit);

MODULE_LICENSE("GPL");
