/* platform data for OMAP BMI slots */

struct omap_bmi_platform_data {
  short gpios[4];
  char spi_cs;
  char i2c_bus_no;
};
