/* platform data for the PCA9539 16-bit I/O expander driver */

struct sc16is7x_platform_data {
	/* number of the first GPIO */
	unsigned	gpio_base;

	/* initial polarity inversion setting */
	uint16_t	invert;

	void		*context;	/* param to setup/teardown */

	int		(*setup)(struct spi_device *spi,
				unsigned gpio, unsigned ngpio,
				void *context);
	int		(*teardown)(struct spi_device *spi,
				unsigned gpio, unsigned ngpio,
				void *context);
	char		**names;
};
