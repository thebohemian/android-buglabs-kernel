#include <linux/device.h>
#include <linux/bmi.h>


#define bmi_config_attr(field, format_string)				\
static ssize_t								\
field##_show(struct device *dev, struct device_attribute *attr, char *buf)				\
{									\
	struct bmi_device *pdev;						\
									\
	pdev = to_bmi_device (dev);					\
	return sprintf (buf, format_string, pdev->field);		\
}

static ssize_t description_show(struct device *dev, struct device_attribute *attr, char *buf)
{									
	struct bmi_device *pdev;					
									
	pdev = to_bmi_device (dev);					
	return sprintf (buf, "%s\n", pdev->ident->description);		
}

bmi_config_attr(vendor, "0x%04x\n");
bmi_config_attr(product, "0x%04x\n");
bmi_config_attr(revision, "0x%04x\n");


struct device_attribute bmi_dev_attrs[] = {
	__ATTR_RO(vendor),
	__ATTR_RO(product),
	__ATTR_RO(revision),
	__ATTR_RO(description),
	__ATTR_NULL,
};
