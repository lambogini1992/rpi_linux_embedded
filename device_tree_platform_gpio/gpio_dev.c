#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

/*using for platform device*/
#include <linux/of.h>
#include <linux/platform_device.h>

static int gpio_driver_probe(struct platform_device *pdev);
static int gpio_driver_remove(struct platform_device *pdev);

static const struct of_device_id example_gpio_of_match[] = {
	{ .compatible = "test,test-rpi2b-gpio",  },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, flexcan_of_match);

