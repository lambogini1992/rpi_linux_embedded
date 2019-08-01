#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
/*Using for register character device*/
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
/*using for platform device*/
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>



/*Function declared for gpio*/
int set_direction(uint8_t gpio_pin);
int set_pin_level(uint8_t gpio_pin, uint8_t level);
int read_pin_level(uint8_t gpio_pin);

/*Register character device*/
static int dev_open(struct inode *, struct file *);
static int dev_close(struct inode *, struct file *);
static ssize_t dev_read(struct file*, char __user *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char __user *, size_t, loff_t *);
static long dev_ioctl(struct file *, unsigned int, unsigned long);

/*Using for init device type*/
static int device_probe(struct platform_device *pdev);
static int device_remove(struct platform_device *pdev);

static const struct of_device_id test_leds_of_match[] =
{
	{.compatible = "test_leds, test_leds_gpio_bcm", },
	{},
};

MODULE_DEVICE_TABLE(of, test_gpio_of_match);

static struct platform_driver test_leds_pldriver = {
	.probe 		= device_probe,
	.remove		= device_remove,
	.device 	= {
		.name 						= DRIVER_NAME,
		.owner						= THIS_MODULE,
		.of_match_table		= of_match_ptr(test_leds_of_match),
	},
};

static int device_probe(struct platform_device *pdev)
{
	printk("Hello! This is Leds GPIO\n");
	
}

static int device_remove(struct platform_device *pdev)
{
	printk("Goodbye Leds GPIO\n");
}
