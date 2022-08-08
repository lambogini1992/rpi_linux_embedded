#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include "./gpio_reg.h"

#define MAGIC_NO	262

#define OUTPUT_LED	_IO(MAGIC_NO, 0)
#define INPUT_LED	_IO(MAGIC_NO, 1)
#define BLINK_LED   _IO(MAGIC_NO, 2)

static volatile uint32_t *gpio_base = NULL;

struct _gpio_dev_
{
	dev_t dev_num;
	struct cdev *cdev_dev;
	struct class *dev_cls;
	struct device *dev;
}gpio_dev;

struct _gpio_infor_dev
{
	uint16_t gpio_pin;
	uint16_t gpio_sel;
	uint16_t set_level;
	uint16_t input_count;
}gpio_infor_dev;


static int dev_open(struct inode *, struct file *);
static int dev_close(struct inode *, struct file *);
static ssize_t dev_read(struct file*, char __user *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char __user *, size_t, loff_t *);
static long dev_ioctl(struct file *, unsigned int, unsigned long);

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = dev_open,
	.release = dev_close,
	.read = dev_read,
	.write = dev_write,
	.unlocked_ioctl = dev_ioctl,
};

static void init_gpio_base(void)
{
	gpio_base = (volatile uint32_t *)ioremap(GPIO_BASE, TOTAL_GPIO_REG);
	printk("%d\n", *gpio_base);
}

static void release_gpio(void)
{
	iounmap(gpio_base);
	gpio_base = NULL;
}

static void set_func_pin(uint16_t pin, uint8_t func)
{
	uint16_t index_selreg;
	uint16_t shift;

	index_selreg = GPFSEL0 + pin/10;
	shift = (pin % 10) * 3;

	gpio_base[index_selreg] = gpio_base[index_selreg] | ((func * 0x07) << shift);
}

static int set_level_pin(uint16_t pin, uint8_t level)
{
	switch(level)
	{
		case 0:
			if(pin <= 31)
			{
				gpio_base[GPCLR0] |= 1 << pin;
			}
			else
			{
				pin = pin - 31;
				gpio_base[GPCLR1] |= 1 << pin;
			}
			break;

		case 1:
			if(pin <= 31)
			{
				gpio_base[GPSET0] |= 1 << pin;
			}
			else
			{
				pin = pin - 31;
				gpio_base[GPSET1] |= 1 << pin;
			}
			break;		

		default:
			printk("Wrong level for gpio pin\n");
			return -1;
			break;
	}
	return 0;
}

static uint8_t read_level_gpio(uint16_t pin)
{
	int level_pin;

	if(pin <= 31)
	{
		level_pin = gpio_base[GPLEV0] & (1 << pin);
		level_pin = level_pin >> pin;
	}
	else
	{
		pin = pin - 31;
		level_pin = gpio_base[GPLEV1] & (1 << pin);
		level_pin = level_pin >> pin;
	}
	return (uint8_t)level_pin;
}

static int blink_led_out(uint16_t pin)
{
	int i;
	uint8_t level_pin;

	for(i = 0; i < 20; i++)
	{
		level_pin = read_level_gpio(pin);

		switch(level_pin)
		{
			case 0:
				set_level_pin(pin, 1);
				break;

			case 1:
				set_level_pin(pin, 0);
				break;

			default:
				break;		
		}	
	}
	return 0;
}

static int dev_open(struct inode *inodep, struct file *filep)
{
	init_gpio_base();

	gpio_infor_dev.gpio_pin = 21;
	printk("open gpio pin %d", gpio_infor_dev.gpio_pin);
	return 0;
}

static int dev_close(struct inode *inodep, struct file *filep)
{
	printk("close gpio pin %d", gpio_infor_dev.gpio_pin);
	gpio_infor_dev.gpio_pin = 0;

	release_gpio();

	return 0;
}

static ssize_t dev_read(struct file*filep, char __user *buf, size_t len, loff_t *offset)
{
	size_t buff_len;
	char *kernel_buff;
	
	kernel_buff = kzalloc(sizeof(uint8_t), GFP_KERNEL);

	buff_len = snprintf(kernel_buff, sizeof(uint8_t), "%d", read_level_gpio(gpio_infor_dev.gpio_pin));

	if(copy_to_user(buf, kernel_buff, buff_len))
	{
		printk("fail to get data from pin\n");
		return -EFAULT;
	}
	kfree(kernel_buff);
	return buff_len;
}

static ssize_t dev_write(struct file*filep, const char __user *buf, size_t len, loff_t *offset)
{
	char *kernel_buff;
	char *buff_cmd;

	kernel_buff = kzalloc(len, GFP_KERNEL);
	buff_cmd 	= kzalloc(len, GFP_KERNEL);

	if(NULL == kernel_buff)
	{
		printk("fail to allocate select gpio pin\n");
		return 0;
	}

	if(copy_from_user(kernel_buff, buf, len))
	{
		printk("fail to get data from user buffer\n");
		return -EFAULT;
	}

	snprintf(buff_cmd, len, "%s", kernel_buff);

	if(strcmp(buff_cmd, "in") == 0)
	{
		printk("in\n");
		gpio_infor_dev.gpio_sel = 0;
		set_func_pin(gpio_infor_dev.gpio_pin, 0x00);
	}
	else if(strcmp(buff_cmd, "out") == 0)
	{
		printk("out\n");
		gpio_infor_dev.gpio_sel = 1;
		set_func_pin(gpio_infor_dev.gpio_pin, 0x01);
	}
	else if(strcmp(buff_cmd, "high") == 0)
	{
		printk("high\n");
		gpio_infor_dev.set_level = 1;
		set_level_pin(gpio_infor_dev.gpio_pin, 0x01);
	}
	else if(strcmp(buff_cmd, "low") == 0)
	{
		printk("low\n");
		gpio_infor_dev.set_level = 1;
		set_level_pin(gpio_infor_dev.gpio_pin, 0x00);
	}
	else
	{
		printk("wrong syntax for gpio function\n");
	}
	kfree(kernel_buff);
	return len;
}

static long dev_ioctl(struct file *flip, unsigned int cmd, unsigned long arg)
{
	// int value;

	switch(cmd)
	{
		case OUTPUT_LED:
			set_func_pin(gpio_infor_dev.gpio_pin, 1);
			break;

		case INPUT_LED:
			set_func_pin(gpio_infor_dev.gpio_pin, 0);
			break;

		case BLINK_LED:
			blink_led_out(gpio_infor_dev.gpio_pin);	
			break;

		default:
			break;	
	}
	return 0;
}

static int __init gpio_init(void)
{
	int ret_val;

	ret_val = alloc_chrdev_region(&gpio_dev.dev_num, 0, 1, "gpio_dev");
	if(ret_val)
	{
		printk("can not register major no\n");
		return ret_val;
	}

	printk(KERN_INFO "register successfully major %d and minor %d\n", MAJOR(gpio_dev.dev_num), MINOR(gpio_dev.dev_num));

	gpio_dev.dev_cls = class_create(THIS_MODULE, "gpio_class_dev");
	if(NULL == gpio_dev.dev_cls)
	{
		printk("cannot register class device file for device\n");
		goto fail_register_class;
	}

	gpio_dev.dev = device_create(gpio_dev.dev_cls, NULL, gpio_dev.dev_num, NULL, "dev_gpio");
	if(NULL == gpio_dev.dev)
	{
		printk("cannot register device file for device\n");
		goto fail_register_device;
	}

	gpio_dev.cdev_dev = cdev_alloc();
	if(NULL == gpio_dev.cdev_dev)
	{
		printk("Fail to allocate memory for cdev\n");
		goto fail_alloc_cdev;
	}
	cdev_init(gpio_dev.cdev_dev, &fops);
	ret_val = cdev_add(gpio_dev.cdev_dev, gpio_dev.dev_num, 1);
	if(0 > ret_val)
	{
		printk("fail to add device file into device number\n");
		goto fail_alloc_cdev;
	}
	printk("successfully create gpio device driver\n");

	return 0;

fail_alloc_cdev:
	device_destroy(gpio_dev.dev_cls, gpio_dev.dev_num);
fail_register_device:
	class_destroy(gpio_dev.dev_cls);
fail_register_class:
	unregister_chrdev_region(gpio_dev.dev_num, 3);	
	return ret_val;
}

static void __exit gpio_exit(void)
{
	printk("Release gpio device driver\n");

	cdev_del(gpio_dev.cdev_dev);

	device_destroy(gpio_dev.dev_cls, gpio_dev.dev_num);

	class_destroy(gpio_dev.dev_cls);

	unregister_chrdev_region(gpio_dev.dev_num, 3);	

	printk("Driver is removed\n");
}

module_init(gpio_init);
module_exit(gpio_exit);

MODULE_LICENSE("GPL");