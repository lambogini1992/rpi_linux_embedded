
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
/*for character device*/
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
/*using for platform device*/
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>


#define DRIVER_NAME "DEV_SERIAL"



MODULE_LICENSE("GPL");

static volatile uint32_t *gpio_base = NULL;
struct resource *res;
uint8_t set_level;


struct _exam_dev_
{
	dev_t dev_num;
	struct cdev *cdev_dev;
	struct class *dev_cls;
	struct device *dev;
}exam_dev;

/*Function declared for gpio*/
static void set_direction(uint8_t gpio_pin);
static void set_pin_level(uint8_t gpio_pin, uint8_t level);
static int read_level_gpio(uint16_t pin);

/*Register character device*/
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
};

/*FUNCTION GPIO HARDWARE*/
static void set_direction(uint8_t gpio_pin)
{
	// int major_pin;
	// int minor_pin;
	//
	// major_pin = gpio_pin/10;
	// minor_pin = gpio_pin - (major_pin*10);
	//
	// switch(major_pin)
	// {
	// 	case 0:
	// 		gpio_base[GPFSEL0] |= 1 << minor_pin;
	// 		break;
	//
	// 	case 1:
	// 		gpio_base[GPFSEL1] |= 1 << minor_pin;
	// 		break;
	//
	// 	case 2:
	// 		gpio_base[GPFSEL2] |= 1 << minor_pin;
	// 		break;
	//
	// 	case 3:
	// 		gpio_base[GPFSEL0] |= 1 << minor_pin;
	// 		break;
	// }
}

static void set_pin_level(uint8_t gpio_pin, uint8_t level)
{
	// uint8_t pin;
	//
	// pin = gpio_pin;
	// switch(level)
	// {
	// 	case 0:
	// 		if(pin <= 31)
	// 		{
	// 			gpio_base[GPCLR0] |= 1 << pin;
	// 		}
	// 		else
	// 		{
	// 			pin = pin - 31;
	// 			gpio_base[GPCLR1] |= 1 << pin;
	// 		}
	// 		break;
	//
	// 	case 1:
	// 		if(pin <= 31)
	// 		{
	// 			gpio_base[GPSET0] |= 1 << pin;
	// 		}
	// 		else
	// 		{
	// 			pin = pin - 31;
	// 			gpio_base[GPSET1] |= 1 << pin;
	// 		}
	// 		break;
	//
	// 	default:
	// 		printk("Wrong level for gpio pin\n");
	// 		break;
	// }
}

static int read_level_gpio(uint16_t pin)
{
	// int level_pin;
	//
	// if(pin <= 31)
	// {
	// 	level_pin = gpio_base[GPLEV0] & (1 << pin);
	// 	level_pin = level_pin >> pin;
	// }
	// else
	// {
	// 	pin = pin - 31;
	// 	level_pin = gpio_base[GPLEV1] & (1 << pin);
	// 	level_pin = level_pin >> pin;
	// }
	return (int)0;
}
/*FUNCTION DEVICE FILE*/
static int dev_open(struct inode *inode, struct file *fp)
{
	printk("Open Driver file\n");
	set_direction(18);
	printk("Kernel is set GPIO PIN 18 to become OUTPUT\n");
	return 0;
}

static int dev_close(struct inode *inode, struct file *fp)
{
	printk("Close Driver Device File\n");
	return 0;
}

static ssize_t dev_read(struct file *fp, char __user *ubuf, size_t size_of, loff_t *offset)
{
	return size_of;
}

static ssize_t dev_write(struct file *fp, const char __user *buf, size_t size_of, loff_t *offset)
{
	char *kernel_buff = NULL;
	char *buff_cmd = NULL;

	kernel_buff = kzalloc(size_of, GFP_KERNEL);
	buff_cmd 	= kzalloc(size_of, GFP_KERNEL);

	if(NULL == kernel_buff)
	{
		printk("fail to allocate kernel_buff\n");
		return 0;
	}

	if(NULL == buff_cmd)
	{
		printk("fail to allocate buff_cmd\n");
		return 0;
	}

	if(copy_from_user(kernel_buff, buf, size_of))
	{
		printk("fail to get data from user buffer\n");
		return -EFAULT;
	}

	snprintf(buff_cmd, size_of, "%s", kernel_buff);

	if(strcmp(buff_cmd, "1") == 0)
	{
		set_pin_level(18, 1);
	}
	else if(strcmp(buff_cmd, "0") == 0)
	{
		set_pin_level(18, 0);
	}
	kfree(kernel_buff);
	kfree(buff_cmd);
	return size_of;
}

static long dev_ioctl(struct file *fops, unsigned int cmd, unsigned long len)
{
	return len;
}
/*Using for init device type*/
/*Start create device platform driver*/
static int device_probe(struct platform_device *pdev);
static int device_remove(struct platform_device *pdev);

static const struct of_device_id test_leds_of_match[] =
{
	{.compatible = "bcm, dev_serial", },
	{},
};

MODULE_DEVICE_TABLE(of, test_leds_of_match);



static int device_probe(struct platform_device *pdev)
{
	int ret_val;
	const struct  of_device_id *match;
	int res_map_size;

	printk(KERN_INFO "Hello! This is UART DEVICES\n");

	res = NULL;

	match = of_match_device(test_leds_of_match, &(pdev->dev));
	if(!match)
	{
		printk(KERN_ALERT "Fail to matching device and device table\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res)
	{
		printk(KERN_ALERT "Fail to get address register of device\n");
		return -EINVAL;
	}

	res_map_size = res->end -res->start + 1;

	printk(KERN_INFO "Register of device driver start at %x and has size is %x", res->start, res_map_size);

	gpio_base = ioremap_nocache(res->start, res_map_size);
	if(!gpio_base)
	{
		printk(KERN_ALERT "Fail to mapping register to kernel space\n");
		return -EINVAL;
	}

	ret_val = alloc_chrdev_region(&exam_dev.dev_num, 0, 1, "exam_dev");
	if(ret_val)
	{
		printk(KERN_ALERT "Fail to allocate character device\n");
		goto FAIL_ALLOCATE_CHRDEV;
	}
	printk(KERN_INFO "Allocate succesfully char device with major %d and minor %d\n", \
		MAJOR(exam_dev.dev_num), MINOR(exam_dev.dev_num));

	exam_dev.dev_cls = class_create(THIS_MODULE, "class_exam_dev_serial");
	if(NULL == exam_dev.dev_cls)
	{
		printk(KERN_ALERT "Fail to create class for device\n");
		goto FAIL_CREATE_CLS;
	}

	exam_dev.dev = device_create(exam_dev.dev_cls, NULL, exam_dev.dev_num, NULL, "dev_serial_exam");
	if(NULL == exam_dev.dev)
	{
		printk(KERN_ALERT "Fail to create device\n");
		goto FAIL_CREATE_DEV;
	}

	exam_dev.cdev_dev = cdev_alloc();
	if(NULL == exam_dev.cdev_dev)
	{
		printk(KERN_ALERT "Fail to allocate cdev");
		goto FAIL_ALLOCATE_CDEV;
	}

	cdev_init(exam_dev.cdev_dev, &fops);
	ret_val = cdev_add(exam_dev.cdev_dev, exam_dev.dev_num, 1);
	if(0 > ret_val)
	{
		printk(KERN_ALERT "Fail to add device file with device driver\n");
		goto FAIL_ADD_CDEV;
	}

	printk(KERN_INFO "Create platform device file in dev folder\n");

	return 0;
/*FAIL TO CREATE CHARACTER DEVICE*/
FAIL_ADD_CDEV:
	cdev_del(exam_dev.cdev_dev);

FAIL_ALLOCATE_CDEV:
	device_destroy(exam_dev.dev_cls, exam_dev.dev_num);

FAIL_CREATE_DEV:
	class_destroy(exam_dev.dev_cls);

FAIL_CREATE_CLS:
	unregister_chrdev_region(exam_dev.dev_num, 1);

FAIL_ALLOCATE_CHRDEV:
	iounmap(gpio_base);

	return ret_val;
}

static int device_remove(struct platform_device *pdev)
{
	printk("Goodbye DEVICE SERIAL\n");

	cdev_del(exam_dev.cdev_dev);

	device_destroy(exam_dev.dev_cls, exam_dev.dev_num);

	class_destroy(exam_dev.dev_cls);

	unregister_chrdev_region(exam_dev.dev_num, 1);

	iounmap(gpio_base);
	gpio_base = NULL;

	return 0;
}

static struct platform_driver test_leds_pldriver = {
	.probe 		= device_probe,
	.remove		= device_remove,
	.driver  	= {
		.name 						= DRIVER_NAME,
		.owner						= THIS_MODULE,
		.of_match_table		= of_match_ptr(test_leds_of_match),
	},
};

/*Register device to system linux*/

static int __init initmodule(void)
{
	printk(KERN_INFO ">>>>>>>>START DEVICE INIT<<<<<<<<<<\n");

	platform_driver_register(&test_leds_pldriver);

	return 0;
}

static void __exit exitmodule(void)
{
	platform_driver_unregister(&test_leds_pldriver);

	printk(KERN_INFO ">>>>>>>>>>>>EXIT DEVICE<<<<<<<<<<<\n");
	return;
}

module_init(initmodule);
module_exit(exitmodule);
