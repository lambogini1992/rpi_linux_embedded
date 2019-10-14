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
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/tty_ldisc.h>
#include <linux/tty_flip.h>


#define DRIVER_NAME "SERIAL_MCU_DRIVERs"
#define DEV_NAME    "serial_mcu"



MODULE_LICENSE("GPL");

static volatile uint32_t *gpio_base = NULL;
struct resource *res;
uint8_t set_level;



/*Register character device*/
static int open_uart_mcu(struct inode *, struct file *);
static int close_uart_mcu(struct inode *, struct file *);
static ssize_t read_uart_mcu(struct file*, char __user *, size_t, loff_t *);
static ssize_t write_uart_mcu(struct file *, const char __user *, size_t, loff_t *);
static long ioclt_uart_mcu(struct file *, unsigned int, unsigned long);


static const struct tty_operations mcu_uart_ops = {
	.open				= mcu_uart_open,
	.close				= mcu_uart_close,
	.write				= mcu_uart_write,
	.write_room			= mcu_uart_write_room,
	.chars_in_buffer	= mcu_uart_chars_in_buffer,
	.send_xchar			= mcu_uart_send_xchar,
	.throttle			= mcu_uart_throttle,
	.unthrottle			= mcu_uart_unthrottle,
	.set_termios		= mcu_uart_set_termios,
	.hangup				= mcu_uart_hangup,
	.break_ctl			= mcu_uart_break_ctl,
	.tiocmget			= mcu_uart_tiocmget,
	.tiocmset			= mcu_uart_tiocmset,
	.install			= mcu_uart_install,
	.cleanup			= mcu_uart_cleanup,
	.proc_show			= mcu_uart_proc_show,
};
/*FUNCTION DEVICE FILE*/
static int mcu_uart_open(struct inode *inode, struct file *fp)
{
	printk("Open Driver file\n");
	printk("Kernel is set GPIO PIN 18 to become OUTPUT\n");
	return 0;
}

static int mcu_uart_close(struct inode *inode, struct file *fp)
{
	printk("Close Driver Device File\n");
	return 0;
}

static ssize_t read_uart_mcu(struct file *fp, char __user *ubuf, size_t size_of, loff_t *offset)
{
	return size_of;
}

static ssize_t mcu_uart_write(struct file *fp, const char __user *buf, size_t size_of, loff_t *offset)
{
	return size_of;
}

static long ioclt_uart_mcu(struct file *fops, unsigned int cmd, unsigned long len)
{
	return len;
}
/*Using for init device type*/
/*Start create device platform driver*/
static int device_probe(struct platform_device *pdev);
static int device_remove(struct platform_device *pdev);

static const struct of_device_id test_uart_of_match[] =
{
	{.compatible = "bcm, mcu_uart", },
	{},
};

MODULE_DEVICE_TABLE(of, test_uart_of_match);



static int device_probe(struct platform_device *pdev)
{
	int ret_val;
	const struct  of_device_id *match;
	int res_map_size;

	printk(KERN_INFO "Hello! This is UART DEVICES\n");

	res = NULL;

	match = of_match_device(test_uart_of_match, &(pdev->dev));
	if(!match)
	{
		printk(KERN_ALERT "Fail to matching device and device table\n");
		return -EINVAL;
	}



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
