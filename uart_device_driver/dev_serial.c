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
#include <linux/tty_driver.h>
#include <linux/serdev.h>


#define DRIVER_NAME "mcu_serial"
#define DEV_NAME    "serial_mcu"



MODULE_LICENSE("GPL");

static volatile uint32_t *gpio_base = NULL;
struct resource *res;
uint8_t set_level;

static struct tty_driver *mcu_uart_tty_driver;


/*Register character device*/
static int mcu_uart_open(struct tty_struct *tty, struct file *filp);
static void mcu_uart_close(struct tty_struct *tty, struct file * filp);
static ssize_t read_uart_mcu(struct file*, char __user *, size_t, loff_t *);
static int sdio_uart_write(struct tty_struct *tty, const unsigned char *buf, int count);
static long ioclt_uart_mcu(struct file *, unsigned int, unsigned long);


static const struct tty_operations mcu_uart_ops = {
	.open				= mcu_uart_open,
	.close				= mcu_uart_close,
	// .write				= mcu_uart_write,
	// .write_room			= mcu_uart_write_room,
	// .chars_in_buffer	= mcu_uart_chars_in_buffer,
	// .send_xchar			= mcu_uart_send_xchar,
	// .throttle			= mcu_uart_throttle,
	// .unthrottle			= mcu_uart_unthrottle,
	// .set_termios		= mcu_uart_set_termios,
	// .hangup				= mcu_uart_hangup,
	// .break_ctl			= mcu_uart_break_ctl,
	// .tiocmget			= mcu_uart_tiocmget,
	// .tiocmset			= mcu_uart_tiocmset,
	// .install			= mcu_uart_install,
	// .cleanup			= mcu_uart_cleanup,
	// .proc_show			= mcu_uart_proc_show,
};
/*FUNCTION DEVICE FILE*/
static int mcu_uart_open(struct tty_struct *tty, struct file *filp)
{
	printk("Open Driver file\n");
	return 0;
}

static void mcu_uart_close(struct tty_struct *tty, struct file * filp)
{
	printk("Close Driver Device File\n");
	return 0;
}

/*Using for init device type*/
/*Start create device platform driver*/
static int device_probe(struct platform_device *pdev);
static int device_remove(struct platform_device *pdev);





static int device_probe(struct platform_device *pdev)
{
	int ret_val;
	const struct  of_device_id *match;
	int res_map_size;
	struct tty_driver *tty_drv;

	printk(KERN_INFO "Hello! This is UART MCU DEVICES\n");

	res = NULL;

	// match = of_match_device(test_uart_of_match, &(pdev->dev));
	// if(!match)
	// {
	// 	printk(KERN_ALERT "Fail to matching device and device table\n");
	// 	return -EINVAL;
	// }

	// mcu_uart_tty_driver = tty_drv = alloc_tty_driver(1);

	// if (!tty_drv)
	// 	return -ENOMEM;

	// tty_drv->driver_name 			= "mcu_uart_tty";
	// tty_drv->name        			= "tty_mcu";
	// tty_drv->major		 			= 0;
	// tty_drv->minor_start 			= 0;
	// tty_drv->type 		 			= TTY_DRIVER_TYPE_SERIAL;
	// tty_drv->subtype 	 			= SERIAL_TYPE_NORMAL;
	// tty_drv->flags 					= TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	// tty_drv->init_termios 			= tty_std_termios;
	// tty_drv->init_termios.c_cflag 	= B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	// tty_drv->init_termios.c_ispeed 	= 9600;
	// tty_drv->init_termios.c_ospeed 	= 9600;

	// tty_set_operations(tty_drv, &mcu_uart_ops);

	// ret_val = tty_register_driver(tty_drv);
	// if(ret_val)
	// {
	// 	printk(KERN_ERR "FAIL TO REGISTER TTY DEVICE DRIVER \n\n");
	// 	goto fail_regs_tty;
	// }

	return 0;
// fail_regs_mcu_uart:
// 	tty_unregister_driver(tty_drv);
// fail_regs_tty:
// 	put_tty_driver(tty_drv);
// 	return ret_val;
}

static int device_remove(struct platform_device *pdev)
{
	printk("Goodbye MCU DEVICE SERIAL\n");
	// tty_unregister_driver(mcu_uart_tty_driver);
	// put_tty_driver(mcu_uart_tty_driver);

	return 0;
}

static const struct of_device_id test_uart_of_match[] =
{
	{.compatible = "uart, mcu_stm32f4", NULL},
	{},
};

MODULE_DEVICE_TABLE(of, test_uart_of_match);

static struct platform_driver mcu_uart_pldriver = {
	.probe 		= device_probe,
	.remove		= device_remove,
	.id_table   = test_uart_of_match,
	.driver  	= {
		.name 				= DRIVER_NAME,
		.of_match_table		= test_uart_of_match,
	},
};

/*Register device to system linux*/
module_platform_driver(mcu_uart_pldriver);

// static int __init initmodule(void)
// {
// 	printk(KERN_INFO ">>>>>>>>START DEVICE INIT<<<<<<<<<<\n");

// 	platform_driver_register(&mcu_uart_pldriver);

// 	return 0;
// }

// static void __exit exitmodule(void)
// {
// 	platform_driver_unregister(&mcu_uart_pldriver);

// 	printk(KERN_INFO ">>>>>>>>>>>>EXIT DEVICE<<<<<<<<<<<\n");
// 	return;
// }

// module_init(initmodule);
// module_exit(exitmodule);
