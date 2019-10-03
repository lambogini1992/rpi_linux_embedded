p
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
#include <linux/serial_core.h>


#define DRIVER_NAME "DEV_SERIAL"
#define DEV_NAME    "tty_dev_s"

/*THIS IS DATA BUFFER IF BIT DLAB = 0*/
#define AUX_MU_IO_REG								0

/*ENABLE INTERRUPT IF BIT DLAB = 0*/
/*SET BAUDRATE IF BIT DLAB = 1*/
#define AUX_MU_IIR_REG							1

/*THIS REGISTER IS SHOW THE INTERRUPT STATUS*/
/*CAN USING FOR CLEAR FIFO REVC AND TRANS BUFFER*/
#define AUX_MU_IER_REG							2

/*THIS REGISTER IS USED FOR CONTROL LINE DATA FORMAT
**GIVES ACCESS BAUDRATE REGISTER*/
#define AUX_MU_LCR_REG 							3

/*register controls the 'modem' signals*/
#define AUX_MU_MCR_REG 							4

/*register shows the data status*/
#define AUX_MU_LSR_REG 							5

/*register shows the 'modem' status*/
#define AUX_MU_MSR_REG							6

#define AUX_MU_SCRATCH							7

/*provides access to some extra useful and nice features not
**found on a normal 16550 UART*/
#define AUX_MU_CNTL_REG 						8

/*provides a lot of useful information about the internal status*/
#define AUX_MU_STAT_REG 						9

/*USING FOR SETTING BAUDRATE of device*/
#define AUX_MU_BAUD									10




MODULE_LICENSE("GPL");

static volatile uint32_t *gpio_base = NULL;
struct resource *res;
uint8_t set_level;

typedef struct _bcm_uart_
{
	struct uart_port port;

}


struct _exam_dev_
{
	dev_t dev_num;
	struct cdev *cdev_dev;
	struct class *dev_cls;
	struct device *dev;
}exam_dev;

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


static inline void uart_write_reg(struct uart_port *port, u32 reg, u32 val)
{
	__raw_write(val, port->membase + reg);
}

static inline u32 uart_read_reg(struct uart_port *port, u32 reg)
{
	 return __raw_read(port->membase + reg);
}

static inline

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
