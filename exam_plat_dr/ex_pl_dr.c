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
//for platform drivers....
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "Sample_Pldrv"

MODULE_LICENSE("GPL");

struct _exam_dev_
{
	dev_t dev_num;
	struct cdev *cdev_dev;
	struct class *dev_cls;
	struct device *dev;
}exam_dev;

static int dev_open(struct inode *, struct file *);
static int dev_close(struct inode *, struct file *);
static ssize_t dev_read(struct file*, char __user *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char __user *, size_t, loff_t *);

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = dev_open,
	.release = dev_close,
	.read = dev_read,
	.write = dev_write,
};

/***************/
static const struct of_device_id example_device_table[]=
{
	{ .compatible = "test,simple-misc-dev", },
	{},
};

MODULE_DEVICE_TABLE(of,example_device_table);

/*********CHARACTER DEVICE*********/
static int dev_open(struct inode *inodep, struct file *filep)
{
	printk("Open example file\n");
	return 0;
}

static int dev_close(struct inode *inodep, struct file *filep)
{
	printk("Close example file\n");
	return 0;
}

static ssize_t dev_read(struct file*filep, char __user *buf, size_t len, loff_t *offset)
{
	printk("Read example file\n");
	return 0;
}

static ssize_t dev_write(struct file*filep, const char __user *buf, size_t len, loff_t *offset)
{
	printk("Write example file\n");
	return 0;
}
/**************/
static int sample_drv_probe(struct platform_device *pdev)
{
	int retval;
	const struct of_device_id *match;
	// int rc = 0;
	struct resource *res;
	unsigned long res_map_size;
	void *registers;
	printk("\nInitialize Driver\n");

	match = of_match_device(example_device_table, &(pdev->dev));
	if(!match)
	{
		printk("\nFail to matching table device with platform device\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res_map_size = res->end - res->start + 1;

	printk("Start reg_add of device is %x and size is %i\n", res->start, res_map_size);

	retval = alloc_chrdev_region(&exam_dev.dev_num, 0, 1, "exam_dev");
	if(retval)
	{
		printk("Fail to allocate device driver\n");
		return retval;
	}
	printk(KERN_INFO "Register successfully with major %i and minor %i\n", MAJOR(exam_dev.dev_num), MINOR(exam_dev.dev_num));

	exam_dev.dev_cls = class_create(THIS_MODULE, "exam_class_dev");
	if(NULL == exam_dev.dev_cls)
	{
		printk("FAIL TO CREATE CLASS DEVICE\n");
		goto FAIL_TO_CREATE_CLASS;
	}

	exam_dev.dev = device_create(exam_dev.dev_cls, NULL, exam_dev.dev_num, NULL, "dev_exam");
	if(NULL == exam_dev.dev)
	{
		printk("FAIL TO CREATE DEVICE FILE");
		goto FAIL_TO_CREATE_DEVICE;
	}

	exam_dev.cdev_dev = cdev_alloc();
	if(NULL == exam_dev.cdev_dev)
	{
		printk("FAIL to ALLOCATION CDEV\n");
		goto FAIL_TO_ALLOC_CDEV;
	}

	cdev_init(exam_dev.cdev_dev, &fops);
	retval = cdev_add(exam_dev.cdev_dev, exam_dev.dev_num, 1);
	if(0 > retval)
	{
		printk("fail to add device file into device number\n");
		goto FAIL_TO_INIT_CDEV;
	}
	printk("Create Device File is successfully\n");
	return 0;

FAIL_TO_INIT_CDEV:
	cdev_del(exam_dev.cdev_dev);
FAIL_TO_ALLOC_CDEV:
	device_destroy(exam_dev.dev_cls, exam_dev.dev_num);
FAIL_TO_CREATE_DEVICE:
	class_destroy(exam_dev.dev_cls);
FAIL_TO_CREATE_CLASS:
	unregister_chrdev_region(exam_dev.dev_num, 1);
	return retval;
}
static int sample_drv_remove(struct platform_device *pdev)
{
	printk("\nRemove driver\n");

	cdev_del(exam_dev.cdev_dev);

	device_destroy(exam_dev.dev_cls, exam_dev.dev_num);

	class_destroy(exam_dev.dev_cls);

	unregister_chrdev_region(exam_dev.dev_num, 1);

	return 0;
}

static struct platform_driver sample_pldriver = {
    .probe          = sample_drv_probe,
    .remove         = sample_drv_remove,
    .driver = {
            .name  = DRIVER_NAME,
            .owner = THIS_MODULE,
            .of_match_table = of_match_ptr(example_device_table),
    },
};
/**************/

static int __init ourinitmodule(void)
{
    printk(KERN_ALERT "\n Welcome to sample Platform driver.... \n");

    /* Registering with Kernel */
    platform_driver_register(&sample_pldriver);

    return 0;
}

static void __exit ourcleanupmodule(void)
{
    printk(KERN_ALERT "\n Thanks....Exiting sample Platform driver... \n");

    /* Unregistering from Kernel */
    platform_driver_unregister(&sample_pldriver);

    return;
}

module_init(ourinitmodule);
module_exit(ourcleanupmodule);
