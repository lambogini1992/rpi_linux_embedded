#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>

#define NO_DEV			1
#define FIFO_SIZE		32

static DEFINE_MUTEX(write_lock);
static DEFINE_MUTEX(read_lock);

// static DECLARE_KFIFO(kfifo_ex, unsigned char, FIFO_SIZE);

struct exam_char
{
	dev_t t_dev;
	struct cdev t_cdev;
	struct class *dev_cls;
	struct device *dev;
	char reg_value;
	struct kfifo *kfifo_ex;
	struct mutex write_lock;
	struct mutex read_lock;
};

static int fifo_open(struct inode *, struct file *);
static int fifo_close(struct inode *, struct file *);
static ssize_t fifo_read(struct file*, char __user *, size_t, loff_t *);
static ssize_t fifo_write(struct file *, const char __user *, size_t, loff_t *);

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = fifo_open,
	.release = fifo_close,
	.read = fifo_read,
	.write = fifo_write,
};

struct exam_char *exam_char;

static int fifo_write_from_user(struct kfifo *kfifo, char __user *usbuf, size_t len)
{
	char *kbuf = NULL;
	int ret;

	kbuf = (char *)kzalloc(len, GFP_KERNEL);
	if(!kbuf)
	{
		printk("Fail to allocate memory for kbuf\n");
		return -1;
	}

	// printk("Start to get data from user\n");

	ret = copy_from_user(kbuf, usbuf, len);
	if(ret)
	{
		printk("Fail to get data from user\n");
		return ret;
	}

	
	kfifo_in(kfifo, kbuf, len);

	kfree(kbuf);

	return kfifo_avail(kfifo);
}

static int fifo_read_for_user(struct kfifo *kfifo, char __user *usbuf, size_t len)
{
	char *kbuf;
	int ret;

	printk("Lenght of request get data %d\n", len);
	kbuf = (char *)kzalloc(len, GFP_KERNEL);
	if(!kbuf)
	{
		printk("Fail to allocate memory for kbuf\n");
		return -1;
	}
	
	kfifo_out(kfifo, kbuf, len);

	printk("Get data to kfifo %s\n", kbuf);
	ret = copy_to_user(usbuf, kbuf, len);
	if(ret)
	{
		printk("Fail to get data for user\n");
		return ret;
	}

	kfree(kbuf);
	
	return kfifo_len(kfifo);
}

static int fifo_open(struct inode *inodep, struct file *filep)
{
	struct exam_char *dev_char = NULL;
	int ret;
	printk("open character device\n");
	dev_char = container_of(inodep->i_cdev, struct exam_char, t_cdev);
	if(dev_char == NULL)
	{
		printk("fail to get data of exam_char\n");
	}
	else
	{

		filep->private_data = dev_char;
		printk("successfully to get data of exam_char\n");
	}
	return 0;
}

static int fifo_close(struct inode *inodep, struct file *filep)
{
	struct exam_char *dev_char = NULL;
	printk("close character device\n");
	dev_char = container_of(inodep->i_cdev, struct exam_char, t_cdev);
	if(dev_char == NULL)
	{
		printk("fail to get data of exam_char\n");
	}
	else
	{
		filep->private_data = NULL;
	}

	return 0;
}

static ssize_t fifo_read(struct file *filep, char __user *buf, size_t len, loff_t *offset)
{
	struct exam_char *dev_char = NULL;
	size_t len_kfifo;
	int ret;
	printk("read\n");

	dev_char = filep->private_data;
	if(kfifo_is_empty(dev_char->kfifo_ex))
	{
		printk("KFIFO is empty\n");
		return 0;
	}

	len_kfifo = kfifo_len(dev_char->kfifo_ex);
	if(len > len_kfifo)
	{
		len = len_kfifo;
	}

	if(mutex_lock_interruptible(&dev_char->read_lock))
	{
		return -ERESTARTSYS; 
	}
	
	ret = fifo_read_for_user(dev_char->kfifo_ex, buf, len);
	
	mutex_unlock(&dev_char->read_lock);
	len_kfifo = kfifo_len(dev_char->kfifo_ex);
	if(ret < 0)
	{
		printk("Fail to get data from KFIFO\n");
		return ret;
	}
	printk("Available data in FIFO %d to read\n", len_kfifo);

	return len;
}

static ssize_t fifo_write(struct file *filep, const char __user *buf, size_t len, loff_t *offset)
{
	int ret;
	size_t remaind_slot;
	struct exam_char *dev_char=NULL;
	printk("write\n");

	dev_char = filep->private_data;
	if(kfifo_is_full(dev_char->kfifo_ex))
	{
		printk("KFIFO is full\n");
		return 0;
	}

	remaind_slot = kfifo_avail(dev_char->kfifo_ex);
	if(len > remaind_slot)
	{
		len = remaind_slot;
	}

	if(mutex_lock_interruptible(&dev_char->write_lock))
	{
		return -ERESTARTSYS; 
	}
	ret = fifo_write_from_user(dev_char->kfifo_ex, buf, len);
	mutex_unlock(&dev_char->write_lock);
	remaind_slot = kfifo_avail(dev_char->kfifo_ex);
	if(ret < 0)
	{
		printk("Fail to write data into KFIFO\n");
		return ret;
	}

	printk("Available slot in FIFO %d\n", remaind_slot);

	
	return len;
}

static int __init kfifo_example_init(void)
{
	int ret;

	exam_char = (struct exam_char *)kmalloc(sizeof(struct exam_char), GFP_KERNEL);
	if(exam_char == NULL)
	{
		printk("Fail to allocate memory exam_char\n");
		return -1;
	}

	exam_char->kfifo_ex = kzalloc(sizeof(struct kfifo), GFP_KERNEL);
	if(!exam_char->kfifo_ex)
	{
		printk("Fail to allocate kfifo exam_char\n");
		goto fail_alloc_mem_kfifo;
	}

	ret = kfifo_alloc(exam_char->kfifo_ex, FIFO_SIZE, GFP_KERNEL);
	if(ret)
	{
		printk("Fail to init KFIFO\n");
		goto fail_init_kfifo;
	}

	ret = alloc_chrdev_region(&exam_char->t_dev, 0, NO_DEV, "example_char_dev");
	if(ret)
	{
		printk("can not register major alloc_chrdev_region\n");
		goto fail_alloc_chrdev_region;
	}
	printk(KERN_INFO "register successfully major now is: %d\n", MAJOR(exam_char->t_dev));
	


	exam_char->dev_cls = class_create(THIS_MODULE, "example_class_dev");
	if(exam_char->dev_cls == NULL)
	{
		printk("cannot register class for device\n");
		goto fail_register_class;
		
	}

	// for(dev_idx = 0; dev_idx < NO_DEV; dev_idx++)
	{
		exam_char->dev = device_create(exam_char->dev_cls, NULL, MKDEV(MAJOR(exam_char->t_dev), \
			MINOR(exam_char->t_dev)), NULL, "kfifo_example%d", 1);
		if(exam_char->dev == NULL)
		{
			printk("cannot register device file for device\n");
			goto fail_register_device;
		}

		// exam_char->t_cdev = cdev_alloc();
		// if(exam_char->t_cdev == NULL)
		// {
		// 	printk("cannot allocate entry point\n");
		// 	goto fail_alloc_cdev;
		// }
		cdev_init(&exam_char->t_cdev, &fops);
		ret = cdev_add(&exam_char->t_cdev, MKDEV(MAJOR(exam_char->t_dev), \
			MINOR(exam_char->t_dev)), 1);
		if(ret < 0)
		{
			printk("fail to add device file into device number\n");
			goto fail_alloc_cdev;
		}
	}

	mutex_init(&exam_char->write_lock);
	mutex_init(&exam_char->read_lock);

	printk("successfully for create device driver\n");

	return 0;

fail_alloc_cdev:
	device_destroy(exam_char->dev_cls, MKDEV(MAJOR(exam_char->t_dev), \
						MINOR(exam_char->t_dev)));
fail_register_device:
	class_destroy(exam_char->dev_cls);

fail_register_class:	
	unregister_chrdev_region(exam_char->t_dev, 0);

fail_alloc_chrdev_region:
	kfifo_free(exam_char->kfifo_ex);
fail_init_kfifo:
	kfree(exam_char->kfifo_ex);
fail_alloc_mem_kfifo:
	kfree(exam_char);

// fail_tp_aloc_chrdev_region:
// 	kfifo_free(exam_char->kfifo_ex);

	return -1;
}

static void __exit kfifo_example_exit(void)
{
	printk("goodbye\n");

	// kfifo_free(exam_char->kfifo_ex);
	// for(dev_idx = 0; dev_idx < NO_DEV; dev_idx++)
	{
		cdev_del(&exam_char->t_cdev);
	
		device_destroy(exam_char->dev_cls, MKDEV(MAJOR(exam_char->t_dev), \
			MINOR(exam_char->t_dev)));
	}

	
	class_destroy(exam_char->dev_cls);
	
	unregister_chrdev_region(exam_char->t_dev, 0);

	kfifo_free(exam_char->kfifo_ex);

	kfree(exam_char->kfifo_ex);

	kfree(exam_char);
}

module_init(kfifo_example_init);
module_exit(kfifo_example_exit);

MODULE_LICENSE("GPL");