#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/string.h>

typedef struct _hw_dev
{
	unsigned char *data_reg;
	unsigned char *control_reg;
	unsigned char *status_reg;
}HW_DEV;

struct _exam_char
{
	dev_t t_dev;
	struct cdev *t_cdev;
	struct class *dev_cls;
	struct device *dev;
	HW_DEV *hw_reg;
	char reg_value;
}exam_char;

static int dev_open(struct inode *, struct file *);
static int dev_close(struct inode *, struct file *);
static ssize_t dev_read(struct file*, char __user *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char __user *, size_t, loff_t *);
static int buff_str_cmp(const char *buff, const char *str, int len_buf);
static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = dev_open,
	.release = dev_close,
	.read = dev_read,
	.write = dev_write,
};

static int buff_str_cmp(const char *buff, const char *str, int len_buf)
{
	int len_temp;
	int len_str;
	int i, j;
	len_temp = len_buf;
	len_str = sizeof(str);
	printk("sizeof str: %d\n", len_str);

	for( i = 0; i < len_buf; i++)
	{
		for( j = 0; j < len_str; j++)
		{
			
			if(buff[i] == str[j])
			{
				printk("buff[i] %c and str[j] %c\n", buff[i], str[j]);
				printk("value: %d\n", len_temp);
				len_temp--;
				break;
			}
		}
	}
	printk("%d\n", len_temp);
	if(len_temp == (len_buf - len_str))
	{
		return 1;
	}
	return 0;
}

static int dev_open(struct inode *inodep, struct file *filep)
{
	printk("open character device\n");
	unsigned char *buff_reg;
	buff_reg = kzalloc(3, GFP_KERNEL);

	exam_char.hw_reg->data_reg = buff_reg;
	exam_char.hw_reg->control_reg = exam_char.hw_reg->data_reg + 1;
	exam_char.hw_reg->status_reg  = exam_char.hw_reg->data_reg + 2;

	return 0;
}
static int dev_close(struct inode *inodep, struct file *filep)
{
	printk("close\n");
	kfree(exam_char.hw_reg->data_reg);
	return 0;
}

static ssize_t dev_read(struct file*filep, char __user *buf, size_t len, loff_t *offset)
{
	printk("read\n");
	
	char *buff_reg;

	buff_reg = kzalloc(len, GFP_KERNEL);

	memcpy(buff_reg, exam_char.hw_reg->data_reg, 3);

	if(copy_to_user(buf, buff_reg, 3))
	{
		return -EFAULT;
	}
	printk("successfully to copy data %d\n", (int *)exam_char.hw_reg->data_reg);

	return 0;
}
static ssize_t dev_write(struct file*filep, const char __user *buf, size_t len, loff_t *offset)
{

	printk("write\n");

	char *buff_reg;
	char *buff_str;

	buff_reg = kzalloc(len, GFP_KERNEL);
	buff_str = kzalloc(len, GFP_KERNEL);
	exam_char.reg_value = 0;

	if(copy_from_user(buff_reg, buf, len))
	{
		return -EFAULT;
	}
	printk("lenght of buff is %d \n", len);
	buff_reg[len] = '\0';
	snprintf(buff_str, len, "%s", buff_reg);
	if(strcmp(buff_str, "control") == 0)
	{
		printk("control reg is selected\n");
		exam_char.hw_reg->control_reg = exam_char.reg_value;
	}
	else if(strcmp(buff_str, "status") == 0)
	{
		printk("status reg is selected\n");
		exam_char.hw_reg->status_reg = exam_char.reg_value;
	}
	else if(strcmp(buff_str, "data") == 0)
	{
		printk("data reg is selected\n");
		exam_char.hw_reg->data_reg = exam_char.reg_value;
	}
	else
	{
		exam_char.reg_value = buff_reg[0];
	}
	return len;
}

static int __init exam_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&exam_char.t_dev, 0, 1, "example_char_dev");
	if(ret)
	{
		printk("can not register major no\n");
		return ret;
	}
	printk(KERN_INFO "register successfully major now is: %d\n", MAJOR(exam_char.t_dev));
	
	exam_char.dev_cls = class_create(THIS_MODULE, "example_class_dev");
	if(exam_char.dev_cls == NULL)
	{
		printk("cannot register class for device\n");
		goto fail_register_class;
		
	}

	exam_char.dev = device_create(exam_char.dev_cls, NULL, exam_char.t_dev, NULL, "exam_dev");
	if(exam_char.dev == NULL)
	{
		printk("cannot register device file for device\n");
		goto fail_register_device;
	}

	exam_char.t_cdev = cdev_alloc();
	if(exam_char.t_cdev == NULL)
	{
		printk("cannot allocate entry point\n");
		goto fail_alloc_cdev;
	}
	cdev_init(exam_char.t_cdev, &fops);
	ret = cdev_add(exam_char.t_cdev, exam_char.t_dev, 1);
	if(ret < 0)
	{
		printk("fail to add device file into device number\n");
		goto fail_alloc_cdev;
	}

	exam_char.hw_reg = kzalloc(sizeof(HW_DEV), GFP_KERNEL);
	if(exam_char.hw_reg == NULL)
	{
		printk("Fail to allocate memory\n");
		goto fail_alloc_memory;
	}
	else
	{
		printk("Successful to allocate memory\n");
	}

	printk("successfully for create device driver\n");

	return 0;

fail_alloc_memory:
	cdev_del(exam_char.t_cdev);

fail_alloc_cdev:
	device_destroy(exam_char.dev_cls, exam_char.t_dev);

fail_register_device:
	class_destroy(exam_char.dev_cls);

fail_register_class:	
	unregister_chrdev_region(exam_char.t_dev, 0);

	return -1;
}

static void __exit exam_exit(void)
{
	printk("goodbye\n");

	kfree(exam_char.hw_reg);

	cdev_del(exam_char.t_cdev);
	
	device_destroy(exam_char.dev_cls, exam_char.t_dev);
	
	class_destroy(exam_char.dev_cls);
	
	unregister_chrdev_region(exam_char.t_dev, 0);
}

module_init(exam_init);
module_exit(exam_exit);

MODULE_LICENSE("GPL");