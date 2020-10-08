#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>

#define DEVICE_NAME "lcd_i2c"
#define CLASS_NAME "lcd"

#define LCD_MAX_ROW         2
#define LCD_MAX_COLUM       16

struct lcd_i2c_device
{
    struct i2c_client *client; // i2c client struct data
    struct kobject *kobj;       // Kernel object
    uint8_t max_rows;              // Lines of the display
    uint8_t max_columns;           // Columns
    uint8_t cur_rows;
    uint8_t cur_col;

    struct cdev *lcd_cdev;    
    struct device *dev;
    struct class *lcd_class;
    dev_t lcd_dev_t;
    // uint8_t address;           // I2C address shifted left by 1
    // uint8_t backlight;         // Backlight
    // uint8_t modeBits;          // Display on/off control bits
    // uint8_t entryBits;         // Entry mode set bits
};

static bool lcd_send_string (struct i2c_client *client,char *str);
static bool lcd_send_cmd (struct i2c_client *client, char cmd);
static bool lcd_send_data (struct i2c_client *client, char data);
static bool lcd_goto_XY(struct i2c_client *client, int row, int col); 

static bool lcd_send_cmd (struct i2c_client *client, char cmd)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd&0xf0);
    data_l = ((cmd<<4)&0xf0);
    data_t[0] = data_u|0x0C;  //en=1, rs=0
    data_t[1] = data_u|0x08;  //en=0, rs=0
    data_t[2] = data_l|0x0C;  //en=1, rs=0
    data_t[3] = data_l|0x08;  //en=0, rs=0

    if(0 > i2c_master_send(client, (const char *)data_t, 4))
    {
        printk(KERN_ERR "%s failed to send command\n", __func__);
        return false;
    }
    msleep(10);
    return true;
}

static bool lcd_send_data (struct i2c_client *client, char data)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data&0xf0);
    data_l = ((data<<4)&0xf0);
    data_t[0] = data_u|0x0D;  //en=1, rs=0
    data_t[1] = data_u|0x09;  //en=0, rs=0
    data_t[2] = data_l|0x0D;  //en=1, rs=0
    data_t[3] = data_l|0x09;  //en=0, rs=0
    if(0 > i2c_master_send(client, (const char *)data_t, 4))
    {
        printk(KERN_ERR "%s failed to write data\n", __func__);
        return false;
    }
    msleep(10);
    return true;
}

static bool lcd_init (struct i2c_client *client)
{
    struct lcd_i2c_device *lcd_dev = i2c_get_clientdata(client);

    lcd_dev->max_rows = LCD_MAX_ROW;
    lcd_dev->max_columns = LCD_MAX_COLUM;

    if(!lcd_send_cmd (client, 0x33)) /* set 4-bits interface */
    {
        printk(KERN_ERR "%s failed to set 4-bits interface for LCD at step 1\n", __func__);
        return false;
    }
    
    if(!lcd_send_cmd (client, 0x32))
    {
        printk(KERN_ERR "%s failed to set 4-bits interface for LCD at step 2\n", __func__);
        return false;
    }

    if(!lcd_send_cmd (client, 0x28)) /* start to set LCD function */
    {
        printk(KERN_ERR "%s failed to set LCD functions\n", __func__);
        return false;
    }

    if(!lcd_send_cmd (client, 0x01)) /* clear display */
    {
        printk(KERN_ERR "%s failed to clear LCD\n", __func__);
        return false;        
    }

    if(!lcd_send_cmd (client, 0x06)) /* set entry mode */
    {
        printk(KERN_ERR "%s failed to set entry mode for LCD\n", __func__);
        return false;          
    }

    if(!lcd_send_cmd (client, 0x0c)) /* set display to on */
    {
        printk(KERN_ERR "%s failed to set display on LCD\n", __func__);
        return false; 
    }    

    if(!lcd_send_cmd (client, 0x02)) /* move cursor to home and set data address to 0 */
    {
       printk(KERN_ERR "%s failed to move cursor to home and set data address to 0 LCD\n", __func__);
        return false;
    }

    if(!lcd_send_cmd (client, 0x80))
    {
       printk(KERN_ERR "%s failed to move cursor to home and set data address to 0 LCD\n", __func__);
        return false;
    }      

    if(!lcd_goto_XY (client, 1, 1))
    {
       printk(KERN_ERR "%s failed to move cursor to pos 1-1 LCD\n", __func__);
        return false;
    }  

    if(!lcd_send_string(client, "HELLO"))
    {
       printk(KERN_ERR "%s failed display HELLO str LCD\n", __func__);
        return false;
    }    

    return true;
}

static bool lcd_send_string(struct i2c_client *client,char *str)
{
    uint8_t idx = 0;
    while (*str) 
    {
        if(!lcd_send_data (client, *str++))
        {
            printk(KERN_ERR "%s failed to display character %d\n", __func__, idx);
            return false;
        }
        idx++;
    }
    return true;
}

static bool lcd_clear_display(struct i2c_client *client)
{
    return lcd_send_cmd (client, 0x01); //clear display
}

static bool lcd_goto_XY(struct i2c_client *client, int row, int col)
{
    static uint8_t rowOffsets[4] = { 0x00, 0x40, 0x14, 0x54 };
    struct lcd_i2c_device *lcd_dev = i2c_get_clientdata(client);
    uint8_t pos_Addr;

    if(row >= lcd_dev->max_rows)
    {
        printk(KERN_ERR "%s: row number is invalid\n", __func__);
        return false;
    }

    if(col >= lcd_dev->max_columns)
    {
        printk(KERN_ERR "%s: columns number is invalid\n", __func__);
        return false;
    }

    lcd_dev->cur_col    = col;
    lcd_dev->cur_rows   = row - 1;

    pos_Addr = 0x80 | (rowOffsets[lcd_dev->cur_rows] + lcd_dev->cur_col);

    return lcd_send_cmd(client, pos_Addr);
}


/*--------------Device file operation----------------*/

static int lcd_open(struct inode *inode, struct file *fp)
{
    return 0;
}


static int lcd_release(struct inode *inode, struct file *fp)
{
    return 0;
}


/*
 * always read the whole buffer
 */
static ssize_t lcd_read(struct file *fp, char *buf, size_t len, loff_t *off)
{
    return 0;
}

/*
 * start writing from gko_buffer_start
 */
static ssize_t lcd_write(struct file *fp, const char *buf, size_t len, loff_t *off)
{
    return 0;
}

static struct file_operations lcd_fops = {
        .owner = THIS_MODULE,
        .read = lcd_read,
        .write = lcd_write,
        .open = lcd_open,
        .release = lcd_release
};

static ssize_t lcd_details_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = NULL;
    struct lcd_i2c_device *lcd_dev = NULL;
    client = dev_get_drvdata(dev);
    printk("%s DEBUG\n", __func__);
    if(client == NULL)
    {
        return 0;
    }
    printk("%s DEBUG1\n", __func__);
    lcd_dev = i2c_get_clientdata(client);
    printk("%s DEBUG1\n", __func__);
    if(lcd_dev != NULL)
    {
        return sprintf(buf, "row:column -- %d:%d\n", lcd_dev->max_rows, lcd_dev->max_columns); 
    }
    return sprintf(buf, "ERROR");
}

static ssize_t lcd_set_row_store(struct device *dev, 
                                struct device_attribute *attr, 
                                const char *buf, size_t count)
{
    int row = simple_strtoul(buf, NULL, 10);
    struct i2c_client *client = dev_get_drvdata(dev);
    struct lcd_i2c_device *lcd_dev = NULL;

    printk("%s DEBUG\n", __func__);
    if(client == NULL)
    {
        return 0;
    }
    lcd_dev = i2c_get_clientdata(client);
    printk("%s DEBUG1\n", __func__);
    if(lcd_dev == NULL)
    {
        return 0; 
    }
    lcd_dev->cur_rows = row;
    return count;
}


static ssize_t lcd_set_collum_store(struct device *dev, 
                                struct device_attribute *attr, 
                                const char *buf, size_t count)
{
    int collum = simple_strtoul(buf, NULL, 10);
    struct i2c_client *client = dev_get_drvdata(dev);
    struct lcd_i2c_device *lcd_dev = NULL;

    printk("%s DEBUG\n", __func__);
    if(client == NULL)
    {
        return 0;
    }
    lcd_dev = i2c_get_clientdata(client);
    printk("%s DEBUG1\n", __func__);
    if(lcd_dev == NULL)
    {
        return 0; 
    }
    lcd_dev->cur_col = collum;
    return count;
}

static ssize_t lcd_clear_display_store(struct device *dev, 
                                struct device_attribute *attr, 
                                const char *buf, size_t count)
{
    int cmd = simple_strtoul(buf, NULL, 10);
    struct i2c_client *client = dev_get_drvdata(dev);
    struct lcd_i2c_device *lcd_dev = NULL;

    printk("%s DEBUG\n", __func__);
    if(client == NULL)
    {
        return 0;
    }
    lcd_dev = i2c_get_clientdata(client);
    printk("%s DEBUG1\n", __func__);
    if(lcd_dev == NULL)
    {
        return 0; 
    }
    if(cmd != 0)
    {
        lcd_clear_display(client);
    }
    return count;
}

static ssize_t lcd_display_string_store(struct device *dev, 
                                struct device_attribute *attr, 
                                const char *buf, size_t count)
{
    struct lcd_i2c_device *lcd_dev = NULL;
    struct i2c_client *client = dev_get_drvdata(dev);

    printk("%s DEBUG\n", __func__);
    if(client == NULL)
    {
        return 0;
    }
    lcd_dev = i2c_get_clientdata(client);
    printk("%s DEBUG1\n", __func__);
    if(lcd_dev == NULL)
    {
        return 0; 
    }

    if(!lcd_goto_XY(client, lcd_dev->cur_rows, lcd_dev->cur_col))
    {
        return 0;
    }
    lcd_send_string(client, (char *)buf);
    return count;
}

static DEVICE_ATTR(lcd_details, S_IRUSR, lcd_details_show, NULL);    
static DEVICE_ATTR(lcd_set_row, S_IWUSR, NULL, lcd_set_row_store);   
static DEVICE_ATTR(lcd_set_collum, S_IWUSR, NULL, lcd_set_collum_store);  
static DEVICE_ATTR(lcd_clear_display, S_IWUSR, NULL, lcd_clear_display_store);   
static DEVICE_ATTR(lcd_display_string, S_IWUSR, NULL, lcd_display_string_store);   


static int lcd_register_sysfs_device(struct lcd_i2c_device *lcd_dev)
{
    int err = -1;
    printk(KERN_INFO "%s: register lcd_i2c system kernel object\n", __func__);
    err = device_create_file(lcd_dev->dev, &dev_attr_lcd_details);
    if (err < 0) {
        printk(KERN_ERR DEVICE_NAME " cant create device attribute %s %s\n", 
               DEVICE_NAME, dev_attr_lcd_details.attr.name);
        goto out;
    }

    err = device_create_file(lcd_dev->dev, &dev_attr_lcd_set_row);
    if (err < 0) {
        printk(KERN_ERR DEVICE_NAME " cant create device attribute %s %s\n", 
               DEVICE_NAME, dev_attr_lcd_set_row.attr.name);
        goto out;
    }

    err = device_create_file(lcd_dev->dev, &dev_attr_lcd_set_collum);
    if (err < 0) {
        printk(KERN_ERR DEVICE_NAME " cant create device attribute %s %s\n", 
               DEVICE_NAME, dev_attr_lcd_set_collum.attr.name);
        goto out;
    }

    err = device_create_file(lcd_dev->dev, &dev_attr_lcd_clear_display);
    if (err < 0) {
        printk(KERN_ERR DEVICE_NAME " cant create device attribute %s %s\n", 
               DEVICE_NAME, dev_attr_lcd_clear_display.attr.name);
        goto out;
    }

    err = device_create_file(lcd_dev->dev, &dev_attr_lcd_display_string);
    if (err < 0) {
        printk(KERN_ERR DEVICE_NAME " cant create device attribute %s %s\n", 
               DEVICE_NAME, dev_attr_lcd_display_string.attr.name);
        goto out;
    }

    return 0;
out:
    printk(KERN_ERR "%s: failed to create sysfs\n", __func__);
    return err;
}


static int lcd_register_device(struct lcd_i2c_device *lcd_dev)
{
    int rval;

/* Alloc a device region */
    rval = alloc_chrdev_region(&lcd_dev->lcd_dev_t, 1, 1, DEVICE_NAME);
    if (rval != 0)          /* error */
        goto cdev_alloc_err;

/* Registring */
    lcd_dev->lcd_cdev = cdev_alloc();
    if (!lcd_dev->lcd_cdev) 
        goto cdev_alloc_err;

/* Init it! */
    cdev_init(lcd_dev->lcd_cdev, &lcd_fops); 

/* Tell the kernel "hey, I'm exist" */
    rval = cdev_add(lcd_dev->lcd_cdev, lcd_dev->lcd_dev_t, 1);
    if (rval < 0) 
            goto cdev_add_out;

/* class */
    lcd_dev->lcd_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(lcd_dev->lcd_class)) {
            printk(KERN_ERR DEVICE_NAME " cant create class %s\n", CLASS_NAME);
            goto class_err;
    }

/* device */
    lcd_dev->dev = device_create(lcd_dev->lcd_class, NULL, lcd_dev->lcd_dev_t, NULL, DEVICE_NAME);
    if (IS_ERR(lcd_dev->dev)) {
            printk(KERN_ERR DEVICE_NAME " cant create device %s\n", DEVICE_NAME);
            goto device_err;
    }
            
    return 0;


device_err:
        device_destroy(lcd_dev->lcd_class, lcd_dev->lcd_dev_t);
class_err:
        class_unregister(lcd_dev->lcd_class);
        class_destroy(lcd_dev->lcd_class);
cdev_add_out:
        cdev_del(lcd_dev->lcd_cdev);
cdev_alloc_err:
        return -EFAULT;
}

static void lcd_unregister_device(struct lcd_i2c_device *lcd_dev)
{
    device_destroy(lcd_dev->lcd_class, lcd_dev->lcd_dev_t);
    class_unregister(lcd_dev->lcd_class);
    class_destroy(lcd_dev->lcd_class);
    cdev_del(lcd_dev->lcd_cdev);
}

static const struct of_device_id of_lcd_i2c_id[] =
{
	{.compatible = "pta, lcd_i2c", 0},
	{},
};

MODULE_DEVICE_TABLE(of, of_lcd_i2c_id);



static int  lcd_i2c_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result;
	struct lcd_i2c_device *lcd_dev;
	struct i2c_adapter *adapter;

	printk(KERN_INFO "Start to load and initialize lcd_i2c sensor\n");

	adapter = to_i2c_adapter(client->dev.parent);
	result = i2c_check_functionality(adapter,
					 I2C_FUNC_SMBUS_BYTE |
					 I2C_FUNC_SMBUS_BYTE_DATA);
	if (!result)
		goto err_out;

	printk(KERN_INFO "I2C bus is used for lcd_i2c: %d\n", adapter->nr);


    lcd_dev = kzalloc(sizeof(struct lcd_i2c_device), GFP_KERNEL);
	if(!lcd_dev)
	{
		result = -ENOMEM;
		dev_err(&client->dev, "alloc data memory error!\n");
		goto err_out;
    }

    printk(KERN_INFO "I2C Address of lcd_i2c: %d\n", client->addr);
	lcd_dev->client = client;

    if(0 != lcd_register_device(lcd_dev))
    {
        printk(KERN_ERR " LCD I2C failed to register device file\n");
        result = -EINVAL;
        goto err_out;
    }

    if(0 != lcd_register_sysfs_device(lcd_dev))
    {
        printk(KERN_ERR " LCD I2C failed to register sysfs\n");
        result = -EINVAL;
        goto err_out;
    }


    i2c_set_clientdata(client, lcd_dev);
    dev_set_drvdata(lcd_dev->dev, client);
	printk(KERN_INFO "Set Client data with lcd_i2c_device\n");
	if (!lcd_init(client)) 
	{
		printk(KERN_ERR " LCD I2C device init failed\n");
		result = -EINVAL;
		goto err_out;
	}
	printk("Success to initialize device lcd_i2c\n");

	printk("%s success for loading platform device i2c\n",__FUNCTION__);
	return 0;

err_out:
	printk(KERN_ERR "failed to probe lcd i2c\n");
	return result;
}

static int lcd_i2c_remove(struct i2c_client *client)
{
	struct lcd_i2c_device *lcd_dev = i2c_get_clientdata(client);
	if(!lcd_dev)
		return 0;
    lcd_unregister_device(lcd_dev);
    kfree(lcd_dev);	
    printk("Success to remove lcd_i2c\n");	
	return 0;
}

static const struct i2c_device_id lcd_i2c_id[] = {
	{"lcd_i2c", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, lcd_i2c_id);

static struct i2c_driver lcd_i2c_driver = {
	.driver = {
		   .name = "lcd_i2c",
		   .owner = THIS_MODULE,
		   .of_match_table		= of_match_ptr(of_lcd_i2c_id),
		   },
	.probe = lcd_i2c_probe,
	.remove = lcd_i2c_remove,
	.id_table = lcd_i2c_id,
};

static int __init lcd_i2c_init(void)
{
	/* register driver */
	int res;
	res = i2c_add_driver(&lcd_i2c_driver);
	if (res < 0) {
		printk(KERN_ERR "add lcd_i2c i2c driver failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO "insmod lcd_i2c i2c driver\n");
	return res;
}

static void __exit lcd_i2c_exit(void)
{
	i2c_del_driver(&lcd_i2c_driver);
	printk(KERN_INFO "rmmod lcd_i2c i2c driver\n");
}

module_init(lcd_i2c_init);
module_exit(lcd_i2c_exit);

MODULE_AUTHOR("Pham Tuan Anh");
MODULE_DESCRIPTION("LCD I2C Device Driver");
MODULE_LICENSE("GPL");