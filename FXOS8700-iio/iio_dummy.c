#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h> 
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>

#define FAKE_VOLTAGE_CHANNEL(num) \
{\
	.type = IIO_VOLTAGE,\
	.indexed = 1,\
	.channel = (num),\
	.address = (num),\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) \
}

struct fxos8700_data{
	struct i2c_client * client;
	struct iio_dev *indio_dev;
	struct miscdevice * acc_miscdev;
	struct miscdevice * mag_miscdev;
	struct input_dev * acc_idev;
	struct input_dev * mag_idev; 
	
	int irq_in;
	
	atomic_t acc_delay;
	atomic_t mag_delay;
	atomic_t acc_active;
	atomic_t mag_active;
	atomic_t position;
};

struct my_private_data {
	int foo;
	int bar;
	struct mutex lock;
};

static int fake_read_raw(struct iio_dev *indio_dev,\
		struct iio_chan_spec const *channel, int *val,\
		int *val2, long mask)
{	
	switch(mask)
	{
		case IIO_CHAN_INFO_RAW:
			*val = 10;
			*val2 = 11;
			printk(KERN_INFO "fake_read_raw IIO_CHAN_INFO_RAW val:val2 %d:%d\n", *val, *val2);
			break;

		case IIO_CHAN_INFO_SCALE:
			*val = 20;
			*val2 = 21;
			printk(KERN_INFO "fake_read_raw IIO_CHAN_INFO_SCALE val:val2 %d:%d\n", *val, *val2);
			break;	
	}
	return 0;
}
static int fake_write_raw(struct iio_dev *indio_dev,\
		struct iio_chan_spec const *chan,\
		int val, int val2, long mask)
{
	// switch(mask)
	// {
	// 	case IIO_CHAN_INFO_RAW:
	// 		printk(KERN_INFO " fake_write_raw IIO_CHAN_INFO_RAW \n");
	// 		printk(KERN_INFO " fake_write_raw val %d\n", val);
	// 		printk(KERN_INFO "fake_write_raw val2 %d\n", val2);
	// 		break;

	// 	case IIO_CHAN_INFO_SCALE:
	// 		printk(KERN_INFO " fake_write_raw IIO_CHAN_INFO_SCALE \n");
	// 		printk(KERN_INFO " fake_write_raw val %d\n", val);
	// 		printk(KERN_INFO "fake_write_raw val2 %d\n", val2);
	// 		break;	
	// }

	return 0;
}
static const struct iio_chan_spec fake_channels[] = {
	FAKE_VOLTAGE_CHANNEL(0),
	FAKE_VOLTAGE_CHANNEL(1),
	FAKE_VOLTAGE_CHANNEL(2),
	FAKE_VOLTAGE_CHANNEL(3),
};

static IIO_CONST_ATTR(in_accel_sampling_frequency_available,
		      "1.5625 6.25 12.5 50 100 200 400 800");
static IIO_CONST_ATTR(in_magn_sampling_frequency_available,
		      "1.5625 6.25 12.5 50 100 200 400 800");
static IIO_CONST_ATTR(in_accel_scale_available, "0.000244 0.000488 0.000976");
static IIO_CONST_ATTR(in_magn_scale_available, "0.000001200");

static struct attribute *fxos8700_attrs[] = {
	&iio_const_attr_in_accel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_magn_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_accel_scale_available.dev_attr.attr,
	&iio_const_attr_in_magn_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group fxos8700_attrs_group = {
	.attrs = fxos8700_attrs,
};

static const struct iio_info fake_iio_info = {
	.read_raw = fake_read_raw,
	.write_raw = fake_write_raw,
	.attrs = &fxos8700_attrs_group,
	.driver_module = THIS_MODULE,
};

static const struct of_device_id of_fxos8700_id[] =
{
	{.compatible = "fsl,fxos8700-test", 0},
	{},
};

MODULE_DEVICE_TABLE(of, of_fxos8700_id);


static int  iio_dummy_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result, client_id;
	struct fxos8700_data *pdata;
	struct i2c_adapter *adapter;
	const struct of_device_id *match;
	struct iio_dev *indio_dev;

	match = of_match_device(of_fxos8700_id, &client->dev);
	if(!match)
	{
		printk(KERN_ERR "Cannot detect match device fxos8700 sensor in device tree\n");
		return -ENODEV;
	}
	else
	{
		printk(KERN_INFO "Start to load and initialize fxos8700 sensor\n");
	}

	adapter = to_i2c_adapter(client->dev.parent);
	result = i2c_check_functionality(adapter,
					 I2C_FUNC_SMBUS_BYTE |
					 I2C_FUNC_SMBUS_BYTE_DATA);
	if (!result)
		goto err_out;

	printk(KERN_INFO "I2C Address of FXOS8700: %d\n", client->addr);
	printk(KERN_INFO "I2C bus is used for FXOS8700: %d\n", adapter->nr);


    pdata = kzalloc(sizeof(struct fxos8700_data), GFP_KERNEL);
	if(!pdata){
		result = -ENOMEM;
		dev_err(&client->dev, "alloc data memory error!\n");
		goto err_out;
    }


    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*pdata));
    if(!indio_dev)
    {
    	dev_err(&client->dev, "fail to allocate data memory for iio device\n");
    	return -ENODEV;
    }

    pdata = iio_priv(indio_dev);



	pdata->irq_in = of_get_named_gpio(client->dev.of_node, "irq_property", 0);
	if(!gpio_is_valid(pdata->irq_in))
	{
		dev_err(&client->dev, "Cannot get interrupt pin\n");
		return -ENODEV;
	}

	atomic_set(&pdata->acc_delay, FXOS8700_DELAY_DEFAULT);
	atomic_set(&pdata->mag_delay, FXOS8700_DELAY_DEFAULT);
	

	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = fake_channels;
	indio_dev->num_channels = ARRAY_SIZE(fake_channels);
	indio_dev->name = client->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &fake_iio_info;
	
	// printk(KERN_INFO "IRQ GPIO FOR FXOS8700: %d\n", pdata->irq_in);
	// if(pdata->irq_in){
	// 	result= request_threaded_irq(gpio_to_irq(pdata->irq_in),NULL, fxos8700_irq_handler,
	// 			  IRQF_TRIGGER_LOW | IRQF_ONESHOT, client->dev.driver->name, pdata);
	// 	if (result < 0) {
	// 		dev_err(&client->dev, "failed to register fxos8700 irq %d!\n",
	// 			client->irq);
	// 		goto err_register_irq;
	// 	}
	// }

	pdata->client = client;
	pdata->indio_dev = indio_dev;

	i2c_set_clientdata(client,pdata);

	result = fxos8700_register_input_device(pdata);
	if (result) {
		dev_err(&client->dev, "create device file failed!\n");
		result = -EINVAL;
		goto err_register_input_device;
	}
	
	result = fxos8700_device_init(client);
	if (result) {
		dev_err(&client->dev, "init device failed!\n");
		result = -EINVAL;
		goto err_init_device;
	}

	printk("%s success for loading platform device i2c\n",__FUNCTION__);

	return devm_iio_device_register(&client->dev, indio_dev);

err_init_device:
	fxos8700_unregister_input_device(pdata);
err_register_input_device:
	// fxos8700_unregister_sysfs_device(pdata);
err_register_sys:

err_regsiter_mag_misc:

err_regsiter_acc_misc:
	i2c_set_clientdata(client,NULL);
	kfree(pdata);
err_out:
	return result;
}

static int iio_dummy_remove(struct i2c_client *client)
{
	struct fxos8700_data *pdata =  i2c_get_clientdata(client);
	struct iio_dev *indio_dev;
	if(!pdata)
		return 0;

	indio_dev = pdata->indio_dev;
	if(!indio_dev)
	{
		return 0;
	}

    fxos8700_device_stop(client);
	fxos8700_unregister_sysfs_device(pdata);
	devm_iio_device_unregister(&client->dev, indio_dev);

    kfree(pdata);		
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int iio_dummy_suspend(struct device *dev)
{
	// struct i2c_client *client = to_i2c_client(dev);
	// struct fxos8700_data *pdata =  i2c_get_clientdata(client);
	// // if(atomic_read(&pdata->acc_active)|| atomic_read(&pdata->mag_active))
	// 	fxos8700_device_stop(client);
	return 0;
}

static int iio_dummy_resume(struct device *dev)
{
    int ret = 0;
	// struct i2c_client *client = to_i2c_client(dev);
	// struct fxos8700_data *pdata =  i2c_get_clientdata(client);
	// if(atomic_read(&pdata->acc_active))
	// 	fxos8700_change_mode(client,FXOS8700_TYPE_ACC,FXOS8700_ACTIVED);
	// if(atomic_read(&pdata->mag_active))
	// 	fxos8700_change_mode(client,FXOS8700_TYPE_MAG,FXOS8700_ACTIVED);
	return ret;
	  
}
#endif

static const struct i2c_device_id fxos8700_id[] = {
	{"fxos8700", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, fxos8700_id);

static SIMPLE_DEV_PM_OPS(fxos8700_pm_ops, iio_dummy_suspend, iio_dummy_resume);
static struct i2c_driver iio_dummy_driver = {
	.driver = {
		   .name = "fxos8700",
		   .owner = THIS_MODULE,
		   .pm = &fxos8700_pm_ops,
		   .of_match_table		= of_match_ptr(of_fxos8700_id),
		   },
	.probe = iio_dummy_probe,
	.remove = iio_dummy_remove,
	.id_table = fxos8700_id,
};

static int __init iio_dummy_init(void)
{
	/* register driver */
	int res;
	res = i2c_add_driver(&iio_dummy_driver);
	if (res < 0) {
		printk(KERN_ERR "add fxos8700 i2c driver failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO "insmod fxos8700 i2c driver\n");
	return res;
}

static void __exit iio_dummy_exit(void)
{
	i2c_del_driver(&iio_dummy_driver);
	printk(KERN_INFO "rmmod fxos8700 i2c driver\n");
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("FXOS8700 6-Axis Acc and Mag Combo Sensor driver");
MODULE_LICENSE("GPL");

module_init(iio_dummy_init);
module_exit(iio_dummy_exit);