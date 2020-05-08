/*
 *  fxos8700.c - Linux kernel modules for FXOS8700 6-Axis Acc and Mag 
 *  Combo Sensor 
 *
 *  Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
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
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include "fxos8700.h"

#define FXOS8700_DELAY_DEFAULT		200 	/* msecs */
#define FXOS8700_POSITION_DEFAULT	1 	/* msecs */

#define FXOS8700_TYPE_ACC 	0x00
#define FXOS8700_TYPE_MAG 	0x01

#define FXOS8700_STANDBY 	0x00
#define FXOS8700_ACTIVED 	0x01

#define ABS_STATUS 			ABS_WHEEL

#define FXOS8700_DISABLE_IRQ		false
#define FXOS8700_ENABLE_IRQ			true

#define FXOS8700_DISABLE_FIFO				0x00
#define FXOS8700_SET_FIFO_TRIGGER			0x03
#define FXOS8700_SET_FIFO_COUTN_WMARK		0x1A

#define FXOS8700_INT_SOURCE_DATA_READY      0x01
/* Bit definitions for FXOS8700_CTRL_REG1 */
#define FXOS8700_CTRL_ODR_MSK       0x38
#define FXOS8700_CTRL_ODR_MAX       0x00
#define FXOS8700_CTRL_ODR_MIN       GENMASK(4, 3)

enum fxos8700_accel_scale_bits {
	MODE_2G = 0,
	MODE_4G,
	MODE_8G,
};

enum fxos8700_axis_enum
{
	AXIS_X = 0,
	AXIS_Y,
	AXIS_Z,
};

struct fxos8700_scale {
	u8 bits;
	int uscale;
};

struct fxos8700_odr {
	u8 bits;
	int odr;
	int uodr;
};

static const struct fxos8700_scale fxos8700_accel_scale[] = {
	{ MODE_2G, 244},
	{ MODE_4G, 488},
	{ MODE_8G, 976},
};

enum fxos8700_scan_axis {
	FXOS8700_SCAN_ACCEL_X = 0,
	FXOS8700_SCAN_ACCEL_Y,
	FXOS8700_SCAN_ACCEL_Z,
	FXOS8700_SCAN_MAGN_X,
	FXOS8700_SCAN_MAGN_Y,
	FXOS8700_SCAN_MAGN_Z,
	FXOS8700_SCAN_RHALL,
	FXOS8700_SCAN_TIMESTAMP,
};

#define IIO_FXOS8700_CHANNEL(_axis, _type) \
{ \
	.type = _type, \
	.modified = 1, \
	.channel2 = IIO_MOD_##_axis, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) \
	| BIT(IIO_CHAN_INFO_SCALE), \
	.scan_index = FXOS8700_SCAN_ACCEL_##_axis,					\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.shift = 0,							\
		.endianness = IIO_BE,				\
	},										\
}

struct iio_chan_spec fxos8700_chan_spec[] = 
{
	IIO_FXOS8700_CHANNEL(X, IIO_ACCEL),
	IIO_FXOS8700_CHANNEL(Y, IIO_ACCEL),
	IIO_FXOS8700_CHANNEL(Z, IIO_ACCEL),
	IIO_FXOS8700_CHANNEL(X, IIO_MAGN),
	IIO_FXOS8700_CHANNEL(Y, IIO_MAGN),
	IIO_FXOS8700_CHANNEL(Z, IIO_MAGN),
	IIO_CHAN_SOFT_TIMESTAMP(FXOS8700_SCAN_TIMESTAMP)
};


struct fxos8700_data_axis{
    short x;
	short y;
	short z;
};

struct fxos8700_data{
	struct i2c_client * client;
	struct iio_trigger *trig;
	struct miscdevice * acc_miscdev;
	struct miscdevice * mag_miscdev;
	struct input_dev * acc_idev;
	struct input_dev * mag_idev;
	struct mutex mutex; 
	
	int irq_in;
	
	atomic_t irq_set;
	atomic_t acc_delay;
	atomic_t mag_delay;
	atomic_t acc_active;
	atomic_t mag_active;
	atomic_t position;
};

static struct fxos8700_data * g_fxos8700_data = NULL;
static int fxos8700_position_settings[8][3][3] =
{
   {{ 0, -1,  0}, { 1,  0,	0}, {0, 0,	1}},
   {{-1,  0,  0}, { 0, -1,	0}, {0, 0,	1}},
   {{ 0,  1,  0}, {-1,  0,	0}, {0, 0,	1}},
   {{ 1,  0,  0}, { 0,  1,	0}, {0, 0,	1}},
   
   {{ 0, -1,  0}, {-1,  0,	0}, {0, 0,  -1}},
   {{-1,  0,  0}, { 0,  1,	0}, {0, 0,  -1}},
   {{ 0,  1,  0}, { 1,  0,	0}, {0, 0,  -1}},
   {{ 1,  0,  0}, { 0, -1,	0}, {0, 0,  -1}},
};

static const struct fxos8700_odr fxos8700_odr[] = {
	{0x00, 800, 0},
	{0x01, 400, 0},
	{0x02, 200, 0},
	{0x03, 100, 0},
	{0x04, 50, 0},
	{0x05, 12, 500000},
	{0x06, 6, 250000},
	{0x07, 1, 562500},
};

// static int iio_trigger_validate_own_device(struct iio_trigger *trig,
// 	struct iio_dev *indio_dev)
// {
// 	if (indio_dev->dev.parent != trig->dev.parent)
// 		return -EINVAL;
// 	return 0;
// }

static int fxos8700_scan_mode(struct iio_chan_spec const *fxos8700_chan)
{
	switch(fxos8700_chan->type)
	{
		case IIO_ACCEL:
			return FXOS8700_TYPE_ACC;

		case IIO_MAGN:
			return FXOS8700_TYPE_MAG;

		default:
			break;	
	}

	return -EINVAL;	
}

static int fxos8700_data_convert(struct fxos8700_data_axis *axis_data,int position)
{
   short rawdata[3],data[3];
   int i,j;

   if(axis_data == NULL)
   	return -1;

   if(position < 0 || position > 7 )
   		position = 0;
   rawdata [0] = axis_data->x;
   rawdata [1] = axis_data->y; 
   rawdata [2] = axis_data->z;  
   for(i = 0; i < 3 ; i++)
   {
   	data[i] = 0;
   	for(j = 0; j < 3; j++)
		data[i] += rawdata[j] * fxos8700_position_settings[position][i][j];
   }
   axis_data->x = data[0];
   axis_data->y = data[1];
   axis_data->z = data[2];
   return 0;
}

static int fxos8700_change_mode(struct i2c_client *client, int type,int active)
{
	u8 data;
	int acc_act,mag_act;
	int ret;

	acc_act = FXOS8700_STANDBY;
	mag_act = FXOS8700_STANDBY;
	data = i2c_smbus_read_byte_data(client, FXOS8700_CTRL_REG1);
	if(type == FXOS8700_TYPE_ACC)
		acc_act = active;
	else
		mag_act = active;
	if(acc_act ==  FXOS8700_ACTIVED || mag_act == FXOS8700_ACTIVED)     
		data |= 0x01;
	else
		data &= ~0x01;
	ret = i2c_smbus_write_byte_data(client, FXOS8700_CTRL_REG1,data); 
	return ret;
	
}

static int fxos8700_motion_detect_cfg(struct i2c_client *client,u8 threshold,u8 debounce_count)
{
	int result;
	u8 val,ctrl_reg1;
	ctrl_reg1 = i2c_smbus_read_byte_data(client, FXOS8700_CTRL_REG1);
	result = i2c_smbus_write_byte_data(client, FXOS8700_CTRL_REG1, (ctrl_reg1 & ~0x01));// force to standby
	if (result < 0)
		goto out;
	result = i2c_smbus_write_byte_data(client, FXOS8700_FFMT_CFG, 0xf8);
	if(result < 0)
		goto out;
		
	result = i2c_smbus_write_byte_data(client, FXOS8700_FFMT_THS, threshold);
		if(result < 0)
			goto out;
	result = i2c_smbus_write_byte_data(client, FXOS8700_FFMT_COUNT, debounce_count);
		if(result < 0)
			goto out;
	val = i2c_smbus_read_byte_data(client, FXOS8700_CTRL_REG4);
	result = i2c_smbus_write_byte_data(client, FXOS8700_CTRL_REG4, val | (0x01 << 2));//motion detect route to pin1
	if(result < 0)
		goto out;
	i2c_smbus_write_byte_data(client, FXOS8700_CTRL_REG1, ctrl_reg1); //restore sensor standby/active mode
	return 0;
out:
	dev_err(&client->dev, "error when set fxos8700 motion detect configure:(%d)", result);
	return result;
}

static int fxos8700_set_odr(struct i2c_client * client,int type, int delay)
{
	struct fxos8700_data *pdata =  i2c_get_clientdata(client);
	int tmp_delay = 0,mag_delay,acc_delay;
	u8 val;
	acc_delay = atomic_read(&pdata->acc_delay);
	mag_delay = atomic_read(&pdata->mag_delay);
	if(delay <= 0)
		return -EFAULT;
	
	if(type == FXOS8700_TYPE_ACC)
		tmp_delay = delay < mag_delay ? delay : mag_delay;
	else if(type == FXOS8700_TYPE_MAG)
		tmp_delay = delay < acc_delay ? delay : acc_delay;
	if(tmp_delay == 0)
		tmp_delay = 100; //default
	val = i2c_smbus_read_byte_data(client, FXOS8700_CTRL_REG1);
	i2c_smbus_write_byte_data(client, FXOS8700_CTRL_REG1, (val & ~0x01)); //set sensor standby
	val &= ~(0x7 << 3);
	if(tmp_delay <= 5)
		val |= 0x01 << 3;
	else if(tmp_delay <= 10)
		val |= 0x02 << 3;
	else if(tmp_delay <= 20)
		val |= 0x03 << 3;
    else if(tmp_delay <= 80)
		val |= 0x04 << 3;
	else if(tmp_delay <= 160)
		val |= 0x05 << 3;
	else if(tmp_delay <= 320)
		val |= 0x06 << 3;
	else
		val |= 0x03 << 3; 
	i2c_smbus_write_byte_data(client, FXOS8700_CTRL_REG1, val); //set sensor standby
	
	return 0;
}

static int fxos8700_device_init(struct fxos8700_data *pdata)
{
	int result;
	struct i2c_client *client = pdata->client;
	
	
	result = i2c_smbus_write_byte_data(client, FXOS8700_CTRL_REG1, 0x00); //standby mode
	if (result < 0)
		goto out;
	result = i2c_smbus_write_byte_data(client, FXOS8700_M_CTRL_REG1, 0x1F); //
	if (result < 0)
		goto out;
	result = i2c_smbus_write_byte_data(client, FXOS8700_M_CTRL_REG2,0x5c); //hybrid mode
	if (result < 0)
		goto out;
	result = i2c_smbus_write_byte_data(client, FXOS8700_CTRL_REG1, 0x03 << 3); //odr 50hz
	if (result < 0)
		goto out;

	atomic_set(&pdata->acc_active,FXOS8700_STANDBY);
	atomic_set(&pdata->mag_active,FXOS8700_STANDBY);
    atomic_set(&pdata->position ,FXOS8700_POSITION_DEFAULT);
	return 0;
out:
	dev_err(&client->dev, "error when init fxos8700 device:(%d)", result);
	return result;
}

static int fxos8700_device_stop(struct i2c_client *client)
{
	i2c_smbus_write_byte_data(client, FXOS8700_CTRL_REG1, 0x00);
	return 0;
}

static int fxos8700_read_data(struct i2c_client *client, int axis, int type, int *val)
{
	u8 tmp_data[2];
	int ret;
	u8 reg;

	if(type == FXOS8700_TYPE_ACC)
		reg = FXOS8700_OUT_X_MSB;
	else
		reg = FXOS8700_M_OUT_X_MSB;

	reg = reg + (axis - IIO_MOD_X)*2;

	ret = i2c_smbus_read_i2c_block_data(client, reg, 2, tmp_data);
	if (ret < 2) {
		dev_err(&client->dev, "i2c block read %s failed\n", (type == FXOS8700_TYPE_ACC ? "acc" : "mag"));
		return -EIO;
	}

	*val = ((tmp_data[0] << 8) & 0xff00) | tmp_data[1];

	return 0;
}

static int fxos8700_acc_open(struct inode *inode, struct file *file)
{
	file->private_data = g_fxos8700_data;
	return nonseekable_open(inode, file);
}

static int fxos8700_acc_release(struct inode *inode, struct file *file)
{
	/* note: releasing the wdt in NOWAYOUT-mode does not stop it */
	return 0;
}



static int fxos8700_mag_open(struct inode *inode, struct file *file)
{
	file->private_data = g_fxos8700_data;
	return nonseekable_open(inode, file);
}

static int fxos8700_mag_release(struct inode *inode, struct file *file)
{
	/* note: releasing the wdt in NOWAYOUT-mode does not stop it */
	return 0;
}



/*acc and mag share sys interface*/
static ssize_t fxos8700_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct miscdevice *misc_dev = dev_get_drvdata(dev);
	struct fxos8700_data * pdata = g_fxos8700_data;
    int enable;
	enable = 0;
	if(pdata->acc_miscdev == misc_dev){
		enable = atomic_read(&pdata->acc_active);
	}
	if(pdata->mag_miscdev == misc_dev){
		enable = atomic_read(&pdata->mag_active);
	}
	return sprintf(buf, "%d\n", enable);
}


static ssize_t fxos8700_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct miscdevice *misc_dev = dev_get_drvdata(dev);
	struct fxos8700_data * pdata = g_fxos8700_data;
	struct i2c_client *client = pdata->client;
	int type;
	int enable;
	int ret;
	enable = simple_strtoul(buf, NULL, 10);   
	if(misc_dev == pdata->acc_miscdev)
		type = FXOS8700_TYPE_ACC;
	if(misc_dev == pdata->mag_miscdev)
		type = FXOS8700_TYPE_MAG;
	enable = (enable > 0 ? FXOS8700_ACTIVED : FXOS8700_STANDBY);
	ret = fxos8700_change_mode(client,type,enable);
	if(!ret){
		if(type == FXOS8700_TYPE_ACC)
			atomic_set(&pdata->acc_active, enable);
		else
			atomic_set(&pdata->mag_active, enable);
	}
	return count;
}

static ssize_t fxos8700_poll_delay_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct miscdevice *misc_dev = dev_get_drvdata(dev);
	struct fxos8700_data * pdata = g_fxos8700_data;
    int poll_delay = 0;
	if(pdata->acc_miscdev == misc_dev){
		poll_delay = atomic_read(&pdata->acc_delay);
	}
	if(pdata->mag_miscdev == misc_dev){
		poll_delay = atomic_read(&pdata->mag_delay);
	}
	return sprintf(buf, "%d\n", poll_delay);
}


static ssize_t fxos8700_poll_delay_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct miscdevice *misc_dev = dev_get_drvdata(dev);
	struct fxos8700_data * pdata = g_fxos8700_data;
	struct i2c_client *client = pdata->client;
	int type;
	int delay;
	int ret;
	delay = simple_strtoul(buf, NULL, 10);   
	if(misc_dev == pdata->acc_miscdev)
		type = FXOS8700_TYPE_ACC;
	if(misc_dev == pdata->mag_miscdev)
		type = FXOS8700_TYPE_MAG;
	ret = fxos8700_set_odr(client,type,delay);
	if(!ret){
		if(type == FXOS8700_TYPE_ACC)
			atomic_set(&pdata->acc_delay,delay);
		else
			atomic_set(&pdata->mag_delay,delay);
	}
	return count;
}

static ssize_t fxos8700_position_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct fxos8700_data * pdata = g_fxos8700_data;
    int position = 0;
    position = atomic_read(&pdata->position) ;
	return sprintf(buf, "%d\n", position);
}

static ssize_t fxos8700_position_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int  position;
	struct fxos8700_data * pdata = g_fxos8700_data;
	position = simple_strtoul(buf, NULL, 10);    
    atomic_set(&pdata->position, position);
	return count;
}

static ssize_t fxos8700_motion_detect_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct fxos8700_data *pdata = g_fxos8700_data;
	struct i2c_client *client = pdata->client;
	u8 threshold, dbounce;
	threshold = i2c_smbus_read_byte_data(client,FXOS8700_FFMT_THS);
	dbounce =  i2c_smbus_read_byte_data(client,FXOS8700_FFMT_COUNT);
	return sprintf(buf, "threshold %d,debounce %d\n", threshold,dbounce);
}

static ssize_t fxos8700_motion_detect_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct fxos8700_data *pdata = g_fxos8700_data;
	struct i2c_client *client = pdata->client;
	int threshold, dbounce;
	sscanf(buf,"%d,%d",&threshold,&dbounce);
	fxos8700_motion_detect_cfg(client,threshold,dbounce);
	return count;
}




static irqreturn_t fxos8700_irq_handler(int irq, void *dev)
{

	struct iio_poll_func *pf = dev;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct fxos8700_data *pdata = iio_priv(indio_dev);
	int data_buff[3];
	int idx;
	int type_read;
	int axis;
	int ret;
	// s64 time_ns = iio_get_time_ns();

	if(pf->irq != gpio_to_irq(pdata->irq_in))
	{
		goto err; 
	}
	
	data_buff[0] = 0x11;
	data_buff[2] = 0x33;
	data_buff[1] = 0x22;

	// for(idx = 0; idx < 3; idx++)
	// {
	// 	type_read = FXOS8700_TYPE_ACC;
	// 	axis      = fxos8700_chan_spec[idx].channel2;
	// 	mutex_lock(&pdata->mutex);
	// 	ret = fxos8700_read_data(pdata->client, axis, type_read, data_buff + idx);
	// 	if(ret)
	// 	{			
	// 		mutex_unlock(&pdata->mutex);
	// 		goto err;
	// 	}
	// 	mutex_unlock(&pdata->mutex);	
	// }

	iio_push_to_buffers_with_timestamp(indio_dev, data_buff, pf->timestamp);
err:
	iio_trigger_notify_done(indio_dev->trig);
	
	return IRQ_HANDLED;	
}

static int fxos8700_get_scale_raw(struct fxos8700_data *data, \
						int type, \
						int *uscale)
{
	int i, val;
	static const int scale_num = ARRAY_SIZE(fxos8700_accel_scale);

	if (type == FXOS8700_TYPE_MAG) {
		*uscale = 1200; /* Magnetometer is locked at 1200uT */
		return 0;
	}

	val = i2c_smbus_read_byte_data(data->client, FXOS8700_XYZ_DATA_CFG);

	for (i = 0; i < scale_num; i++) {
		if (fxos8700_accel_scale[i].bits == (val & 0x3)) {
			*uscale = fxos8700_accel_scale[i].uscale;
			return 0;
		}
	}

	return -EINVAL;
}

static int fxos8700_get_data_raw(struct fxos8700_data *pdata, struct iio_chan_spec const *fxos8700_chan, int *val)
{

	int axis;
	int type_read;
	int ret;

	type_read = fxos8700_scan_mode(fxos8700_chan);
	axis = fxos8700_chan->channel2;

	ret = fxos8700_read_data(pdata->client, axis, type_read, val);	
	if(ret)
		goto err;
		
	return 0;
err:
	return ret;	
}

static int fxos8700_get_odr_raw(struct fxos8700_data *data, int t,\
			    int *odr, int *uodr)
{
	int i, val;
	static const int odr_num = ARRAY_SIZE(fxos8700_odr);

	val = 0;
	val = i2c_smbus_read_byte_data(data->client, FXOS8700_CTRL_REG1);

	val &= FXOS8700_CTRL_ODR_MSK;

	for (i = 0; i < odr_num; i++)
		if (val == fxos8700_odr[i].bits)
			break;

	if (i >= odr_num)
		return -EINVAL;

	*odr = fxos8700_odr[i].odr;
	*uodr = fxos8700_odr[i].uodr;

	return 0;
}

static int fxos8700_read_raw(struct iio_dev *indio_dev, \
			     struct iio_chan_spec const *chan, \
			     int *val, int *val2, long mask)
{
	struct fxos8700_data *pdata = iio_priv(indio_dev);
	int ret;

	switch(mask)
	{
		case IIO_CHAN_INFO_RAW:
			mutex_lock(&pdata->mutex);
			ret = fxos8700_get_data_raw(pdata, chan, val);
			mutex_unlock(&pdata->mutex);
			if (ret)
				return ret;
			printk(KERN_INFO "IIO_CHAN_INFO_RAW %d\n", *val);
			return IIO_VAL_INT;
		case IIO_CHAN_INFO_SCALE:
			*val = 0;
			ret = fxos8700_get_scale_raw(pdata, fxos8700_scan_mode(chan),\
						 val2);
			printk(KERN_INFO "IIO_CHAN_INFO_SCALE %d\n", *val);
			return ret ? ret : IIO_VAL_INT_PLUS_MICRO;
		case IIO_CHAN_INFO_SAMP_FREQ:
			ret = fxos8700_get_odr_raw(pdata, fxos8700_scan_mode(chan),\
					       val, val2);
			printk(KERN_INFO "IIO_CHAN_INFO_SAMP_FREQ %d:%d\n", *val, *val2);
			return ret ? ret : IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
	}
	return 0;
}

static int fxos8700_write_raw(struct iio_dev *indio_dev, \
				struct iio_chan_spec const *chan, \
				int val, int val2, long mask)
{
	return 0;
}

static ssize_t fxos8700_set_mode_active(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct fxos8700_data *pdata = iio_priv(indio_dev);
	int active;
	unsigned int int_data;
	int ret;

	ret = kstrtouint(buf, 0, &int_data);
	if(ret)
	{
		dev_err(&pdata->client->dev, "Fail to get data from userspace %d!\n", ret);
		return ret;
	}

	active = (int)int_data;
	ret = fxos8700_change_mode(pdata->client, FXOS8700_TYPE_ACC, FXOS8700_ACTIVED);
	if(ret)
	{
		dev_err(&pdata->client->dev, "Enable acc device failed %d!\n", ret);
		return -EINVAL;
	}

	atomic_set(&pdata->acc_active, active);
	printk("Success to change mode device fxos8700\n");

	return len;
}

static ssize_t fxos8700_show_mode_active(struct device *dev,\
				struct device_attribute *attr,\
				char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct fxos8700_data *pdata = iio_priv(indio_dev);
	int mode_active;
	int ret;

	mode_active = atomic_read(&pdata->acc_active);

	switch(mode_active)
	{
		case 0:
			ret = sprintf(buf, "%s\n", "Stand By");
			break;

		case 1:
			ret = sprintf(buf, "%s\n", "Active");
			break;

		default:
			ret = sprintf(buf, "%s\n", "Invalid");
			break;	
	}
	return ret;
}

static int fxos8700_config_interrupt(struct i2c_client *client, bool state)
{
	int result;
	if(state == true)
	{
		result = i2c_smbus_write_byte_data(client, FXOS8700_CTRL_REG5, 0xff); //route to pin1
		if (result < 0)
			goto err;

		result = i2c_smbus_write_byte_data(client, FXOS8700_CTRL_REG4, 0x01); //data ready enable
		if (result < 0)
			goto err;
	}
	else
	{
		result = i2c_smbus_write_byte_data(client, FXOS8700_CTRL_REG4, 0x00);//data ready disable
		if (result < 0)
			goto err;
	}

	return 0;
err:
	dev_err(&client->dev, "error when disable/enable buffer trigger fxos8700 device:(%d)", result);
	return result;
}


static int fxos8700_data_rdy_trigger_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct fxos8700_data *data = iio_priv(indio_dev);
	
	int result;

	printk(KERN_INFO, "FXOS8700: Enable or disable trigger set state\n");
	result = fxos8700_change_mode(data->client, FXOS8700_TYPE_ACC, FXOS8700_STANDBY);
	if(result)
	{
		dev_err(&data->client->dev, "Enable acc device failed %d!\n", result);
		return -EINVAL;
	}
	result = fxos8700_config_interrupt(data->client, state);
	if(result < 0)
	{
		goto err;
	}
	

	result = fxos8700_change_mode(data->client, FXOS8700_TYPE_ACC, FXOS8700_ACTIVED);
	if(result)
	{
		dev_err(&data->client->dev, "Enable acc device failed %d!\n", result);
		return -EINVAL;
	}
	if(state == true)
	{
		atomic_set(&data->acc_active, FXOS8700_ACTIVED);
	}
	
	atomic_set(&data->irq_set, state);
	return 0;
err:
	dev_err(&data->client->dev, "Fail to set-up trigger");
	return result;
}

static int fxos8700_try_reenable(struct iio_trigger *trig)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct fxos8700_data *data = iio_priv(indio_dev);
	bool irq_set = atomic_read(&data->irq_set);
	int result;

	printk(KERN_INFO, "FXOS8700: Re-enable trigger set state\n");
	result = i2c_smbus_read_byte_data(data->client, FXOS8700_INT_SOURCE);
	if(result < 0)
	{
		goto err;
	}

	if(!((result & FXOS8700_INT_SOURCE_DATA_READY) & (int)irq_set))
	{
		result = fxos8700_config_interrupt(data->client, irq_set);
		if(result < 0)
		{
			goto out;
		}
	}

	return 0;
err:
	dev_err(&data->client->dev, "Fail to read INT_SOURCE register");
out:
	dev_err(&data->client->dev, "Fail to reset trigger");
	return result;
}

static int fxos8700_buffer_ops_preenable(struct iio_dev *indio_dev)
{
	struct fxos8700_data *pdata = iio_priv(indio_dev);
	int ret;

	/*change mode active for device*/
	printk(KERN_INFO, "FXOS8700: Pre-enable buffer\n");
	ret = fxos8700_change_mode(pdata->client, FXOS8700_TYPE_ACC, FXOS8700_ACTIVED);
	if(ret < 0)
	{
		return ret;
	}

	atomic_set(&pdata->acc_active, FXOS8700_ACTIVED);

	return 0;
}
 
 /*Config enable FIFO watermark*/
static int fxos8700_buffer_ops_postenable(struct iio_dev *indio_dev)
{
	struct fxos8700_data *pdata = iio_priv(indio_dev);
	int result;
	uint8_t write_config;
	int chip_status;

	printk(KERN_INFO, "FXOS8700: Post-enable buffer\n");
	result = iio_triggered_buffer_postenable(indio_dev);
	if(result < 0)
	{
		dev_err(&pdata->client->dev, "Fail to attach buffer to indio_dev\n");
		goto out;
	}

	/*set value for fifo trigger mode and count fifo warter mark is 26*/
	write_config = (uint8_t)((FXOS8700_SET_FIFO_TRIGGER << 5) | (FXOS8700_SET_FIFO_COUTN_WMARK));
	
	/*Refer datashet must set chip to stanby mode*/
	chip_status = atomic_read(&pdata->acc_active);
	if(chip_status = FXOS8700_ACTIVED)
	{
		 (void)fxos8700_change_mode(pdata->client, FXOS8700_TYPE_ACC, FXOS8700_STANDBY);
	}
	
	result = i2c_smbus_write_byte_data(pdata->client, FXOS8700_F_SETUP, write_config);
	if(result < 0)
	{
		result = fxos8700_change_mode(pdata->client, FXOS8700_TYPE_ACC, chip_status);
		dev_err(&pdata->client->dev, "Fail to write config FIFO\n");
		goto err;
	}

	(void)fxos8700_change_mode(pdata->client, FXOS8700_TYPE_ACC, chip_status);

	return 0;
err:
	iio_triggered_buffer_predisable(indio_dev);	
out:
	return result;	
}

 /*Config Disable FIFO watermark*/ 
static int fxos8700_buffer_ops_predisable(struct iio_dev *indio_dev)
{
	struct fxos8700_data *pdata = iio_priv(indio_dev);
	int result;
	uint8_t write_config;

	printk(KERN_INFO, "FXOS8700: Pre-Disable buffer\n");
	result = iio_triggered_buffer_predisable(indio_dev);
	if(result < 0)
	{
		dev_err(&pdata->client->dev, "Fail to attach buffer to indio_dev\n");
		goto out;
	}

	result = fxos8700_change_mode(pdata->client, FXOS8700_TYPE_ACC, FXOS8700_STANDBY);
	if(result < 0)
	{
		goto out;
	}

	result = i2c_smbus_write_byte_data(pdata->client, FXOS8700_F_SETUP, 0x00);
	if(result < 0)
	{
		dev_err(&pdata->client->dev, "Fail to write config FIFO\n");
		goto out;
	}


	return 0;
out:
	return result;	
}

 
static int fxos8700_buffer_ops_postdisable(struct iio_dev *indio_dev)
{
	struct fxos8700_data *pdata = iio_priv(indio_dev);
	int ret;

	printk(KERN_INFO, "FXOS8700: Post-Disable buffer\n");

	atomic_set(&pdata->acc_active, FXOS8700_STANDBY);
	return 0;
}

static IIO_DEVICE_ATTR(in_sensor_change_mode, S_IRUGO | S_IWUSR, \
						fxos8700_show_mode_active,	\
						fxos8700_set_mode_active, 0);
static IIO_CONST_ATTR(in_accel_sampling_frequency_available,
		      "1.5625 6.25 12.5 50 100 200 400 800");
static IIO_CONST_ATTR(in_magn_sampling_frequency_available,
		      "1.5625 6.25 12.5 50 100 200 400 800");
static IIO_CONST_ATTR(in_accel_scale_available, "0.000244 0.000488 0.000976");
static IIO_CONST_ATTR(in_magn_scale_available, "0.000001200");

static struct attribute *fxos8700_attrs[] = {
	&iio_dev_attr_in_sensor_change_mode.dev_attr.attr,
	&iio_const_attr_in_accel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_magn_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_accel_scale_available.dev_attr.attr,
	&iio_const_attr_in_magn_scale_available.dev_attr.attr,
	NULL,
};

static const struct iio_buffer_setup_ops fxos8700_buffer_ops =
{
	.preenable   = &fxos8700_buffer_ops_preenable,
	.postenable  = &fxos8700_buffer_ops_postenable,
	.predisable  = &fxos8700_buffer_ops_predisable,
	.postdisable = &fxos8700_buffer_ops_postdisable,
};

static const struct iio_trigger_ops fxos8700_trigger_ops = {
	.set_trigger_state  = &fxos8700_data_rdy_trigger_set_state,
	.try_reenable 		= &fxos8700_try_reenable,
};

static const struct attribute_group fxos8700_attrs_group = {
	.attrs = fxos8700_attrs,
};

static const struct iio_info fxos8700_info = {
	.read_raw = fxos8700_read_raw,
	.write_raw = fxos8700_write_raw,
	.attrs = &fxos8700_attrs_group,
};

static const struct of_device_id of_fxos8700_id[] =
{
	{.compatible = "fsl,fxos8700-test", 0},
	{},
};

MODULE_DEVICE_TABLE(of, of_fxos8700_id);


static int fxos8700_register_iio_trigger_buff(struct iio_dev *indio_dev, struct fxos8700_data *pdata)
{
	int ret;

	atomic_set(&pdata->irq_set, FXOS8700_DISABLE_IRQ);
	pdata->trig = iio_trigger_alloc("%s-dev-%d", indio_dev->name, indio_dev->id);
	if (!pdata->trig) {
		ret = -ENOMEM;
		goto err;
	}

	pdata->trig->dev.parent = indio_dev->dev.parent;
	pdata->trig->ops = &fxos8700_trigger_ops;
	iio_trigger_set_drvdata(pdata->trig, indio_dev);

	ret = iio_trigger_register(pdata->trig);
	if (ret < 0) {
		dev_err(&indio_dev->dev, "Failed to register iio trigger\n");
		goto err_trigger_free;
	}

	ret = devm_request_irq(indio_dev->dev.parent, gpio_to_irq(pdata->irq_in), \
		iio_trigger_generic_data_rdy_poll, IRQF_TRIGGER_RISING, \
		"fxos8700_event", pdata->trig);
	if (ret) {
		dev_err(indio_dev->dev.parent, "unable to request IRQ\n");
		goto err_trigger_unregister;
	}

	// ret = iio_triggered_buffer_setup(indio_dev,  &iio_pollfunc_store_time, &fxos8700_irq_handler, &fxos8700_buffer_ops);
	// if (ret < 0) {
	// 	dev_err(dev, "unable to setup iio triggered buffer\n");
	// 	goto err_trigger_unregister;
	// }

	return 0;

err_trigger_unregister:
	iio_trigger_unregister(pdata->trig);
err_trigger_free:
	iio_trigger_free(pdata->trig);
err:
	return ret;
}


static int  fxos8700_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int result, client_id;
	struct fxos8700_data *pdata;
	struct i2c_adapter *adapter;
	const struct of_device_id *match;
	struct iio_dev *indio_dev;

	match = of_match_device(of_fxos8700_id, dev);
	if(!match)
	{
		printk(KERN_ERR "Cannot detect match device fxos8700 sensor in device tree\n");
		return -ENODEV;
	}
	else
	{
		printk(KERN_INFO "Start to load and initialize fxos8700 sensor\n");
	}

	adapter = to_i2c_adapter(dev->parent);
	result = i2c_check_functionality(adapter,
					 I2C_FUNC_SMBUS_BYTE |
					 I2C_FUNC_SMBUS_BYTE_DATA);
	if (!result)
		goto err_out;

	printk(KERN_INFO "I2C Address of FXOS8700: %d\n", client->addr);
	printk(KERN_INFO "I2C bus is used for FXOS8700: %d\n", adapter->nr);

	client_id = i2c_smbus_read_byte_data(client, FXOS8700_WHO_AM_I);
	if (client_id !=  FXOS8700_DEVICE_ID && client_id != FXOS8700_PRE_DEVICE_ID) {
		dev_err(dev,
			"read chip ID 0x%x is not equal to 0x%x or 0x%x\n",
			result, FXOS8700_DEVICE_ID,FXOS8700_PRE_DEVICE_ID);
		result = -EINVAL;
		goto err_out;
	}
    pdata = kzalloc(sizeof(struct fxos8700_data), GFP_KERNEL);
	if(!pdata){
		result = -ENOMEM;
		dev_err(dev, "alloc data memory error!\n");
		goto err_out;
    }


    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*pdata));
    if(!indio_dev)
    {
    	dev_err(dev, "fail to allocate data memory for iio device\n");
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

	mutex_init(&pdata->mutex);
	

	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = fxos8700_chan_spec;
	indio_dev->num_channels = ARRAY_SIZE(fxos8700_chan_spec);
	indio_dev->name = client->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &fxos8700_info;
	
	result = iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time, \
		&fxos8700_irq_handler, &fxos8700_buffer_ops);
	if(result < 0)
		goto err_out;
	printk(KERN_INFO "IRQ GPIO FOR FXOS8700: %d\n", pdata->irq_in);
	if(pdata->irq_in)
	{
		result = fxos8700_register_iio_trigger_buff(indio_dev, pdata);
		if(result)
		{
			dev_err(&client->dev, "Fail to register iio trigger buffer\n");
			goto err_out;
		}
	}

	pdata->client = client;

	printk("Success to register input device fxos8700\n");
	
	result = fxos8700_device_init(pdata);
	if (result) {
		dev_err(&client->dev, "init device failed!\n");
		result = -EINVAL;
		goto err_buffer_cleanup;
	}

	printk("Success to initialize device fxos8700\n");

	printk("%s success for loading platform device i2c\n",__FUNCTION__);

	i2c_set_clientdata(client,indio_dev);
	result = devm_iio_device_register(&client->dev, indio_dev);
	if(result < 0)
	{
		dev_err(&client->dev, "unable to register iio device\n");
		goto err_out;
	}
	return 0; 
err_buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);
err_init_device:
	// fxos8700_unregister_input_device(pdata);
err_register_input_device:
	i2c_set_clientdata(client,NULL);
	kfree(pdata);

err_out:
	return result;
}

static int fxos8700_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct fxos8700_data *pdata = iio_priv(indio_dev);
	int result;

	if(!pdata)
		return 0;
	
	if(pdata->trig)
	{
		iio_triggered_buffer_cleanup(indio_dev);
		iio_trigger_unregister(pdata->trig);
		iio_trigger_free(pdata->trig);
	}


	printk("Success to unregister iio trigger buffer device fxos8700\n");

    fxos8700_device_stop(client);
	devm_iio_device_unregister(&client->dev, indio_dev);

    kfree(pdata);		
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int fxos8700_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fxos8700_data *pdata =  i2c_get_clientdata(client);
	if(atomic_read(&pdata->acc_active)|| atomic_read(&pdata->mag_active))
		fxos8700_device_stop(client);
	return 0;
}

static int fxos8700_resume(struct device *dev)
{
    int ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fxos8700_data *pdata =  i2c_get_clientdata(client);
	if(atomic_read(&pdata->acc_active))
		fxos8700_change_mode(client,FXOS8700_TYPE_ACC,FXOS8700_ACTIVED);
	if(atomic_read(&pdata->mag_active))
		fxos8700_change_mode(client,FXOS8700_TYPE_MAG,FXOS8700_ACTIVED);
	return ret;
	  
}
#endif

static const struct i2c_device_id fxos8700_id[] = {
	{"fxos8700", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, fxos8700_id);

static SIMPLE_DEV_PM_OPS(fxos8700_pm_ops, fxos8700_suspend, fxos8700_resume);
static struct i2c_driver fxos8700_driver = {
	.driver = {
		   .name = "fxos8700",
		   .owner = THIS_MODULE,
		   .pm = &fxos8700_pm_ops,
		   .of_match_table		= of_match_ptr(of_fxos8700_id),
		   },
	.probe = fxos8700_probe,
	.remove = fxos8700_remove,
	.id_table = fxos8700_id,
};

static int __init fxos8700_init(void)
{
	/* register driver */
	int res;
	res = i2c_add_driver(&fxos8700_driver);
	if (res < 0) {
		printk(KERN_ERR "add fxos8700 i2c driver failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO "insmod fxos8700 i2c driver\n");
	return res;
}

static void __exit fxos8700_exit(void)
{
	i2c_del_driver(&fxos8700_driver);
	printk(KERN_INFO "rmmod fxos8700 i2c driver\n");
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("FXOS8700 6-Axis Acc and Mag Combo Sensor driver");
MODULE_LICENSE("GPL");

module_init(fxos8700_init);
module_exit(fxos8700_exit);