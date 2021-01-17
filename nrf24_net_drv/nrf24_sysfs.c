// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2017 Marcin Ciupak <marcin.s.ciupak@gmail.com>
 *
 */

#include <linux/types.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/kfifo.h>
#include <linux/list.h>

#include "nrf24_net.h"
#include "nrf24_hal.h"
#include "nrf24_enums.h"

static ssize_t ack_pipe0_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;


	ret = nrf24_get_auto_ack(device->spi, 0);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t ack_pipe0_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u8 new;

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;
	if (new < 0 || new > 1)
		return -EINVAL;

	ret = nrf24_setup_auto_ack(device->spi, 0, new);
	if (ret < 0)
		return ret;
	device->pipe[0].cfg.ack = new;
	return count;
}

static ssize_t plw_pipe0_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;

	ret = nrf24_get_rx_pload_width(device->spi, 0);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t plw_pipe0_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u8 new;
	ssize_t old;

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new < 0 || new > PLOAD_MAX)
		return -EINVAL;
	old = nrf24_get_rx_pload_width(device->spi, 0);
	if (old < 0)
		return old;

	if ((u8)old != new) {
		ret = nrf24_set_rx_pload_width(device->spi, 0, new);
		if (ret < 0)
			return ret;
		device->pipe[0].cfg.plw = new;
	}

	return count;
}

static ssize_t address_pipe0_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	u8 addr[16];
	int ret;
	int count;
	int i;

	ret = nrf24_get_address(device->spi, 0, addr);
	if (ret < 0)
		return ret;

	count = scnprintf(buf, PAGE_SIZE, "0x");
	for (i = --ret; i >= 0; i--)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%02X", addr[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t address_pipe0_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf,
			     size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u64 address;
	int len;

	ret = kstrtoull(buf, 16, &address);
	if (ret < 0)
		return ret;

	len = nrf24_get_address_width(device->spi);
	if (len < 0)
		return len;

	if (address >= BIT_ULL(len * BITS_PER_BYTE))
		return -EINVAL;

	ret = nrf24_set_address(device->spi, 0, (u8 *)&address);
	if (ret < 0)
		return ret;
	
	device->pipe[0].cfg.address = address;

	return count;
}

static DEVICE_ATTR_RW(ack_pipe0);
static DEVICE_ATTR_RW(plw_pipe0);
static DEVICE_ATTR_RW(address_pipe0);

struct attribute *nrf24_pipe0_attrs[] = {
	&dev_attr_ack_pipe0.attr,
	&dev_attr_plw_pipe0.attr,
	&dev_attr_address_pipe0.attr,
	NULL,
};

static ssize_t ack_pipe1_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;


	ret = nrf24_get_auto_ack(device->spi, 1);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t ack_pipe1_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u8 new;

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;
	if (new < 0 || new > 1)
		return -EINVAL;

	ret = nrf24_setup_auto_ack(device->spi, 1, new);
	if (ret < 0)
		return ret;

    device->pipe[1].cfg.ack = new;

	return count;
}

static ssize_t plw_pipe1_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;

	ret = nrf24_get_rx_pload_width(device->spi, 1);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t plw_pipe1_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u8 new;
	ssize_t old;

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new < 0 || new > PLOAD_MAX)
		return -EINVAL;
	old = nrf24_get_rx_pload_width(device->spi, 1);
	if (old < 0)
		return old;

	if ((u8)old != new) {
		ret = nrf24_set_rx_pload_width(device->spi, 1, new);
		if (ret < 0)
			return ret;
        device->pipe[1].cfg.plw = new;
	}

	return count;
}

static ssize_t address_pipe1_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	u8 addr[16];
	int ret;
	int count;
	int i;

	ret = nrf24_get_address(device->spi, 1, addr);
	if (ret < 0)
		return ret;

	count = scnprintf(buf, PAGE_SIZE, "0x");
	for (i = --ret; i >= 0; i--)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%02X", addr[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t address_pipe1_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf,
			     size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u64 address;
	int len;

	ret = kstrtoull(buf, 16, &address);
	if (ret < 0)
		return ret;

	len = nrf24_get_address_width(device->spi);
	if (len < 0)
		return len;

	if (address >= BIT_ULL(len * BITS_PER_BYTE))
		return -EINVAL;

	ret = nrf24_set_address(device->spi, 1, (u8 *)&address);
	if (ret < 0)
		return ret;

    device->pipe[1].cfg.address = address;

	return count;
}

static DEVICE_ATTR_RW(ack_pipe1);
static DEVICE_ATTR_RW(plw_pipe1);
static DEVICE_ATTR_RW(address_pipe1);

struct attribute *nrf24_pipe1_attrs[] = {
	&dev_attr_ack_pipe1.attr,
	&dev_attr_plw_pipe1.attr,
	&dev_attr_address_pipe1.attr,
	NULL,
};

static ssize_t ack_pipe2_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;


	ret = nrf24_get_auto_ack(device->spi, 2);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t ack_pipe2_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u8 new;

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;
	if (new < 0 || new > 1)
		return -EINVAL;

	ret = nrf24_setup_auto_ack(device->spi, 2, new);
	if (ret < 0)
		return ret;
	
	device->pipe[1].cfg.ack = new;

	return count;
}

static ssize_t plw_pipe2_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;

	ret = nrf24_get_rx_pload_width(device->spi, 2);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t plw_pipe2_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u8 new;
	ssize_t old;

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new < 0 || new > PLOAD_MAX)
		return -EINVAL;
	old = nrf24_get_rx_pload_width(device->spi, 2);
	if (old < 0)
		return old;

	if ((u8)old != new) {
		ret = nrf24_set_rx_pload_width(device->spi, 2, new);
		if (ret < 0)
			return ret;
		device->pipe[2].cfg.plw = new;
	}

	return count;
}

static ssize_t address_pipe2_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	u8 addr[16];
	int ret;
	int count;
	int i;

	ret = nrf24_get_address(device->spi, 2, addr);
	if (ret < 0)
		return ret;

	count = scnprintf(buf, PAGE_SIZE, "0x");
	for (i = --ret; i >= 0; i--)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%02X", addr[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t address_pipe2_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf,
			     size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u64 address;
	int len;

	ret = kstrtoull(buf, 16, &address);
	if (ret < 0)
		return ret;

	len = nrf24_get_address_width(device->spi);
	if (len < 0)
		return len;

	if (address >= BIT_ULL(len * BITS_PER_BYTE))
		return -EINVAL;

	ret = nrf24_set_address(device->spi, 2, (u8 *)&address);
	if (ret < 0)
		return ret;
	device->pipe[2].cfg.address = address;
	return count;
}

static DEVICE_ATTR_RW(ack_pipe2);
static DEVICE_ATTR_RW(plw_pipe2);
static DEVICE_ATTR_RW(address_pipe2);

struct attribute *nrf24_pipe2_attrs[] = {
	&dev_attr_ack_pipe2.attr,
	&dev_attr_plw_pipe2.attr,
	&dev_attr_address_pipe2.attr,
	NULL,
};

static ssize_t ack_pipe3_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;


	ret = nrf24_get_auto_ack(device->spi, 3);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t ack_pipe3_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u8 new;

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;
	if (new < 0 || new > 1)
		return -EINVAL;

	ret = nrf24_setup_auto_ack(device->spi, 3, new);
	if (ret < 0)
		return ret;
    device->pipe[3].cfg.ack = new;
	return count;
}

static ssize_t plw_pipe3_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;

	ret = nrf24_get_rx_pload_width(device->spi, 3);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t plw_pipe3_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u8 new;
	ssize_t old;

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new < 0 || new > PLOAD_MAX)
		return -EINVAL;
	old = nrf24_get_rx_pload_width(device->spi, 3);
	if (old < 0)
		return old;

	if ((u8)old != new) {
		ret = nrf24_set_rx_pload_width(device->spi, 3, new);
		if (ret < 0)
			return ret;
		device->pipe[3].cfg.plw = new;
	}
	return count;
}

static ssize_t address_pipe3_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	u8 addr[16];
	int ret;
	int count;
	int i;

	ret = nrf24_get_address(device->spi, 3, addr);
	if (ret < 0)
		return ret;

	count = scnprintf(buf, PAGE_SIZE, "0x");
	for (i = --ret; i >= 0; i--)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%02X", addr[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t address_pipe3_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf,
			     size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u64 address;
	int len;

	ret = kstrtoull(buf, 16, &address);
	if (ret < 0)
		return ret;

	len = nrf24_get_address_width(device->spi);
	if (len < 0)
		return len;

	if (address >= BIT_ULL(len * BITS_PER_BYTE))
		return -EINVAL;

	ret = nrf24_set_address(device->spi, 3, (u8 *)&address);
	if (ret < 0)
		return ret;

    device->pipe[3].cfg.address = address;

	return count;
}

static DEVICE_ATTR_RW(ack_pipe3);
static DEVICE_ATTR_RW(plw_pipe3);
static DEVICE_ATTR_RW(address_pipe3);

struct attribute *nrf24_pipe3_attrs[] = {
	&dev_attr_ack_pipe3.attr,
	&dev_attr_plw_pipe3.attr,
	&dev_attr_address_pipe3.attr,
	NULL,
};

static ssize_t ack_pipe4_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;


	ret = nrf24_get_auto_ack(device->spi, 4);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t ack_pipe4_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u8 new;

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;
	if (new < 0 || new > 1)
		return -EINVAL;

	ret = nrf24_setup_auto_ack(device->spi, 4, new);
	if (ret < 0)
		return ret;

	device->pipe[4].cfg.ack = new;

	return count;
}

static ssize_t plw_pipe4_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;

	ret = nrf24_get_rx_pload_width(device->spi, 4);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t plw_pipe4_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u8 new;
	ssize_t old;

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new < 0 || new > PLOAD_MAX)
		return -EINVAL;
	old = nrf24_get_rx_pload_width(device->spi, 4);
	if (old < 0)
		return old;

	if ((u8)old != new) {
		ret = nrf24_set_rx_pload_width(device->spi, 4, new);
		if (ret < 0)
			return ret;
		device->pipe[4].cfg.plw = new;
	}

	return count;
}

static ssize_t address_pipe4_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	u8 addr[16];
	int ret;
	int count;
	int i;

	ret = nrf24_get_address(device->spi, 4, addr);
	if (ret < 0)
		return ret;

	count = scnprintf(buf, PAGE_SIZE, "0x");
	for (i = --ret; i >= 0; i--)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%02X", addr[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t address_pipe4_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf,
			     size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u64 address;
	int len;

	ret = kstrtoull(buf, 16, &address);
	if (ret < 0)
		return ret;

	len = nrf24_get_address_width(device->spi);
	if (len < 0)
		return len;

	if (address >= BIT_ULL(len * BITS_PER_BYTE))
		return -EINVAL;

	ret = nrf24_set_address(device->spi, 4, (u8 *)&address);
	if (ret < 0)
		return ret;
	
	device->pipe[4].cfg.address = address;
	
	return count;
}

static DEVICE_ATTR_RW(ack_pipe4);
static DEVICE_ATTR_RW(plw_pipe4);
static DEVICE_ATTR_RW(address_pipe4);

struct attribute *nrf24_pipe4_attrs[] = {
	&dev_attr_ack_pipe4.attr,
	&dev_attr_plw_pipe4.attr,
	&dev_attr_address_pipe4.attr,
	NULL,
};

static ssize_t ack_pipe5_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;


	ret = nrf24_get_auto_ack(device->spi, 5);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t ack_pipe5_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	uint8_t new;

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;
	if (new < 0 || new > 1)
		return -EINVAL;

	ret = nrf24_setup_auto_ack(device->spi, 5, new);
	if (ret < 0)
		return ret;
	device->pipe[5].cfg.ack = new;
	return count;
}

static ssize_t plw_pipe5_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;

	ret = nrf24_get_rx_pload_width(device->spi, 5);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t plw_pipe5_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u8 new;
	ssize_t old;

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new < 0 || new > PLOAD_MAX)
		return -EINVAL;
	old = nrf24_get_rx_pload_width(device->spi, 5);
	if (old < 0)
		return old;

	if ((uint8_t)old != new) {
		ret = nrf24_set_rx_pload_width(device->spi, 5, new);
		if (ret < 0)
			return ret;
		device->pipe[5].cfg.plw = new;
	}

	return count;
}

static ssize_t address_pipe5_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	uint8_t addr[16];
	int ret;
	int count;
	int i;

	ret = nrf24_get_address(device->spi, 5, addr);
	if (ret < 0)
		return ret;

	count = scnprintf(buf, PAGE_SIZE, "0x");
	for (i = --ret; i >= 0; i--)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%02X", addr[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t address_pipe5_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf,
			     size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u64 address;
	int len;

	ret = kstrtoull(buf, 16, &address);
	if (ret < 0)
		return ret;

	len = nrf24_get_address_width(device->spi);
	if (len < 0)
		return len;

	if (address >= BIT_ULL(len * BITS_PER_BYTE))
		return -EINVAL;

	ret = nrf24_set_address(device->spi, 5, (uint8_t *)&address);
	if (ret < 0)
		return ret;

	device->pipe[5].cfg.address = address;

	return count;
}

static DEVICE_ATTR_RW(ack_pipe5);
static DEVICE_ATTR_RW(plw_pipe5);
static DEVICE_ATTR_RW(address_pipe5);

struct attribute *nrf24_pipe5_attrs[] = {
	&dev_attr_ack_pipe5.attr,
	&dev_attr_plw_pipe5.attr,
	&dev_attr_address_pipe5.attr,
	NULL,
};

struct attribute_group nrf24_pipe0_group = {
	.attrs = nrf24_pipe0_attrs,
	.name = "nrf24_pipe0"
};

struct attribute_group nrf24_pipe1_group = {
	.attrs = nrf24_pipe1_attrs,
	.name = "nrf24_pipe1"
};

struct attribute_group nrf24_pipe2_group = {
	.attrs = nrf24_pipe2_attrs,
	.name = "nrf24_pipe2"
};

struct attribute_group nrf24_pipe3_group = {
	.attrs = nrf24_pipe3_attrs,
	.name = "nrf24_pipe3"
};

struct attribute_group nrf24_pipe4_group = {
	.attrs = nrf24_pipe4_attrs,
	.name = "nrf24_pipe4"
};

struct attribute_group nrf24_pipe5_group = {
	.attrs = nrf24_pipe5_attrs,
	.name = "nrf24_pipe5"
};

static ssize_t tx_address_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev);
	u8 addr[16];
	int ret;
	int count;
	int i;

	ret = nrf24_get_address(device->spi, NRF24_TX, addr);
	if (ret < 0)
		return ret;

	count = scnprintf(buf, PAGE_SIZE, "0x");
	for (i = --ret; i >= 0; i--)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%02X", addr[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t tx_address_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev);
	int ret;
	u64 address;
	int len;

	ret = kstrtoull(buf, 16, &address);
	if (ret < 0)
		return ret;

	len = nrf24_get_address_width(device->spi);
	if (len < 0)
		return len;

	if (address >= BIT_ULL(len * BITS_PER_BYTE))
		return -EINVAL;

	ret = nrf24_set_address(device->spi, NRF24_TX, (u8 *)&address);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t status_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	nrf24_print_status(device->spi);
	ret = nrf24_get_status(device->spi);
	if (ret < 0)
		return ret;
	return scnprintf(buf, PAGE_SIZE, "STATUS = 0x%02X\n", ret);
}

static ssize_t available_crc_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0 8 16\n");
}

static ssize_t crc_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	ret = nrf24_get_crc_mode(device->spi);
	if (ret < 0)
		return ret;

	switch (ret) {
	case NRF24_CRC_OFF:
		ret = scnprintf(buf, PAGE_SIZE, "0\n");
		break;
	case NRF24_CRC_8BIT:
		ret = scnprintf(buf, PAGE_SIZE, "8\n");
		break;
	case NRF24_CRC_16BIT:
		ret = scnprintf(buf, PAGE_SIZE, "16\n");
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static ssize_t crc_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	int ret;
	u8 new;
	struct nrf24_device *device;

	device = to_nrf24_device(dev);

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;

	switch (new) {
	case 0:
		new = NRF24_CRC_OFF;
		break;
	case 8:
		new = NRF24_CRC_8BIT;
		break;
	case 16:
		new = NRF24_CRC_16BIT;
		break;
	default:
		return -EINVAL;
	}

	ret = nrf24_get_crc_mode(device->spi);
	if (ret < 0)
		return ret;

	if (new != ret) {
		ret = nrf24_set_crc_mode(device->spi, new);
		if (ret < 0)
			return ret;
		dev_dbg(dev, "%s: new crc mode = %d\n", __func__, new);
	}
	return count;
}

static ssize_t available_address_width_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "3 4 5\n");
}

static ssize_t address_width_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	ret = nrf24_get_address_width(device->spi);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t address_width_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf,
				   size_t count)
{
	int ret;
	u8 new;
	struct nrf24_device *device;

	device = to_nrf24_device(dev);

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new != NRF24_AW_3 &&
	    new != NRF24_AW_4 &&
	    new != NRF24_AW_5)
		return -EINVAL;

	ret = nrf24_get_address_width(device->spi);
	if (ret < 0)
		return ret;

	if (new != ret) {
		ret = nrf24_set_address_width(device->spi, new);
		if (ret < 0)
			return ret;
		dev_dbg(dev, "%s: new address width = %d\n", __func__, new);
	}
	return count;
}

static ssize_t available_output_power_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0 -6 -12 -18\n");
}

static ssize_t rf_power_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	ret = nrf24_get_rf_power(device->spi);
	if (ret < 0)
		return ret;

	switch (ret) {
	case NRF24_POWER_0DBM:
		ret = scnprintf(buf, PAGE_SIZE, "0\n");
		break;
	case NRF24_POWER_6DBM:
		ret = scnprintf(buf, PAGE_SIZE, "-6\n");
		break;
	case NRF24_POWER_12DBM:
		ret = scnprintf(buf, PAGE_SIZE, "-12\n");
		break;
	case NRF24_POWER_18DBM:
		ret = scnprintf(buf, PAGE_SIZE, "-18\n");
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static ssize_t rf_power_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
	int ret;
	u8 new;
	s8 tmp;
	struct nrf24_device *device;

	device = to_nrf24_device(dev);

	ret = kstrtos8(buf, 10, &tmp);
	if (ret < 0)
		return ret;

	switch (abs(tmp)) {
	case 0:
		new = NRF24_POWER_0DBM;
		break;
	case 6:
		new = NRF24_POWER_6DBM;
		break;
	case 12:
		new = NRF24_POWER_12DBM;
		break;
	case 18:
		new = NRF24_POWER_18DBM;
		break;
	default:
		return -EINVAL;
	}

	ret = nrf24_get_rf_power(device->spi);
	if (ret < 0)
		return ret;

	if (new != ret) {
		ret = nrf24_set_rf_power(device->spi, new);
		if (ret < 0)
			return ret;
		dev_dbg(dev, "%s: new rf power level = %d\n", __func__, new);
	}
	return count;
}

static ssize_t available_data_rate_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "256 1024 2048\n");
}

static ssize_t data_rate_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	ret = nrf24_get_datarate(device->spi);
	if (ret < 0)
		return ret;

	switch (ret) {
	case NRF24_DATARATE_256KBPS:
		ret = scnprintf(buf, PAGE_SIZE, "256\n");
		break;
	case NRF24_DATARATE_1MBPS:
		ret = scnprintf(buf, PAGE_SIZE, "1024\n");
		break;
	case NRF24_DATARATE_2MBPS:
		ret = scnprintf(buf, PAGE_SIZE, "2048\n");
		break;
	}

	return ret;
}

static ssize_t data_rate_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	int ret;
	u16 new;
	u16 tmp;
	struct nrf24_device *device;

	device = to_nrf24_device(dev);

	ret = kstrtou16(buf, 10, &tmp);
	if (ret < 0)
		return ret;

	switch (tmp) {
	case 256:
		new = NRF24_DATARATE_256KBPS;
		break;
	case 1024:
		new = NRF24_DATARATE_1MBPS;
		break;
	case 2048:
		new = NRF24_DATARATE_2MBPS;
		break;
	default:
		return -EINVAL;
	}

	ret = nrf24_get_datarate(device->spi);
	if (ret < 0)
		return ret;

	if (new != ret) {
		ret = nrf24_set_datarate(device->spi, new);
		if (ret < 0)
			return ret;
		dev_dbg(dev, "%s: new datarate = %d\n", __func__, new);
	}
	return count;
}

static ssize_t available_retr_delay_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int i;
	int count = 0;

	for (i = 1; i <= 16; i++)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%d ", i * 250);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t retr_delay_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	ret = nrf24_get_auto_retr_delay(device->spi);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t retr_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	int ret;
	u16 new;
	struct nrf24_device *device;

	device = to_nrf24_device(dev);

	ret = kstrtou16(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new < 250 || new > 4000 || new % 250)
		return -EINVAL;

	ret = nrf24_get_auto_retr_delay(device->spi);
	if (ret < 0)
		return ret;

	if (new != ret) {
		ret = nrf24_set_auto_retr_delay(device->spi, new);
		if (ret < 0)
			return ret;
		dev_dbg(dev, "%s: new autr retr delay = %d\n", __func__, new);
	}
	return count;
}

static ssize_t available_retr_count_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int i;
	int count = 0;

	for (i = 0; i < 16; i++)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%d ", i);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t retr_count_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	ret = nrf24_get_auto_retr_count(device->spi);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t retr_count_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	int ret;
	u16 new;
	struct nrf24_device *device;

	device = to_nrf24_device(dev);

	ret = kstrtou16(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new < 0 || new > 15)
		return -EINVAL;

	ret = nrf24_get_auto_retr_count(device->spi);
	if (ret < 0)
		return ret;

	if (new != ret) {
		ret = nrf24_set_auto_retr_count(device->spi, new);
		if (ret < 0)
			return ret;
		dev_dbg(dev, "%s: new autr retr count = %d\n", __func__, new);
	}
	return count;
}

static DEVICE_ATTR_RW(tx_address);
static DEVICE_ATTR_RO(status);
static DEVICE_ATTR_RO(available_crc);
static DEVICE_ATTR_RW(crc);
static DEVICE_ATTR_RO(available_address_width);
static DEVICE_ATTR_RW(address_width);
static DEVICE_ATTR_RO(available_output_power);
static DEVICE_ATTR_RW(rf_power);
static DEVICE_ATTR_RO(available_data_rate);
static DEVICE_ATTR_RW(data_rate);
static DEVICE_ATTR_RO(available_retr_delay);
static DEVICE_ATTR_RW(retr_delay);
static DEVICE_ATTR_RO(available_retr_count);
static DEVICE_ATTR_RW(retr_count);

struct attribute *nrf24_attrs[] = {
	&dev_attr_tx_address.attr,
	&dev_attr_status.attr,
	&dev_attr_crc.attr,
	&dev_attr_available_crc.attr,
	&dev_attr_address_width.attr,
	&dev_attr_available_address_width.attr,
	&dev_attr_rf_power.attr,
	&dev_attr_available_output_power.attr,
	&dev_attr_data_rate.attr,
	&dev_attr_available_data_rate.attr,
	&dev_attr_retr_delay.attr,
	&dev_attr_available_retr_delay.attr,
	&dev_attr_retr_count.attr,
	&dev_attr_available_retr_count.attr,
	NULL,
};


const struct attribute_group nrf24_dev_general =
{
	.attrs = nrf24_attrs,
	.name   = "nrf24_general"
};

const struct attribute_group* nrf_24_device_attr_group[] = {
	&nrf24_dev_general,
	&nrf24_pipe0_group,
	&nrf24_pipe1_group,
	&nrf24_pipe2_group,
	&nrf24_pipe3_group,
	&nrf24_pipe4_group,
	&nrf24_pipe5_group,
	NULL
};
