#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>


#include "nrf24_net.h"
#include "nrf24_sysfs.h"
#include "nrf24_hal.h"


extern const struct attribute_group* nrf_24_device_attr_group[];

static struct device_type nrf24_dev_type = {
	.name = "nrf24_device",
};

static void nrf24_ce_hi(struct nrf24_device *device);
static void nrf24_ce_lo(struct nrf24_device *device);

static void nrf24_ce_hi(struct nrf24_device *device)
{
	gpiod_set_value(device->ce, 1);
}

static void nrf24_ce_lo(struct nrf24_device *device)
{
	gpiod_set_value(device->ce, 0);
}

static void nrf24_rx_active_timer_cb(struct timer_list *t)
{
	struct nrf24_device *device = from_timer(device, t, rx_active_timer);

	dev_dbg(&device->dev, "RX finished\n");

	device->rx_active = false;

	if (!kfifo_is_empty(&device->tx_fifo)) {
		dev_dbg(&device->dev, "wake up TX...\n");
		wake_up_interruptible(&device->tx_wait_queue);
	}
}
static void nrf24_rx_handle_data(struct nrf24_device *device)
{
	struct nrf24_device *device;
	struct net_device *ndev;
	ssize_t pipe;
	ssize_t length;
	uint8_t pload[PLOAD_MAX];
	uint64_t pipe_add;
	uint8_t pipe_idx;
	struct sk_buff *skb;

	ndev = device->net_dev;

	pipe_idx = nrf24_get_rx_data_source(device->spi);
	if((pipe_idx < 0) && (pipe_idx > NRF24_PIPE5))
	{
		dev_dbg(&device->dev,
				"%s: could not get pipe have incomming packet\n",
				__func__);
		return;
	}

	memset(pload, 0, PLOAD_MAX);
	memcpy(pload, &(device->pipe[pipe_idx].address), sizeof(uint64_t));
	length = nrf24_read_rx_pload(device->spi, pload + PAYLOAD_DATA_PACKET_POS);
	if (length < 0 || length > PLOAD_MAX) 
	{
		dev_dbg(&device->dev,
			"%s: could not read pload (err = %zd) of pipe %d\n",
			__func__,
			length, pipe_idx);
		return;
	}
	length = length + sizeof(uint64_t);
	dev_dbg(&device->dev,, "rx %zd bytes\n", length);

	skb = netdev_alloc_skb(ndev, length + NET_IP_ALIGN);
	if(!skb)
	{
		netdev_err(ndev, "%s:out of memory for Rx'd frame\n", __func__);
		ndev->stats.rx_dropped++;
		return;
	}

	skb_reserve(skb, NET_IP_ALIGN);
	memcpy(skb_put(sbk, length), pload, length);
	/* update statistics */
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += len;
	/* Submit socket buffer to the network layer */
	netif_rx_ni(skb);
}

static void nrf24_rx_work_handler(struct work_struct *work)
{
	struct nrf24_device *device;

	device = container_of(work, struct nrf24_device, rx_work);
	if(!device)
	{
		return;
	}
	while (!nrf24_is_rx_fifo_empty(device->spi)) 
	{
		nrf24_rx_handle_data(device);	
	}
}

static void nrf24_isr_work_handler(struct work_struct *work)
{
	struct nrf24_device *device;
	ssize_t status;
	uint32_t usecs;

	device = container_of(work, struct nrf24_device, isr_work);

	status = nrf24_get_status(device->spi);
	if (status < 0)
		return;

	if (status & RX_DR) {
		dev_dbg(&device->dev, "%s: RX_DR\n", __func__);
		device->rx_active = true;
		//keep rx active untli next time recevied for 1.5*Toa
		usecs = 8192 * (1 + device->cfg.address_width + PLOAD_MAX + (device->cfg.crc - 1)) + 9;
		usecs /= device->cfg.data_rate;
		usecs += (usecs / 2);
		dev_dbg(&device->dev, "rx_active_timer = %u us\n", usecs);
		mod_timer(&device->rx_active_timer,
			  jiffies + usecs_to_jiffies(usecs));
		nrf24_clear_irq(device->spi, RX_DR);
		schedule_work(&device->rx_work);
	}

	if (status & TX_DS) {
		dev_dbg(&device->dev, "%s: TX_DS\n", __func__);
		device->tx_failed = false;
		device->tx_done = true;
		nrf24_clear_irq(device->spi, TX_DS);
		wake_up_interruptible(&device->tx_done_wait_queue);
	}

	if (status & MAX_RT) {
		dev_dbg_ratelimited(&device->dev, "%s: MAX_RT\n", __func__);
		device->tx_failed = true;
		device->tx_done = true;
		nrf24_flush_tx_fifo(device->spi);
		nrf24_clear_irq(device->spi, MAX_RT);
		wake_up_interruptible(&device->tx_done_wait_queue);
	}
}

static irqreturn_t nrf24_isr(int irq, void *dev_id)
{
	unsigned long flags;
	struct nrf24_device *device = dev_id;

	spin_lock_irqsave(&device->lock, flags);

	schedule_work(&device->isr_work);

	spin_unlock_irqrestore(&device->lock, flags);

	return IRQ_HANDLED;
}

static void nrf24_gpio_free(struct nrf24_device *device)
{
	if (!IS_ERR(device->ce))
		gpiod_put(device->ce);

	free_irq(device->spi->irq, device);
}

static struct nrf24_device *nrf24_network_init(struct spi_device *spi)
{
	struct net_device *dev;
	struct nrf24_device *priv;
	int ret;

	dev = alloc_etherdev(sizeof(struct nrf24_device));
	if(!dev)
	{
		dev_err(&spi->dev, "%s: cannot allocate ethernet device struct", __func__);
		return ERR_PTR(-ENOMEM); 
	}

	priv = netdev_priv(dev);

	priv->net_dev 		= dev;
	priv->spi			= spi;
	priv->msg_enable 	= netif_msg_init(debug.msg_enable, NRF24L1_MSG_DEFAULT);
	
	dev_set_name(&device->dev, "nrf24l01");
	device->dev.parent = &spi->dev;
	device->dev.class = class_create(THIS_MODULE, "nrf24l01_device_class");
	device->dev.type = &nrf24_dev_type;
	device->dev.groups = nrf_24_device_attr_group;
	ret = device_register(&device->dev);
	if (ret < 0) {
		goto FAILED_TO_DEVICE_REGISTER;
	}

	init_waitqueue_head(&device->tx_wait_queue);
	init_waitqueue_head(&device->tx_done_wait_queue);
	INIT_WORK(&device->isr_work, nrf24_isr_work_handler);
	INIT_WORK(&device->rx_work, nrf24_rx_work_handler);
	INIT_WORK(&device->tx_work, nrf24_tx_work_handler);
	spin_lock_init(&device->lock);
	mutex_init(&device->tx_skb_mutex);

	device->tx_done 		= ATOMIC_INIT(0);
	device->tx_failed 		= ATOMIC_INIT(0);
	device->rx_active 		= ATOMIC_INIT(0);

	timer_setup(&device->rx_active_timer, nrf24_rx_active_timer_cb, 0);
	
	spi_set_drvdata(spi, device);
	return priv;
FAILED_TO_DEVICE_REGISTER:
	put_device(&device->dev);
	return ret;
}

static int nrf24_gpio_setup(struct nrf24_device *device)
{
	int ret;

	device->ce = gpiod_get(&device->spi->dev, "ce", 0);

	if (device->ce == ERR_PTR(-ENOENT))
	{
		dev_dbg(&device->dev, "%s: no entry for CE\n", __func__);
	}
	else if (device->ce == ERR_PTR(-EBUSY))
	{
		dev_dbg(&device->dev, "%s: CE is busy\n", __func__);
	}

	if (IS_ERR(device->ce)) {
		ret = PTR_ERR(device->ce);
		dev_err(&device->dev, "%s: CE gpio setup error\n", __func__);
		return ret;
	}

	nrf24_ce_lo(device);

	ret = request_irq(device->spi->irq,
			  nrf24_isr,
			  0,
			  dev_name(&device->dev),
			  device);
	if (ret < 0) {
		gpiod_put(device->ce);
		return ret;
	}

	return 0;
}

static int nrf24_hal_init(struct nrf24_device *device)
{
	int ret;
	struct spi_device *spi = device->spi;
	struct nrf24_pipe *pipe;
	uint8_t count_idx;

	ret = nrf24_soft_reset(spi);
	if (ret < 0)
		return ret;

	for(count_idx = 0; count_idx < 6; count_idx++)
	{
		device->pipe[count_idx].cfg.plw = 0;
		ret = nrf24_set_rx_pload_width(spi, pipe->id, 0);
		if (ret < 0)
			return ret;
	}

	ret = nrf24_flush_fifo(spi);
	if (ret < 0)
		return ret;

	ret = nrf24_open_pipe(spi, NRF24_PIPE_ALL);
	if (ret < 0)
		return ret;

	ret = nrf24_lock_unlock(spi);
	if (ret < 0)
		return ret;

	ret = nrf24_set_mode(spi, NRF24_MODE_RX);
	if (ret < 0)
		return ret;

	device->cfg.crc = NRF24_CRC_16BIT;
	ret = nrf24_set_crc_mode(spi, NRF24_CRC_16BIT);
	if (ret < 0)
		return ret;

	device->cfg.retr_count = 15;
	ret = nrf24_set_auto_retr_count(spi, 15);
	if (ret < 0)
		return ret;

	device->cfg.retr_delay = 4000;
	ret = nrf24_set_auto_retr_delay(spi, 4000);
	if (ret < 0)
		return ret;

	device->cfg.rf_power = NRF24_POWER_0DBM;
	ret = nrf24_set_rf_power(spi, NRF24_POWER_0DBM);
	if (ret < 0)
		return ret;

	device->cfg.data_rate = NRF24_DATARATE_2MBPS;
	ret = nrf24_set_datarate(spi, NRF24_DATARATE_2MBPS);
	if (ret < 0)
		return ret;

	ret = nrf24_get_address_width(spi);
	if (ret < 0)
		return ret;

	device->cfg.address_width = ret;

	ret = nrf24_power_up(spi);
	if (ret < 0)
		return ret;

	nrf24_ce_hi(device);

	return ret;
}

static int nrf24_probe(struct spi_device *spi)
{
    int ret;
	struct nrf24_device *device;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;

	ret = spi_setup(spi)
	if(ret < 0)
	{
		dev_err(&spi->dev, "%s: set-up SPI failed\n", __func__);
		return ret;
	}


}

static int nrf24_remove(struct spi_device *spi)
{

}

static const struct of_device_id nrf24_dt_ids[] = {
	{ .compatible = "nordic,nrf24" },
	{},
};

MODULE_DEVICE_TABLE(of, nrf24_dt_ids);

static  struct spi_driver nrf24_spi_driver = {
	.driver = {
		.name = "nrf24",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(nrf24_dt_ids),
	},
	.probe = nrf24_probe,
	.remove = nrf24_remove,
};


module_spi_driver(nrf24_spi_driver);

MODULE_AUTHOR("Pham Tuan Anh <lambogini1992@gmail.com>");
MODULE_DESCRIPTION("Driver for NRF24L01+");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:nrf24");