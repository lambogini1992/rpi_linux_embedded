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
#include <linux/etherdevice.h>


#include "nrf24_net.h"
#include "nrf24_sysfs.h"
#include "nrf24_hal.h"


static int debug = -1;     /* defaults above */;

extern const struct attribute_group* nrf_24_device_attr_group[];

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

static int nrf24_hal_init(struct nrf24_device *device)
{
	int ret;
	struct spi_device *spi = device->spi;
	struct nrf24_pipe *pipe;
	uint8_t count_idx;
	
	nrf24_ce_lo(device);
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

static int nrf24_tx_thread(void *data)
{
	struct nrf24_device *device = data;
	struct nrf24_pipe *p;
	struct nrf24_tx_data tx_data;
	int ret;
	bool dpl;

	while (true) {
		dev_dbg(&device->dev,
				"%s: waiting for TX FIFO is not empty and RX mode time is timeout\n",
				__func__);
		wait_event_interruptible(device->tx_wait_queue,
					 			 kthread_should_stop() ||
					 			 (!device->rx_active && !kfifo_is_empty(&device->tx_fifo)));
		
		if (kthread_should_stop())
			return 0;
		dev_dbg(&device->dev,
				"%s: Have TX FIFO is available and RX mode is timeout\n",
				__func__);
		if (mutex_lock_interruptible(&device->tx_fifo_mutex))
			continue;

		ret = kfifo_out(&device->tx_fifo, &tx_data, sizeof(tx_data));
		if (ret != sizeof(tx_data)) {
			dev_dbg(&device->dev, "get tx_data from fifo failed\n");
			mutex_unlock(&device->tx_fifo_mutex);
			continue;
		}

		mutex_unlock(&device->tx_fifo_mutex);

		dpl = false;

RESEND_PKT:
		//enter Standby-I mode
		nrf24_ce_lo(device);

		if (nrf24_set_mode(device->spi, NRF24_MODE_TX) < 0)
			goto next;

		//check if dynamic payload length is enabled
		dpl = nrf24_get_dynamic_pl(device->spi);

		if (dpl) {
			//disable dynamic payload if pipe
			//does not use dynamic payload
			//and dynamic paload is enabled
			if (nrf24_disable_dynamic_pl(device->spi) < 0)
				goto next;
		}


		ret = nrf24_write_tx_pload(device->spi, tx_data.pload, tx_data.size);

		if (ret < 0) 
		{
			dev_dbg(&device->dev,
					"write TX PLOAD failed (%d)\n",
					ret);
			goto next;
		}

		//enter TX MODE and start transmission
		nrf24_ce_hi(device);

		//wait for ACK
		device->tx_done = false;
		wait_event_interruptible(device->tx_done_wait_queue,
					 			 (device->tx_done ||
								 kthread_should_stop()));

		if (kthread_should_stop())
		{
			return 0;
		}
			

		if(device->tx_failed)
		{
			goto RESEND_PKT;
		}

		//signal write function that write operation was finished
		p->write_done = true;
next:
		//restore dynamic payload feature
		if (dpl)
			nrf24_enable_dynamic_pl(device->spi);

		//if all sent enter RX MODE and start receiving
		if (kfifo_is_empty(&device->tx_fifo) || device->rx_active) {
			dev_dbg(&device->dev, "%s: NRF24_MODE_RX\n", __func__);
			//enter Standby-I
			nrf24_ce_lo(device);
			nrf24_set_mode(device->spi, NRF24_MODE_RX);
			nrf24_ce_hi(device);
		}
	}

	return 0;
}

/*************************************************/
/*THIS IS CALLBACK FUNCTION.
  RX mode of module will run in 1.5*Toa.
  When timeout is set flag. This function will be called
  It will set for TX thread run if TX FIFO isn't empty*/
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

/*HANDLE RX WORK STRUCT*/
static void nrf24_rx_handle_data(struct nrf24_device *device)
{
	struct net_device *ndev;
	ssize_t length;
	uint8_t pload[PLOAD_MAX];
	uint8_t pipe_idx;
	struct sk_buff *skb;

	dev_info(&device->dev,  "%s", __func__);
	ndev = device->net_dev;

	/*Get Index pipe have data in device fifo*/ 
	pipe_idx = nrf24_get_rx_data_source(device->spi);
	if((pipe_idx < 0) && (pipe_idx > NRF24_PIPE5))
	{
		dev_dbg(&device->dev, "%s: could not get pipe have incomming packet\n", __func__);
		return;
	}

	memset(pload, 0, PLOAD_MAX);
	/*Put address of pipe index into data buffer*/
	memcpy(pload, &(device->pipe[pipe_idx].cfg.address), sizeof(uint64_t));
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
	dev_dbg(&device->dev, "rx %zd bytes\n", length);

	skb = netdev_alloc_skb(ndev, length + NET_IP_ALIGN);
	if(!skb)
	{
		dev_err(&device->dev, "%s:out of memory for Rx'd frame\n", __func__);
		ndev->stats.rx_dropped++;
		return;
	}

	skb_reserve(skb, NET_IP_ALIGN);

	/*Add data of FIFO into socket*/ 
	memcpy(skb_put(skb, length), pload, length);
	/* update statistics */
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += length;
	/* Submit socket buffer to the network layer */
	netif_rx_ni(skb);
}

static void nrf24_rx_work_handler(struct work_struct *work)
{
	struct nrf24_device *device;
	printk(KERN_INFO "%s", __func__);
	device = container_of(work, struct nrf24_device, rx_work);
	if(!device)
	{
		return;
	}
	/*Check FIFO data is empty or not*/ 
	while (!nrf24_is_rx_fifo_empty(device->spi)) 
	{
		nrf24_rx_handle_data(device);	
	}
}

static void nrf24_tx_work_handler(struct work_struct *work)
{
	struct nrf24_device *device;
	struct nrf24_tx_data tx_data;
	printk(KERN_INFO "%s", __func__);
	device = container_of(work, struct nrf24_device, tx_work);

	if(!device)
	{
		return;
	}

	/*Check length of socket data is over than MAX of playload size or not*/
	if((device->tx_skb->len) > PLOAD_MAX)
	{
		goto invalid_len;
	}

	memcpy(tx_data.pload, device->tx_skb->data, device->tx_skb->len);
	tx_data.size = device->tx_skb->len;
	/*Lock TX FIFO mutex*/
	if (mutex_lock_interruptible(&device->tx_fifo_mutex))
	{
		goto exit_lock;
	}

	/*Put Socket data in to tx fifo*/ 
	if (kfifo_in(&device->tx_fifo, &tx_data, sizeof(tx_data)) != sizeof(tx_data))
	{
		goto exit_kfifo;
	}

exit_kfifo:
	/*Unlock tx fifo mutex*/
	mutex_unlock(&device->tx_fifo_mutex);
exit_lock:
	/*Trigger for TX thread run to transmit data*/
	wake_up_interruptible(&device->tx_wait_queue);
invalid_len:
	dev_kfree_skb(device->tx_skb);
	device->tx_skb = NULL;
}

static void nrf24_isr_work_handler(struct work_struct *work)
{
	struct nrf24_device *device;
	ssize_t status;
	uint32_t usecs;

	device = container_of(work, struct nrf24_device, isr_work);

	/*Get information of interrupt form status register*/ 
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


/*******************NETWORK DEVICE OPERATION******************************/

static int nrf24l01_net_open(struct net_device *dev)
{
	struct nrf24_device *priv = netdev_priv(dev);

	printk(KERN_INFO "%s", __func__);

	if (!is_valid_ether_addr(dev->dev_addr)) {
		if (netif_msg_ifup(priv))
			netdev_err(dev, "invalid MAC address %pM\n", dev->dev_addr);
		return -EADDRNOTAVAIL;
	}

	nrf24_hal_init(priv);
	netif_start_queue(dev);

	return 0;
}

static int nrf24l01_net_release(struct net_device *dev) {
    printk(KERN_INFO "%s", __func__);
    netif_stop_queue(dev);
    return 0;
}

/*This function is call when we write new packet into socket*/
static netdev_tx_t nrf24l01_send_packet(struct sk_buff *skb,
					struct net_device *dev)
{
	struct nrf24_device *priv = netdev_priv(dev);
	printk(KERN_INFO "%s", __func__);
	/* If some error occurs while trying to transmit this
	 * packet, you should return '1' from this function.
	 * In such a case you _may not_ do anything to the
	 * SKB, it is still owned by the network queueing
	 * layer when an error is returned. This means you
	 * may not modify any SKB fields, you may not free
	 * the SKB, etc.
	 */
	netif_stop_queue(dev);

	/* Remember the skb for deferred processing */
	priv->tx_skb = skb;
	schedule_work(&priv->tx_work);

	return NETDEV_TX_OK;
}

/*This function is called TX transmittion is timeout*/ 
static void nrf24l01_timeout(struct net_device *dev)
{
	struct nrf24_device *priv = netdev_priv(dev);

	printk(KERN_ERR "%s: xmitter timed out, try to restart!\n", __func__);
	if (netif_msg_timer(priv))
	{
		dev_err(&priv->dev, "nrf24l01 tx timeout\n");
	}
	dev->stats.tx_errors++;
	priv->rx_active = false;
	priv->tx_done   = true;
	priv->tx_failed = true;
	nrf24_hal_init(priv);
}


static const struct net_device_ops nrf24l01_netdev_ops = {
	.ndo_open			= nrf24l01_net_open,
	.ndo_stop       	= nrf24l01_net_release,
	.ndo_start_xmit		= nrf24l01_send_packet,
	.ndo_tx_timeout		= nrf24l01_timeout,
	.ndo_validate_addr	= eth_validate_addr,
};

static void nrf24_dev_release(struct device *dev)
{
	printk(KERN_INFO "%s\n", __func__);
}

static struct device_type nrf24_dev_type = {
	.name = "nrf24_device",
	.release = nrf24_dev_release,
};

/*IRQ callback function*/
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

static struct nrf24_device *nrf24_network_init(struct spi_device *spi)
{
	struct net_device *dev;
	struct nrf24_device *priv;
	int ret;

	dev = alloc_etherdev(sizeof(struct nrf24_device));
	if(!dev)
	{
		dev_err(&spi->dev, "%s: cannot allocate ethernet device struct", __func__);
		goto failed_alloc_netdev; 
	}

	priv = netdev_priv(dev);

	priv->net_dev 		= dev;
	priv->spi			= spi;
	priv->msg_enable 	= netif_msg_init(debug, NRF24L1_MSG_DEFAULT);
	
	dev_set_name(&priv->dev, "nrf24l01");
	priv->dev.parent = &spi->dev;
	priv->nrf24_class = class_create(THIS_MODULE, "nrf24l01_device_class");
	priv->dev.class = priv->nrf24_class;
	priv->dev.type = &nrf24_dev_type;
	priv->dev.groups = nrf_24_device_attr_group;
	ret = device_register(&priv->dev);
	if (ret < 0) {
		goto FAILED_TO_DEVICE_REGISTER;
	}

	init_waitqueue_head(&priv->tx_wait_queue);
	init_waitqueue_head(&priv->tx_done_wait_queue);
	INIT_WORK(&priv->isr_work, nrf24_isr_work_handler);
	INIT_WORK(&priv->rx_work, nrf24_rx_work_handler);
	INIT_WORK(&priv->tx_work, nrf24_tx_work_handler);
	INIT_KFIFO(priv->tx_fifo);
	spin_lock_init(&priv->lock);
	mutex_init(&priv->tx_fifo_mutex);

	priv->tx_done 			= false;
	priv->tx_failed 		= false;
	priv->rx_active 		= false;

	
	//This is timeout when device re-transmit over the time
	dev->watchdog_timeo = msecs_to_jiffies(TX_TIMEOUT);
	
	if(nrf24_gpio_setup(priv) < 0)
	{
		goto FAILED_TO_INTI_HANDLE_GPIO;
	}

	dev->if_port 			= IF_PORT_10BASET;
	dev->irq				= spi->irq;
	dev->netdev_ops 		= &nrf24l01_netdev_ops;

	ret = register_netdev(dev);
	if (ret) {
		if (netif_msg_probe(priv))
		{
			dev_err(&spi->dev, "register netdev failed (ret = %d)\n",
					ret);
			goto FAILED_REGISTER_NETDEV;
		}	
	}
	dev_info(&spi->dev, "%s: register netdev success\n",__func__);
	timer_setup(&priv->rx_active_timer, nrf24_rx_active_timer_cb, 0);
	
	spi_set_drvdata(spi, priv);
	return priv;
FAILED_REGISTER_NETDEV:
	free_irq(spi->irq, &priv->dev);
FAILED_TO_INTI_HANDLE_GPIO:
	device_unregister(&priv->dev);
FAILED_TO_DEVICE_REGISTER:
	put_device(&priv->dev);
failed_alloc_netdev:
	return NULL;
}

static int nrf24_probe(struct spi_device *spi)
{
    int ret;
	struct nrf24_device *device;

	dev_info(&spi->dev, "%s: Loading Probe Function\n", __func__);
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;

	ret = spi_setup(spi);
	if(ret < 0)
	{
		dev_err(&spi->dev, "%s: set-up SPI failed\n", __func__);
		return ret;
	}

	device = nrf24_network_init(spi);
	if(!device)
	{
		dev_err(&spi->dev, "%s: failed to initialize device driver\n", __func__);
		return ret;
	}

	device->tx_task_struct = kthread_run(nrf24_tx_thread,
					     device,
					     "nrf24_tx_thread");
	if (IS_ERR(device->tx_task_struct)) 
	{
		dev_err(&device->dev, "start of tx thread failed\n");
		goto err_devs_destroy;
	}

	return 0;
err_devs_destroy:
	unregister_netdev(device->net_dev);
	nrf24_gpio_free(device);
	device_unregister(&device->dev);
	class_destroy(device->nrf24_class);
	free_netdev(device->net_dev);
	return ret;
}

static int nrf24_remove(struct spi_device *spi)
{
	struct nrf24_device *device = spi_get_drvdata(spi);

	kthread_stop(device->tx_task_struct);
	dev_info(&spi->dev, "%s: kthread_stop\n", __func__);
	unregister_netdev(device->net_dev);
	dev_info(&spi->dev, "%s: unregister_netdev\n", __func__);
	nrf24_gpio_free(device);
	dev_info(&spi->dev, "%s: nrf24_gpio_free\n", __func__);
	device_unregister(&device->dev);
	dev_info(&spi->dev, "%s: device_unregister\n", __func__);
	class_destroy(device->nrf24_class);
	dev_info(&spi->dev, "%s: class_destroy\n", __func__);
	free_netdev(device->net_dev);
	dev_info(&spi->dev, "%s: free_netdev\n", __func__);
	return 0;
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