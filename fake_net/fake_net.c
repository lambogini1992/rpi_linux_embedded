#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>

#include <linux/sched.h>
#include <linux/kernel.h> /* printk() */
#include <linux/slab.h> /* kmalloc() */
#include <linux/errno.h>  /* error codes */
#include <linux/types.h>  /* size_t */
#include <linux/interrupt.h> /* mark_bh */
#include <linux/of.h>
#include <linux/in.h>
#include <linux/netdevice.h>   /* struct device, and other headers */
#include <linux/etherdevice.h> /* eth_type_trans */
#include <linux/ip.h>          /* struct iphdr */
#include <linux/tcp.h>         /* struct tcphdr */
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/of_net.h>
#include <linux/of_device.h>
#include <net/mac802154.h>

static int lockup = 0;
static int timeout = 5;

static uint8_t mac_addr[6] = {0x26, 0x1c, 0x42, 0xc9, 0x0f, 0x26};

struct fake_net {
	struct net_device_stats stats;
	struct net_device *ndev;
	struct spi_device *spi;
	struct work_struct irqwork;
	/* Lock the RX and TX actions. */
	uint8_t *tx_data;
	uint32_t tx_data_len;
};

static void fake_net_rx_complete(struct net_device *ndev)
{
	struct fake_net *priv;
	struct sk_buff *rx_skb;
	int ret;

	printk(KERN_INFO "%s\n", __func__);
	priv = netdev_priv(ndev);

		/*
	 * The packet has been retrieved from the transmission
	 * medium. Build an skb around it, so upper layers can handle it
	 */
	rx_skb = dev_alloc_skb(priv->tx_data_len + 2);
	if (!rx_skb) 
	{
		priv->stats.rx_dropped++;
		printk(KERN_ERR "%s: Failed to allocate memory of socket buffer\n", __func__);
		return;
	}

	skb_reserve(rx_skb, 2); /* align IP on 16B boundary */  

	memcpy(skb_put(rx_skb, priv->tx_data_len), priv->tx_data, priv->tx_data_len );
	kfree(priv->tx_data);
	rx_skb->dev = ndev;
	rx_skb->protocol = eth_type_trans(rx_skb, ndev);
	rx_skb->ip_summed = CHECKSUM_UNNECESSARY; /* don't check it */
	priv->stats.rx_packets++;
	priv->stats.rx_bytes += priv->tx_data_len;
	printk(KERN_INFO "%s: Publish Packet Socket\n", __func__);
	netif_rx(rx_skb);
}

static int fake_net_open(struct net_device *dev)
{
	printk(KERN_INFO "%s\n", __func__);
	memcpy(dev->dev_addr, mac_addr, ETH_ALEN);
	netif_start_queue(dev);
	return 0;
}

static int fake_net_release(struct net_device *dev)
{
    /* release ports, irq and such -- like fops->close */
	printk(KERN_INFO "%s\n", __func__);
	netif_stop_queue(dev); /* can't transmit any more */
	return 0;
}

/*
 * Configuration changes (passed on by ifconfig)
 */
static int fake_net_config(struct net_device *dev, struct ifmap *map)
{
	printk(KERN_INFO "%s\n", __func__);
	if (dev->flags & IFF_UP) /* can't act on a running interface */
		return -EBUSY;

	/* Don't allow changing the I/O address */
	if (map->base_addr != dev->base_addr) {
		printk(KERN_WARNING "snull: Can't change I/O address\n");
		return -EOPNOTSUPP;
	}

	/* Allow changing the IRQ */
	if (map->irq != dev->irq) {
		dev->irq = map->irq;
        	/* request_irq() is delayed to open-time */
	}

	/* ignore other fields */
	return 0;
}
/*
 * Transmit a packet (low level interface)
 */
static void fake_net_hw_tx(uint8_t *data, uint32_t len, struct net_device *dev)
{
	/*
	 * This function deals with hw details. This interface loops
	 * back the packet to the other snull interface (if any).
	 * In other words, this function implements the snull behaviour,
	 * while all other procedures are rather device-independent
	 */

	struct iphdr *ih;
	u32 *saddr, *daddr;
    struct fake_net *priv = netdev_priv(dev);
	/* I am paranoid. Ain't I? */
	printk(KERN_INFO "%s\n", __func__);
	if (len < sizeof(struct ethhdr) + sizeof(struct iphdr)) {
		printk("snull: Hmm... packet too short (%i octets)\n",
				len);
		return;
	}
	/*
	 * Ethhdr is 14 bytes, but the kernel arranges for iphdr
	 * to be aligned (i.e., ethhdr is unaligned)
	 */
	ih = (struct iphdr *)(data +sizeof(struct ethhdr));
	saddr = &ih->saddr;
	daddr = &ih->daddr;

	((u8 *)saddr)[2] ^= 1; /* change the third octet (class C) */
	((u8 *)daddr)[2] ^= 1;

	ih->check = 0;         /* and rebuild the checksum (ip needs it) */
	ih->check = ip_fast_csum((unsigned char *)ih,ih->ihl);
	
	printk(KERN_INFO "%08x:%05i --> %08x:%05i\n",
		   ntohl(ih->saddr),ntohs(((struct tcphdr *)(ih+1))->source),
		   ntohl(ih->daddr),ntohs(((struct tcphdr *)(ih+1))->dest));

	/*
	 * Ok, now the packet is ready for transmission: first simulate a
	 * receive interrupt on the twin device, then  a
	 * transmission-done on the transmitting device
	 */
	priv->tx_data = data;
	priv->tx_data_len = len;
	priv->stats.tx_packets++;
	if (lockup && ((priv->stats.tx_packets + 1) % lockup) == 0) 
	{
        	/* Simulate a dropped transmit interrupt */
		netif_stop_queue(dev);
		printk(KERN_ERR "Simulate lockup at %ld, txp %ld\n", jiffies,
				(unsigned long) priv->stats.tx_packets);
	}
	else
	{
		printk(KERN_INFO "Schedule rx work queue\n");
		schedule_work(&priv->irqwork);
	}
}

/*
 * Transmit a packet (called by the kernel)
 */
static int fake_net_tx(struct sk_buff *skb, struct net_device *dev)
{
	int len;
	uint8_t *data;
	struct snull_priv *priv = netdev_priv(dev);
	printk(KERN_INFO "%s\n", __func__);
	netif_trans_update(dev);

	/* Remember the skb, so we can free it at interrupt time */
	data = kmalloc(skb->len, GFP_KERNEL);
	len = skb->len;
	memcpy(data, skb->data, len);
	/* actual deliver of data is device-specific, and not shown here */
	fake_net_hw_tx(data, len, dev);

	return 0; /* Our simple device can not fail */
}

/*
 * Ioctl commands 
 */
static int fake_net_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	printk(KERN_INFO "%s\n", __func__);
	return 0;
}

/*
 * Return statistics to the caller
 */
static struct net_device_stats *fake_net_stats(struct net_device *dev)
{
	struct fake_net *priv = netdev_priv(dev);
	printk(KERN_INFO "%s\n", __func__);
	return &priv->stats;
}

static int fake_net_header(struct sk_buff *skb, struct net_device *dev,
                unsigned short type, const void *daddr, const void *saddr,
                unsigned len)
{
	struct ethhdr *eth = (struct ethhdr *)skb_push(skb,ETH_HLEN);
	printk(KERN_INFO "%s\n", __func__);
	eth->h_proto = htons(type);
	memcpy(eth->h_source, dev->dev_addr, dev->addr_len);
	memcpy(eth->h_dest,   dev->dev_addr, dev->addr_len);
	eth->h_dest[ETH_ALEN-1]   ^= 0x01;   /* dest is us xor 1 */
	return (dev->hard_header_len);
}

/*
 * The "change_mtu" method is usually not needed.
 * If you need it, it must be like this.
 */
static int fake_net_change_mtu(struct net_device *dev, int new_mtu)
{
	unsigned long flags;
	printk(KERN_INFO "%s\n", __func__);
	/* check ranges */
	if ((new_mtu < 68) || (new_mtu > 1500))
		return -EINVAL;
	/*
	 * Do anything you need, and the accept the value
	 */
	dev->mtu = new_mtu;
	return 0; /* success */
}

static void fake_net_tx_timeout (struct net_device *dev)
{
	printk(KERN_INFO "%s\n", __func__);
}

static const struct header_ops fake_net_header_ops = {
        .create  = fake_net_header,
};

static const struct net_device_ops fake_net_netdev_ops = {
	.ndo_open            = fake_net_open,
	.ndo_stop            = fake_net_release,
	.ndo_start_xmit      = fake_net_tx,
	.ndo_do_ioctl        = fake_net_ioctl,
	.ndo_set_config      = fake_net_config,
	.ndo_get_stats       = fake_net_stats,
	.ndo_change_mtu      = fake_net_change_mtu,
	.ndo_tx_timeout      = fake_net_tx_timeout,
};

static void fake_net_irqwork(struct work_struct *work)
{
	struct fake_net *priv;
	printk(KERN_INFO "%s\n", __func__);
	priv = container_of(work, struct fake_net, irqwork);
	fake_net_rx_complete(priv->ndev);
}

void fake_net_init(struct net_device *dev)
{
	ether_setup(dev); /* assign some of the fields */
	dev->watchdog_timeo = timeout;
	dev->netdev_ops 	= &fake_net_netdev_ops;
	dev->header_ops 	= &fake_net_header_ops;
	/* keep the default flags, just add NOARP */
	dev->flags         |= IFF_NOARP;
	dev->features      |= NETIF_F_HW_CSUM;
}

static int fake_net_probe(struct spi_device *spi)
{
	struct net_device *ndev;
	struct fake_net *priv;
	int ret;
	ndev = alloc_netdev(sizeof(struct fake_net), "fake_net",
		                NET_NAME_UNKNOWN, fake_net_init);
	if(!ndev)
	{
		ret = -ENOMEM;
		dev_err(&spi->dev, "%s: failed to allocate memory network device\n", __func__);
		return ret;
	}

	priv = netdev_priv(ndev);
	if(!priv)
	{
		ret = -ENOMEM;
		dev_err(&spi->dev, "%s: failed to allocate memory private fake_net\n", __func__);
		return ret;
	}

	priv->ndev = ndev;
	priv->spi  = spi;
	INIT_WORK(&priv->irqwork, fake_net_irqwork);
	spi_set_drvdata(spi, priv);
	ret = register_netdev(ndev);
	if(ret)
	{
		ret = -ENODEV;
		dev_err(&spi->dev, "%s: failed to register net device\n", __func__);
		return ret;
	}

	dev_info(&spi->dev, "%s: Create net device successful\n", __func__);
	return 0;
}

static int fake_net_remove(struct spi_device *spi)
{
	struct fake_net *priv = spi_get_drvdata(spi);
	struct net_device *ndev = priv->ndev;

	unregister_netdev(ndev);
	free_netdev(ndev);
	return 0;
}

static const struct of_device_id fake_net_dt_ids[] = {
	{ .compatible = "nordic,nrf24" },
	{},
};

MODULE_DEVICE_TABLE(of, fake_net_dt_ids);

static  struct spi_driver fake_net_spi_driver = {
	.driver = {
		.name = "fake_net",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(fake_net_dt_ids),
	},
	.probe = fake_net_probe,
	.remove = fake_net_remove,
};

static void __exit fake_net_rm_module(void)
{
    spi_unregister_driver(&fake_net_spi_driver);
}

static int __init fake_net_init_module(void)
{
    return spi_register_driver(&fake_net_spi_driver);
}

module_init(fake_net_init_module);
module_exit(fake_net_rm_module);

MODULE_AUTHOR("Pham Tuan Anh <lambogini1992@gmail.com>");
MODULE_DESCRIPTION("Driver for fake_net+");
MODULE_LICENSE("GPL");
