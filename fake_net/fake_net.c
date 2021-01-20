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


struct fake_net_device{
    struct net_device *ndev;
    struct sk_buff    *tx_buff;
    struct spi_device *spi;

    struct work_struct skb_work;
};

static int fake_net_open(struct net_device *ndev)
{
    printk(KERN_INFO "%s", __func__);

    netif_start_queue(ndev);
    return 0;
}

static int fake_net_release(struct net_device *ndev)
{
    printk(KERN_INFO "%s", __func__);
    netif_stop_queue(ndev);
    return 0;
}


static netdev_tx_t fake_net_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct fake_net_device *priv = netdev_priv(dev);
    printk(KERN_INFO "%s", __func__);
    netif_stop_queue(dev);

    priv->tx_buff = skb;
    schedule_work(&priv->skb_work);

    return NETDEV_TX_OK;
}

static const struct net_device_ops fake_net_ops = {
	.ndo_open			= fake_net_open,
	.ndo_stop       	= fake_net_release,
	.ndo_start_xmit		= fake_net_xmit,
	.ndo_validate_addr	= eth_validate_addr,
};

static void skb_work_struct(struct work_struct *work)
{
    struct fake_net_device *priv ;
    struct sk_buff *rx_buff;
    struct net_device *ndev;
    uint16_t length;

    priv = container_of(work, struct fake_net_device, skb_work);
    ndev = priv->ndev;
    length = priv->tx_buff->len;
    rx_buff = netdev_alloc_skb(ndev, length + NET_IP_ALIGN);
	if(!rx_buff)
	{
		printk(KERN_ERR "%s:out of memory for Rx'd frame\n", __func__);
		ndev->stats.rx_dropped++;
		return;
	}

    skb_reserve(rx_buff, NET_IP_ALIGN);

    memcpy(skb_put(rx_buff, length), priv->tx_buff->data, length);
    ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += length;

    netif_rx_ni(rx_buff);
}

static void fake_net_init(struct net_device *ndev)
{
    struct fake_net_device   *priv;
    printk(KERN_INFO "%s\n", __func__);
    priv = netdev_priv(ndev);

    
    priv->ndev  = ndev;

    INIT_WORK(&priv->skb_work, skb_work_struct);

    ndev->netdev_ops  = &fake_net_ops; 
    ndev->if_port     = IF_PORT_10BASET;
    ndev->flags      |= IFF_NOARP;
}

static int fake_net_probe(struct spi_device *spi)
{
    int ret;
    struct net_device *ndev;
    struct fake_net_device   *priv;
    printk(KERN_INFO "%s", __func__);
    ndev = alloc_netdev(sizeof(struct fake_net_device), "fake_eth", NET_NAME_UNKNOWN, fake_net_init);
    if(!ndev)
    {
        printk(KERN_ERR "%s: cannot allocate ethernet device struct\n", __func__);
        ret = -ENODEV;
        goto failed_alloc_net_device;
    }
    printk(KERN_INFO "%s finish allocate net device\n", __func__);
    

    ret = register_netdev(ndev);
    if(ret)
    {
        printk(KERN_ERR "%s: cannot register network device\n", __func__);
        goto failed_alloc_net_device;
    }
    printk(KERN_INFO "%s finish register net device\n", __func__);

    priv = netdev_priv(ndev);
    priv->spi   = spi;
    spi_set_drvdata(spi, priv);

    return 0;
failed_alloc_net_device:
    return ret;
}

static int fake_net_remove(struct spi_device *spi)
{
    struct fake_net_device   *priv;

    priv = spi_get_drvdata(spi);

    unregister_netdev(priv->ndev);
    free_netdev(priv->ndev);

    return 0;
}

static const struct of_device_id fake_net_ids[] = {
	{ .compatible = "nordic,nrf24" },
	{},
};

MODULE_DEVICE_TABLE(of, fake_net_ids);

static  struct spi_driver fake_net_driver = {
	.driver = {
		.name = "nrf24",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(fake_net_ids),
	},
	.probe = fake_net_probe,
	.remove = fake_net_remove,
};



void fake_net_rm_module(void)
{
    spi_unregister_driver(&fake_net_driver);
}

int fake_net_init_module(void)
{
    return spi_register_driver(&fake_net_driver);
}

module_init(fake_net_init_module);
module_exit(fake_net_rm_module);

MODULE_AUTHOR("Pham Tuan Anh <lambogini1992@gmail.com>");
MODULE_DESCRIPTION("Driver for NRF24L01+");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:nrf24");