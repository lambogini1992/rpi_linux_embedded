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


struct fake_net {
	struct ieee802154_hw *hw;
	struct spi_device *spi;
	struct regmap *map;
	struct timer_list timer;
	struct work_struct irqwork;
	/* Lock the RX and TX actions. */
	spinlock_t buf_lock;
	struct sk_buff *tx_buf;
	bool one_to_be_sent;
	bool post_tx_done;
	bool is_busy;
};

static void
fake_net_rx_complete(struct ieee802154_hw *hw)
{
	struct fake_net *priv = hw->priv;
	struct spi_device *spi = priv->spi;
	int len = priv->tx_buf->len;
	char *data = priv->tx_buf->data;
	struct sk_buff *skb;

	skb = dev_alloc_skb(IEEE802154_MTU);
	if (!skb) 
	{
		dev_err(&spi->dev, "%s: Failed to alloc skb\n", __func__);
		return;
	}

	memcpy(skb_put(skb, len), data, len);

	ieee802154_rx_irqsafe(hw, skb, 1);

	ieee802154_xmit_complete(hw, skb, false);
}

static int
fake_net_ieee_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct fake_net *priv = hw->priv;
	struct spi_device *spi = priv->spi;
	int ret;

	dev_dbg(&spi->dev, "%s\n", __func__);

	if (priv->tx_buf) {
		dev_err(&spi->dev, "%s: tx_buffer sbk is busy\n", __func__);
		ret = -EBUSY;
	} else {
		priv->tx_buf = skb;
		ret = 0;
		schedule_work(&priv->irqwork);
	}
	return ret;
}

static int
fake_net_ieee_start(struct ieee802154_hw *hw)
{
	struct fake_net *priv = hw->priv;
	struct spi_device *spi = priv->spi;

	dev_dbg(&spi->dev, "%s interface up\n", __func__);

	return 0;
}


static void
fake_net_ieee_stop(struct ieee802154_hw *hw)
{
	struct fake_net *priv = hw->priv;
	struct spi_device *spi = priv->spi;

	dev_dbg(&spi->dev, "%s interface down\n", __func__);
}

static int
fake_net_ieee_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	struct fake_net *priv = hw->priv;
	struct spi_device *spi = priv->spi;

	dev_dbg(&spi->dev, "%s\n", __func__);
	return 0;
}

static int
fake_net_ieee_ed(struct ieee802154_hw *hw, u8 *level)
{
	struct fake_net *priv = hw->priv;
	struct spi_device *spi = priv->spi;

	dev_dbg(&spi->dev, "%s\n", __func__);
	return 0;
}

static int
fake_net_ieee_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct fake_net *priv = hw->priv;
	struct spi_device *spi = priv->spi;

	dev_dbg(&spi->dev, "%s\n", __func__);

	return 0;
}

static int
fake_net_ieee_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	struct fake_net *priv = hw->priv;
	struct spi_device *spi = priv->spi;

	dev_dbg(&spi->dev, "%s\n", __func__);

	return 0;
}

static const struct ieee802154_ops fake_net_ops = {
	.owner = THIS_MODULE,
	.xmit_async = fake_net_ieee_xmit,
	.ed = fake_net_ieee_ed,
	.set_channel = fake_net_ieee_set_channel,
	.set_txpower = fake_net_ieee_set_txpower,
	.start = fake_net_ieee_start,
	.stop = fake_net_ieee_stop,
	.set_promiscuous_mode = fake_net_ieee_set_promiscuous_mode,
};

static void
fake_net_irqwork(struct work_struct *work)
{
	struct fake_net *priv;

	priv = container_of(work, struct fake_net, irqwork);
	fake_net_rx_complete(priv->hw);
}


static int fake_net_probe(struct spi_device *spi)
{
	struct ieee802154_hw *hw;
	struct fake_net *priv;
	int err;

	hw = ieee802154_alloc_hw(sizeof(*priv), &fake_net_ops);
	if (!hw) {
		dev_err(&spi->dev, "%s: not enough memory\n", __func__);
		return -ENOMEM;
	}

	err = ieee802154_register_hw(hw);
	if (err)
	{
		dev_err(&spi->dev,"%s: Register ieee802154 failed\n", __func__);
		return err;
	}
	priv = hw->priv;
	INIT_WORK(&priv->irqwork, fake_net_irqwork);
	priv->spi = spi;
	priv->hw  = hw;
	spi_set_drvdata(spi, priv);
	return 0;
}

static int fake_net_remove(struct spi_device *spi)
{
	struct fake_net *priv = spi_get_drvdata(spi);
	ieee802154_unregister_hw(priv->hw);
	ieee802154_free_hw(priv->hw);
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
