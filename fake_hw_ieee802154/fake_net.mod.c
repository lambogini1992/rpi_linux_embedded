#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(.gnu.linkonce.this_module) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section(__versions) = {
	{ 0xe7cc3b88, "module_layout" },
	{ 0xe875a2f8, "__spi_register_driver" },
	{ 0x8d654576, "driver_unregister" },
	{ 0xc34e5e6a, "ieee802154_register_hw" },
	{ 0xdc11984b, "ieee802154_alloc_hw" },
	{ 0xb2d48a2e, "queue_work_on" },
	{ 0x2d3385d3, "system_wq" },
	{ 0xf4b72d54, "_dev_err" },
	{ 0x58e5be3, "ieee802154_xmit_complete" },
	{ 0xa6de5144, "ieee802154_rx_irqsafe" },
	{ 0x9d669763, "memcpy" },
	{ 0x19ec0cef, "skb_put" },
	{ 0xa55ecf54, "__netdev_alloc_skb" },
	{ 0x43514056, "ieee802154_free_hw" },
	{ 0x2e424d37, "ieee802154_unregister_hw" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

MODULE_INFO(depends, "mac802154");

MODULE_ALIAS("of:N*T*Cnordic,nrf24");
MODULE_ALIAS("of:N*T*Cnordic,nrf24C*");

MODULE_INFO(srcversion, "77BB17FC62ADA627BA71F77");
