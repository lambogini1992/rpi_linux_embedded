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
	{ 0x92c44d06, "netif_rx" },
	{ 0x25fb18f6, "eth_type_trans" },
	{ 0x37a0cba, "kfree" },
	{ 0x19ec0cef, "skb_put" },
	{ 0xa55ecf54, "__netdev_alloc_skb" },
	{ 0xf4b72d54, "_dev_err" },
	{ 0x45e226be, "_dev_info" },
	{ 0x6a52380a, "register_netdev" },
	{ 0x92a3adb6, "alloc_netdev_mqs" },
	{ 0xd32c4e6b, "free_netdev" },
	{ 0x850c530b, "unregister_netdev" },
	{ 0x49ebacbd, "_clear_bit" },
	{ 0xb2d48a2e, "queue_work_on" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x2d6fcc06, "__kmalloc" },
	{ 0x526c3a6c, "jiffies" },
	{ 0x676bbc0f, "_set_bit" },
	{ 0x9d669763, "memcpy" },
	{ 0xdd655a18, "skb_push" },
	{ 0xc5850110, "printk" },
	{ 0x31d95123, "ether_setup" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("of:N*T*Cnordic,nrf24");
MODULE_ALIAS("of:N*T*Cnordic,nrf24C*");

MODULE_INFO(srcversion, "E1414710723C9C2A0BC6EB3");
