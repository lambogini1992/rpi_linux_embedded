#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
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
__used
__attribute__((section("__versions"))) = {
	{ 0xa2263b00, "module_layout" },
	{ 0x5238c695, "platform_driver_unregister" },
	{ 0x62f5ed99, "__platform_driver_register" },
	{ 0x5f754e5a, "memset" },
	{ 0x37a0cba, "kfree" },
	{ 0xc358aaf8, "snprintf" },
	{ 0xae353d77, "arm_copy_from_user" },
	{ 0x2d6fcc06, "__kmalloc" },
	{ 0x6ac9ab92, "cdev_add" },
	{ 0xaed2c5f, "cdev_init" },
	{ 0x82383968, "cdev_alloc" },
	{ 0xf9bed157, "device_create" },
	{ 0xe32a90d5, "__class_create" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0xe97c4103, "ioremap" },
	{ 0x17c97488, "platform_get_resource" },
	{ 0x7a7d3510, "of_match_device" },
	{ 0xedc03953, "iounmap" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0x1fc05a00, "class_destroy" },
	{ 0x118cacb3, "device_destroy" },
	{ 0xc1538bc6, "cdev_del" },
	{ 0xc5850110, "printk" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("of:N*T*Cbcm,_dev_serial");
MODULE_ALIAS("of:N*T*Cbcm,_dev_serialC*");

MODULE_INFO(srcversion, "4775D207A11C9F5B042B94B");
