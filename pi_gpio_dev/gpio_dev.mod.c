#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

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

#ifdef RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xe0f826f1, "module_layout" },
	{ 0xe4e6476, "cdev_del" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0x32ba2c9f, "class_destroy" },
	{ 0xd89618ae, "device_destroy" },
	{ 0xec2a9a6c, "cdev_add" },
	{ 0xd83a371f, "cdev_init" },
	{ 0x36f921af, "cdev_alloc" },
	{ 0x9957615a, "device_create" },
	{ 0x7415e2df, "__class_create" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0x5f754e5a, "memset" },
	{ 0xe2d5255a, "strcmp" },
	{ 0x28cc25db, "arm_copy_from_user" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0xf7802486, "__aeabi_uidivmod" },
	{ 0xe707d823, "__aeabi_uidiv" },
	{ 0xe97c4103, "ioremap" },
	{ 0x6f09dff2, "kmalloc_caches" },
	{ 0xf4fa543b, "arm_copy_to_user" },
	{ 0x37a0cba, "kfree" },
	{ 0xb81960ca, "snprintf" },
	{ 0xa08e0620, "kmem_cache_alloc_trace" },
	{ 0x2e5810c6, "__aeabi_unwind_cpp_pr1" },
	{ 0xedc03953, "iounmap" },
	{ 0x27e1a049, "printk" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "D9CF48AA9D9AB5DC0838DF8");
