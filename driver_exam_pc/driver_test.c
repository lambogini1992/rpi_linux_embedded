#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

static int my_pbdr_probe(struct platform_device *pdev)
{
	pr_info("Hello! device probe\n");
	return 0;
}

static int my_pb_remove(struct platform_device *pdev)
{
	pr_info("Goodbye! driver probe\n");
	return 0;
}

static struct platform_driver my_dr =
{
	.probe 		= my_pbdr_probe,
	.remove 	= my_pb_remove,
	.driver 	=
	{
			.name	= "example driver",
			.owner	= THIS_MODULE,
	}, 
};

static int __init my_drv_init(void)
{
	platform_driver_register(&my_dr);
	pr_info("Hallu everybody\n");
	return 0;
}

static void __exit my_drv_remove(void)
{
	platform_driver_unregister(&my_dr);
	pr_info("Goodbye everbody\n");
}

module_init(my_drv_init);
module_exit(my_drv_remove);

MODULE_AUTHOR("PTA");
MODULE_DESCRIPTION("Platform device driver using char device example");
MODULE_LICENSE("GPL");
