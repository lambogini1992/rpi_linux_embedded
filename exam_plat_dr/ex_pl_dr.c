#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

//for platform drivers....
#include <linux/of.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "Sample_Pldrv"

MODULE_LICENSE("GPL");

/***************/
static const struct of_device_id example_device_table[]=
{
	{ .compatible = "test,simple-misc-dev", },
	{0},
};

MODULE_DEVICE_TABLE(of,example_device_table);

/**************/ 
static int sample_drv_probe(struct platform_device *pdev)
{
	printk("\nInitialize Driver\n");
	return 0;
}
static int sample_drv_remove(struct platform_device *pdev)
{
	printk("\nRemove driver\n");
	return 0;
}

static struct platform_driver sample_pldriver = {
    .probe          = sample_drv_probe,
    .remove         = sample_drv_remove,
    .driver = {
            .name  = DRIVER_NAME,
            .owner = THIS_MODULE,
            .of_match_table = of_match_ptr(example_device_table),
    },
};
/**************/  

static int __init ourinitmodule(void)
{
    printk(KERN_ALERT "\n Welcome to sample Platform driver.... \n");

    /* Registering with Kernel */
    platform_driver_register(&sample_pldriver);

    return 0;
}

static void __exit ourcleanupmodule(void)
{
    printk(KERN_ALERT "\n Thanks....Exiting sample Platform driver... \n");

    /* Unregistering from Kernel */
    platform_driver_unregister(&sample_pldriver);

    return;
}

module_init(ourinitmodule);
module_exit(ourcleanupmodule);