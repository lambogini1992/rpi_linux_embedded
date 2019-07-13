#include <linux/module.h> /* thu vien nay dinh nghia cac macro nhu module_init va module_exit */
#include <linux/fs.h>
#include "gpio_reg.h"

#define DRIVER_AUTHOR "Pham Tuan Anh"
#define DRIVER_DESC   "A gpio character device driver"
#define DRIVER_VERSION "0.1"

#define SET_INPUT_PIN   0x00
#define SET_OUTPUT_PIN  0x01
#define SET_AL0_PIN     0x04
#define SET_AL1_PIN     0x05
#define SET_AL2_PIN		0x06
#define SET_AL3_PIN		0x07
#define SET_AL4_PIN		0x03
#define SET_AL5_PIN		0x02


/****************************** device specific - START *****************************/
/* ham khoi tao thiet bi */

/* ham giai phong thiet bi */

/* ham doc tu cac thanh ghi du lieu cua thiet bi */

/* ham ghi vao cac thanh ghi du lieu cua thiet bi */

/* ham doc tu cac thanh ghi trang thai cua thiet bi */

/* ham ghi vao cac thanh ghi dieu khien cua thiet bi */

/* ham xu ly tin hieu ngat gui tu thiet bi */

/******************************* device specific - END *****************************/

/******************************** OS specific - START *******************************/
/* cac ham entry points */
static int gpio_set_reg_map(void)
{
	int ret_val;

	ret_val = ioremap_nocache(GPIO_BASE_PERIPHERAL, )
}
/* ham khoi tao driver */
static int __init vchar_driver_init(void)
{
	/* cap phat device number */

	/* tao device file */

	/* cap phat bo nho cho cac cau truc du lieu cua driver va khoi tao */

	/* khoi tao thiet bi vat ly */

	/* dang ky cac entry point voi kernel */

	/* dang ky ham xu ly ngat */

	printk("Initialize vchar driver successfully\n");
	return 0;
}

/* ham ket thuc driver */
static void __exit vchar_driver_exit(void)
{
	/* huy dang ky xu ly ngat */

	/* huy dang ky entry point voi kernel */

	/* giai phong thiet bi vat ly */

	/* giai phong bo nho da cap phat cau truc du lieu cua driver */

	/* xoa bo device file */

	/* giai phong device number */

	printk("Exit vchar driver\n");
}
/********************************* OS specific - END ********************************/

module_init(vchar_driver_init);
module_exit(vchar_driver_exit);

MODULE_LICENSE("GPL"); /* giay phep su dung cua module */
MODULE_AUTHOR(DRIVER_AUTHOR); /* tac gia cua module */
MODULE_DESCRIPTION(DRIVER_DESC); /* mo ta chuc nang cua module */
MODULE_VERSION(DRIVER_VERSION); /* mo ta phien ban cuar module */
MODULE_SUPPORTED_DEVICE("testdevice"); /* kieu device ma module ho tro */