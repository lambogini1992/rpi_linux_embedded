#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
/*for character device*/
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
/*using for platform device*/
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/tty_ldisc.h>
#include <linux/tty_flip.h>
#include <linux/tty_driver.h>
#include <linux/serdev.h>


#define DRIVER_NAME "mcu_serial"
#define DEV_NAME    "serial_mcu"

#define N_MCU		29

MODULE_LICENSE("GPL");

static volatile uint32_t *gpio_base = NULL;
struct resource *res;
uint8_t set_level;

typedef struct _mcu_uart_
{
	int current_state;
	struct tty_struct	*tty;
	spinlock_t touch_lock;
   
}MCU_UART;

MCU_UART *mcu_uart;

int	mcu_serial_open(struct tty_struct *tty);
void mcu_serial_close(struct tty_struct *tty);
void mcu_serial_flush_buffer(struct tty_struct *tty);
ssize_t mcu_serial_read(struct tty_struct * tty, struct file * file, unsigned char * buf, size_t nr);
ssize_t mcu_serial_write(struct tty_struct * tty, struct file * file, const unsigned char * buf, size_t nr);
int	mcu_serial_ioctl(struct tty_struct * tty, struct file * file, unsigned int cmd, unsigned long arg);
int	mcu_serial_compat_ioctl(struct tty_struct * tty, struct file * file, unsigned int cmd, unsigned long arg);
void mcu_serial_set_termios(struct tty_struct *tty, struct ktermios * old);
int	mcu_serial_poll(struct tty_struct * tty, struct file * file, poll_table *wait);
void mcu_serial_receive_buf(struct tty_struct *tty, const unsigned char *cp, char *fp, int count);
void mcu_serial_write_wakeup(struct tty_struct *tty);
int mcu_serial_hangup(struct tty_struct *tty);
void mcu_serial_dcd_change(struct tty_struct *tty, unsigned int status);
int	mcu_serial_receive_buf2(struct tty_struct *tty, const unsigned char *cp, char *fp, int count);

struct tty_ldisc_ops mcu_serial_ldisc =
{
	.owner = THIS_MODULE,
	.magic = TTY_LDISC_MAGIC,
	.name  = "n_mcu",
	.num   = N_MCU,
	.open  = mcu_serial_open,
	.close = mcu_serial_close,
	.flush_buffer = mcu_serial_flush_buffer,
	.read = mcu_serial_read,
	.write = mcu_serial_write,
	.ioctl = mcu_serial_ioctl,
	.compat_ioctl = mcu_serial_compat_ioctl,
	.set_termios = mcu_serial_set_termios,
	.poll = mcu_serial_poll,
	.hangup = mcu_serial_hangup,
	.recieve_buf = mcu_serial_receive_buf,
	.write_wakeup = mcu_serial_write_wakeup,
	.dcd_change = mcu_serial_dcd_change,
	.recieve_buf2 = mcu_serial_receive_buf2,
};


/*Using for init device type*/
/*Start create device platform driver*/
static int device_probe(struct platform_device *pdev);
static int device_remove(struct platform_device *pdev);





static int device_probe(struct platform_device *pdev)
{
	int ret_val;
	const struct  of_device_id *match;
	int res_map_size;
	struct tty_driver *tty_drv;
	uint16_t br_speed;

	printk(KERN_INFO "Hello! This is UART MCU DEVICES\n");

	ret_val = 0;

	ret_val = tty_register_ldisc(N_MCU, &mcu_serial_ldisc);
	if(ret_val != 0)
	{
		return ret_val;
	}


	return ret_val;
}

static int device_remove(struct platform_device *pdev)
{
	printk("Goodbye MCU DEVICE SERIAL\n");

	tty_unregister_ldisc(N_MCU);

	return 0;
}

static const struct of_device_id test_uart_of_match[] =
{
	{.compatible = "uart, mcu_stm32", NULL},
	{},
};

MODULE_DEVICE_TABLE(of, test_uart_of_match);

static struct platform_driver mcu_uart_pldriver = {
	.probe 		= device_probe,
	.remove		= device_remove,
	.id_table   = test_uart_of_match,
	.driver  	= {
		.name 				= DRIVER_NAME,
		.of_match_table		= test_uart_of_match,
	},
};

/*Register device to system linux*/
module_platform_driver(mcu_uart_pldriver);

// static int __init initmodule(void)
// {
// 	printk(KERN_INFO ">>>>>>>>START DEVICE INIT<<<<<<<<<<\n");

// 	platform_driver_register(&mcu_uart_pldriver);

// 	return 0;
// }

// static void __exit exitmodule(void)
// {
// 	platform_driver_unregister(&mcu_uart_pldriver);

// 	printk(KERN_INFO ">>>>>>>>>>>>EXIT DEVICE<<<<<<<<<<<\n");
// 	return;
// }

// module_init(initmodule);
// module_exit(exitmodule);
