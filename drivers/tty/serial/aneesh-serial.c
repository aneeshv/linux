#include <linux/module.h>

static int aneesh_ser_debug = 0;
module_param(aneesh_ser_debug, int, 0644);
MODULE_PARM_DESC(aneesh_ser_debug, "Enable module debug");

static int __init aneesh_serial_init(void)
{
	printk("hello world aneesh serial : %d\n", aneesh_ser_debug);

	return 0;
}

static void __exit aneesh_serial_exit(void)
{
	printk("aneesh serial exit\n");
}

module_init(aneesh_serial_init);
module_exit(aneesh_serial_exit);

MODULE_AUTHOR("Aneesh V");
MODULE_DESCRIPTION("Aneesh Serial");
MODULE_LICENSE("GPL");
