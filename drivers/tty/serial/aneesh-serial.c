#include <linux/module.h>

static int __init aneesh_serial_init(void)
{
	printk("hello world aneesh serial - %d\n", module_param);

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
