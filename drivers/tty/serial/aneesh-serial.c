#include <linux/module.h>
#include <linux/platform_device.h>

static int aneesh_ser_debug = 0;
module_param(aneesh_ser_debug, int, 0644);
MODULE_PARM_DESC(aneesh_ser_debug, "Enable module debug");

static int aneesh_serial_probe(struct platform_device *pdev)
{
	return -1;
}

static int aneesh_serial_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver aneesh_ser_drvr = {
	.probe = aneesh_serial_probe,
	.remove = aneesh_serial_remove,
	.driver = {
		.name = "omap_uart",
	}
};

static int __init aneesh_serial_init(void)
{
	int ret = platform_driver_register(&aneesh_ser_drvr);
	printk("hello world aneesh serial : %d %d\n", aneesh_ser_debug, ret);

	return ret;
}

static void __exit aneesh_serial_exit(void)
{
	printk("aneesh serial exit\n");

	platform_driver_unregister(&aneesh_ser_drvr);
}

module_init(aneesh_serial_init);
module_exit(aneesh_serial_exit);

MODULE_AUTHOR("Aneesh V");
MODULE_DESCRIPTION("Aneesh Serial");
MODULE_LICENSE("GPL");
