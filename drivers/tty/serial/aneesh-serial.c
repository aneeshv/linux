#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/interrupt.h>

/*
 * UART module defines
 */
#define UART_RHR	0x0
#define UART_THR	0x0
#define UART_IER	0X4
#define UART_SSR	0x44
#define UART_LSR	0x14

#define UART_SSR_TX_FIFO_FULL_MSK	1

#define	UART_LSR_RX_FIFO_E_MSK		1

#define UART_IER_RHRIT_EN		1

#define tx_fifo_full(base)	(readw((base) + UART_SSR) & UART_SSR_TX_FIFO_FULL_MSK)
#define rx_fifo_empty(base)	(!(readw((base) + UART_LSR) & UART_LSR_RX_FIFO_E_MSK))

/*
 * Other defines
 */
#define MAX_RX_BUF_LEN	64

/*
 * Global variables
 */
static int aneesh_ser_debug = 0;
module_param(aneesh_ser_debug, int, 0644);
MODULE_PARM_DESC(aneesh_ser_debug, "Enable module debug");

struct uart_t {
	void __iomem	*base;
	struct device	*dev;
	uint32_t	rx_buf_len;
	char		rx_buf[MAX_RX_BUF_LEN];
	spinlock_t	rx_lock;
};

static void dump_regs(void __iomem *base)
{
	int offset;

	printk("Register dump:\n");
	for (offset = 0; offset < 0x85; offset += 4)
		printk("0x%08x:\t0x%04x\n", (int)(base + offset), readw(base + offset));
}

void uart_send_buf(struct uart_t *uart, const char *buf, int len)
{
	int i;

	for(i = 0; i < len; i++) {
		/* wait while FIFO is full */
		while(tx_fifo_full(uart->base));

		/* Write one char */
		writeb(buf[i], uart->base + UART_THR);
	}
}

static int uart_empty_rx_fifo(struct uart_t *uart, char *buf, int max)
{
	int i = 0;

	if (rx_fifo_empty(uart->base))
		return 0;

	while ((i < max) && !rx_fifo_empty(uart->base))
		buf[i++] = readb(uart->base + UART_RHR);

	return i;
}

static irqreturn_t aneesh_serial_isr(int irq, void *dev_id)
{
	struct uart_t *uart = (struct uart_t *)dev_id;

	printk("aneesh_serial_isr: IIR 0x%04x\n", readw(uart->base + 8));

	/* I know! This is not nice. But this is just a test driver */
	while(uart->rx_buf_len);

	spin_lock(&uart->rx_lock);
	uart->rx_buf_len = uart_empty_rx_fifo(uart, uart->rx_buf,
		MAX_RX_BUF_LEN);
	spin_unlock(&uart->rx_lock);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t aneesh_serial_threaded_isr(int irq, void *dev_id)
{
	struct uart_t *uart = (struct uart_t *)dev_id;
	unsigned long irqstate;

	uart_send_buf(uart, uart->rx_buf, uart->rx_buf_len);
	spin_lock_irqsave(&uart->rx_lock, irqstate);
	uart->rx_buf_len = 0;
	spin_unlock_irqrestore(&uart->rx_lock, irqstate);

	printk("%s\n", __func__);

	return IRQ_HANDLED;
}

static int aneesh_serial_probe(struct platform_device *pdev)
{
	struct resource *mem, *irq;
	struct uart_t *uart;
	void __iomem *base;

	/* support only uart0 */
	if (!pdev || (pdev->id != 0))
		return -1;

	uart = devm_kzalloc(&pdev->dev,
		sizeof(*uart), GFP_KERNEL);
	uart->dev = &pdev->dev;
	spin_lock_init(&uart->rx_lock);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!devm_request_mem_region(&pdev->dev, mem->start,
		resource_size(mem), pdev->dev.driver->name)) {
		dev_err(&pdev->dev, "%s: memory region already claimed\n",
			__func__);
		return -EBUSY;
	}

	uart->base = devm_ioremap(&pdev->dev, mem->start,
		resource_size(mem));

	printk("aneesh-serial: pdev->id %d res->start 0x%08x uart->base 0x%08x\n",
		pdev->id, (u32)mem->start, (u32)uart->base);

	base = uart->base;

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (request_threaded_irq(irq->start, aneesh_serial_isr,
		aneesh_serial_threaded_isr, 0, dev_name(uart->dev), uart)) {
			printk("aneesh-serial: failed hooking ISR\n");
			return -1;
	}

	/*
	 * Enable interrupts - this is all the UART initialization
	 * we have to do
	 */
	writew(UART_IER_RHRIT_EN, base + UART_IER);

	/* Just for debug */
	dump_regs(uart->base);

	return 0;
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
