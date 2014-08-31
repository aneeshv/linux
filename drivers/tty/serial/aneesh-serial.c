#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <linux/highmem.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>

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
#define DRV_NAME	"omap_uart"
#define RX_BUF_SIZE	4096
#define wraparound_index(p, max) ((p) >= (max) ? (p) - (max) : (p))


/*
 * Global variables
 */
static int aneesh_ser_debug = 0;
module_param(aneesh_ser_debug, int, 0644);
MODULE_PARM_DESC(aneesh_ser_debug, "Enable module debug");
static int aneesh_ser_major;
static LIST_HEAD(dev_list);
static DECLARE_WAIT_QUEUE_HEAD(reader_q);
static DECLARE_WAIT_QUEUE_HEAD(writer_q);
static struct page *pages;

struct circ_buf_t {
	char			*buf;
	u32			rp;
	u32			wp;
};

struct uart_t {
	unsigned int	id;
	void __iomem	*base;
	struct device	*dev;
	struct list_head	node;
	struct circ_buf_t	circ_buf;
	struct mutex		dev_lock;
};


static void circ_buf_init(struct circ_buf_t *buf)
{
	buf->rp = 0;
	buf->wp = 0;
}

static bool circ_buf_full(struct circ_buf_t *buf)
{
	if (wraparound_index(buf->wp + 1, RX_BUF_SIZE) == buf->rp)
		return true;
	else
		return false;
}

static bool circ_buf_empty(struct circ_buf_t *buf)
{
	if (buf->rp == buf->wp)
		return true;
	else
		return false;
}

static int circ_buf_space(struct circ_buf_t *buf)
{
	if (buf->rp > buf->wp)
		return buf->rp - buf->wp - 1;
	else
		/* return only contiguous space */
		return RX_BUF_SIZE - buf->wp;
}

static void circ_buf_update_wp(struct circ_buf_t *buf, int len)
{
	buf->wp = wraparound_index(buf->wp + len, RX_BUF_SIZE);
}

static int do_copy_to_user(char *to, const char *from, int sz, int max)
{
	int reject;

	sz = sz > max ? max : sz;
	reject = copy_to_user(to, from, sz);

	/* copy_to_user() returns number of bytes that could NOT be copied */
	return sz - reject;
}


static int circ_buf_copy_user(struct circ_buf_t *cbuf, char *user_buf, int count)
{
	int sz, copied;

	if (cbuf->wp == cbuf->rp)
		return 0;
	else if (cbuf->wp > cbuf->rp) {
		sz = cbuf->wp - cbuf->rp;
		copied = do_copy_to_user(user_buf, &cbuf->buf[cbuf->rp], sz, count);
	} else { /* wp < rp*/
		/* The first contiguous part */
		sz = RX_BUF_SIZE - cbuf->rp;
		copied = do_copy_to_user(user_buf, &cbuf->buf[cbuf->rp], sz, count);

		/* Do the second contiguous part only if the first one was successful */
		if (copied == sz) {
			sz = cbuf->wp; /* wp - 0 */
			copied += do_copy_to_user(&user_buf[copied], &cbuf->buf[0],
				sz, count);
		}
	}

	cbuf->rp = wraparound_index(cbuf->rp + copied, RX_BUF_SIZE);

	return copied;
}

static void dump_regs(void __iomem *base)
{
	int offset;

	printk("Register dump:\n");
	for (offset = 0; offset < 0x85; offset += 4)
		printk("0x%08x:\t0x%04x\n", (int)(base + offset), readw(base + offset));
}

static void uart_send_buf(struct uart_t *uart, const char *buf, int len)
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

	printk("%s: IIR 0x%04x\n",__func__, readw(uart->base + 8));

	/* Disable UART interrupts temporarily */
	writew(0, uart->base + UART_IER);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t aneesh_serial_threaded_isr(int irq, void *dev_id)
{
	struct uart_t	*uart = (struct uart_t *)dev_id;
	u32		len, space;
	struct circ_buf_t *buf = &uart->circ_buf;


	/* Do until FIFO is empty */
	while (!rx_fifo_empty(uart->base)) {
		printk("%s: before wait\n",__func__);
		wait_event(writer_q, !circ_buf_full(buf));

		space = circ_buf_space(buf);
		printk("%s: space = %d\n",__func__, space);

		/*
		 * circ_buf_space() will return only contiguous space,
		 * so need to worry about while calling uart_empty_rx_fifo()
		 */
		len = uart_empty_rx_fifo(uart, 
			&buf->buf[buf->wp], space);
		printk("%s: read %d chars\n",__func__, len);

		/* Update the write pointer */
		circ_buf_update_wp(buf, len);

		/* Release any blocking reader */
		wake_up(&reader_q);
	}

	printk("%s\n", __func__);

	/* Enable RHR interrupt back again */
	writew(UART_IER_RHRIT_EN, uart->base + UART_IER);

	return IRQ_HANDLED;
}

static int uart_chrdev_open(struct inode *inode, struct file *file)
{
	struct uart_t *uart;
	unsigned int minor;

	printk("%s\n");

	minor = iminor(inode);
	list_for_each_entry(uart, &dev_list, node) {
		if (minor == uart->id) {
			printk("uart found %x\n", uart->circ_buf.buf);
			file->private_data = uart;
			return 0;
		}
	}
	
	return -ENODEV;
}

static ssize_t uart_chrdev_read(struct file *file, char *user_buf, size_t count,
		                loff_t *ptr)
{
	struct uart_t 		*uart;
	struct circ_buf_t 	*buf;
	int			copied;

	if (file->private_data)
		uart = file->private_data;
	else
		return 0;

	buf = &uart->circ_buf;

	if ((file->f_flags & O_NONBLOCK) && circ_buf_empty(buf))
		return -EAGAIN;

	printk("%s: before cs\n",__func__);
	/* Serialize the user requests */
	mutex_lock_interruptible(&uart->dev_lock);

	printk("%s: before wait\n",__func__);
	wait_event_interruptible(reader_q, !circ_buf_empty(buf));
	printk("%s: after wait\n",__func__);
	
	copied = circ_buf_copy_user(buf, user_buf, count);

	wake_up(&writer_q);

	mutex_unlock(&uart->dev_lock);
	printk("%s: after cs\n",__func__);

	return copied;
}

static ssize_t uart_chrdev_write(struct file *file, const char *buf,
		                size_t count, loff_t * ppos)
{
	struct uart_t 		*uart;

	printk("%s\n", __func__);

	if (file->private_data)
		uart = file->private_data;
	else
		return 0;

	uart_send_buf(uart, buf, count);

	return count;
}

int aneesh_serial_mmap(struct file *file, struct vm_area_struct *vma)
{

	unsigned long	off, physical, vsize, psize, new_phys;
	struct uart_t	*uart;
	int ret;

	off = vma->vm_pgoff << PAGE_SHIFT;
	vsize = vma->vm_end - vma->vm_start;
	psize = 4096 - off;

	printk("%s\n", __func__);
	if (file->private_data)
		uart = file->private_data;
	else
		return -1;

	physical = virt_to_phys(uart->circ_buf.buf);
	physical += off;

	printk("physical %x\n", physical);


//	if (vsize > psize)
//	    return -EINVAL; /*  spans too high */
	//flush_dcache_page(pages);
	ret = remap_pfn_range(vma, vma->vm_start, physical >> PAGE_SHIFT, vsize, vma->vm_page_prot);
	//flush_tlb_all();
	new_phys = virt_to_phys(vma->vm_start);
	printk("physical %x\n", new_phys);
	printk("%s %x %x %x %x %d %x\n", __func__, vma->vm_start, vma->vm_end, off, psize, uart->circ_buf.buf, ret);
	
	return ret;	
}

static const struct file_operations uart_chrdev_fops = {
	.open	= uart_chrdev_open,
	.write = uart_chrdev_write,
	.read = uart_chrdev_read,
	.owner = THIS_MODULE,
	.llseek = noop_llseek,
	.mmap = aneesh_serial_mmap,
};

static struct class *aneesh_ser_class;

static int aneesh_serial_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *mem, *irq;
	struct uart_t *uart;
	void __iomem *base;
	struct page *page;

	/* support only uart0 */
	if (!pdev || (pdev->id != 0))
		return -1;

	uart = devm_kzalloc(&pdev->dev,
		sizeof(*uart), GFP_KERNEL);
	uart->dev = &pdev->dev;

	mutex_init(&uart->dev_lock);

	/* hardcoded to one page - OK for now */
	page = alloc_page(GFP_KERNEL);
	printk("%s: uart->circ_buf.buf %x %x\n",__func__, uart->circ_buf.buf, page_address(page));
	uart->circ_buf.buf = kmap(page);
	printk("%s: uart->circ_buf.buf %x %x\n",__func__, uart->circ_buf.buf, page_address(page));
	circ_buf_init(&uart->circ_buf);
	memcpy(uart->circ_buf.buf, "aneesh", 7);
	uart->circ_buf.buf[7] = 0;
	printk("%s: test memcpy : %s\n", __func__, uart->circ_buf.buf);
	list_add(&uart->node, &dev_list);
	pages = page;

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

	//writew(0, base + UART_IER); /* !!!!!!!!!!!!!!!!!!!!!!! TEST CODE !!!!!!!!!!!!!!!!!! */

	if (request_threaded_irq(irq->start, aneesh_serial_isr,
		aneesh_serial_threaded_isr, 0, dev_name(uart->dev), uart)) {
			printk("aneesh-serial: failed hooking ISR\n");
			return -1;
	}

	/*
	 * Enable interrupts - this is all the UART initialization
	 * we have to do
	 */
	//writew(0, base + UART_IER); /* !!!!!!!!!!!!!!!!!!!!!!! TEST CODE !!!!!!!!!!!!!!!!!! */
	writew(UART_IER_RHRIT_EN, base + UART_IER);

	/* Just for debug */
	//dump_regs(uart->base);

	/* Register character device */

#if 0
	ret = device_create(aneesh_ser_class, NULL,
		MKDEV(aneesh_ser_major, pdev->id),
		NULL, "%s.%d", "omap_uart", pdev->id);
#else
	ret = device_create(aneesh_ser_class, NULL,
		MKDEV(aneesh_ser_major, pdev->id),
		NULL, "my_uart.0");
#endif

	printk("%s: ret = %d\n", __func__, ret);

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
		.name = DRV_NAME,
	}
};

static int __init aneesh_serial_init(void)
{
	int ret;
	
	aneesh_ser_class = class_create(THIS_MODULE, DRV_NAME);

	if (IS_ERR(aneesh_ser_class))
		printk("%s: class creation failed \n", __func__);

	/*
	 * Register character device operations or
	 * we could say a character device driver
	 */
	aneesh_ser_major = register_chrdev(0, DRV_NAME, &uart_chrdev_fops); 

	ret = platform_driver_register(&aneesh_ser_drvr);
	printk("hello world aneesh serial : %d %d %d %x\n", aneesh_ser_debug, ret,
		aneesh_ser_major, (unsigned int) aneesh_ser_class);

	return 0;
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
