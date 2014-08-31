#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>
#include <string.h>
#include <fcntl.h>

static const char *console_file_name = "/dev/omap_uart.0";
static pthread_t thread;
static pthread_mutex_t mutex; 
static char buf[100];
static int count;



static void *uart_write(void *arg)
{

	printf("Waiting for data: ");
	while(1) {
		if (pthread_mutex_trylock(&mutex)) {
			putchar('.');
			fflush(stdout);
			sleep(1);
		} else {
			buf[count] = 0;
			printf("\nData ready: %s\n", buf);
			printf("Waiting for data: ");
			pthread_mutex_unlock(&mutex);
			fflush(stdout);
			sleep(5);
		}
	}
}

static void uart_read()
{
	FILE *fdr, *fdw;
	int ret;

	pthread_mutex_init(&mutex, NULL);

	ret = pthread_create(&thread, NULL, uart_write, NULL);
	if (ret) {
		printf("thread creation failed %d\n", ret);
		return;
	}

	fdr = fopen(console_file_name, "r");
	fdw = fopen(console_file_name, "w");

	while (1) {
		pthread_mutex_lock(&mutex);
		count = fread(buf, 1, 10, fdr);
		fwrite(buf, 1, count, fdw);
		fflush(fdw);
		pthread_mutex_unlock(&mutex);
		sleep(2); /* Give some time for the writer to acquire the lock */
	}

	return;
}

#define MMAP_LEN 100
static void mmap_file(const char *fname)
{
	int fd;
	char sample[MMAP_LEN];
	char *map;

	printf("a\n");
	fd = open(fname, O_RDONLY);	
	printf("a %x\n", fd);
	map = mmap(NULL, MMAP_LEN, PROT_READ, MAP_SHARED, fd, 0);
	printf("a %x\n", map);
	memcpy(sample, map, MMAP_LEN);
	printf("a\n");
	sample[MMAP_LEN - 1] = 0;
	printf("a\n");
	while(1);
	printf("data: %s\n", sample);
	printf("a\n");
	munmap(map, MMAP_LEN);
	printf("a\n");
//	close(fd);
	printf("a\n");

}

int main(void)
{
	printf("hello world\n");
	mmap_file("/dev/omap_uart.0");

	return 0;
}