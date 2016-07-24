/*
 * printd
 */

#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/string.h>

#define PRINTD_WQ "printd_TCL"

#define PRINTD_PART "/dev/block/mmcblk0"
#define PRINTD_PART_POS 0x31f82000      //misc
#define PRINTD_PART_LEN 0x100000         //1024k 
#define PRINTD_PART_OFFSET 1024*512

#define PRINTD_KN_OFFSET PRINTD_PART_OFFSET

#if 0
#define _printk(fmt, args...) printk(fmt, ##args)
#else
#define _printk(fmt, args...)
#endif

#define L_SHIFT 2
#define __LOG_BUF_LEN	(1 << (CONFIG_LOG_BUF_SHIFT + L_SHIFT))

static char __log_buf[__LOG_BUF_LEN + 4];
static char *log_buf = __log_buf;
static int log_buf_len = __LOG_BUF_LEN;
static int log_buf_pos = 0;
static int log_buf_out = 0;
static int part_pos = PRINTD_KN_OFFSET;

static struct workqueue_struct *print_wq;
static struct work_struct print_work;
static int count = 0;
static int game_over = 0;
ssize_t res;

void printd_dump (char *buf, int len)
{
	if (log_buf_pos + len < log_buf_len) {
		memcpy(log_buf + log_buf_pos, buf, len);
		log_buf_pos += len;
	} 
       
      else {		              
		log_buf_pos = 0;
              log_buf_out = 0;
              memcpy(log_buf + log_buf_pos, buf, len);
		log_buf_pos += len;
	     }
}
EXPORT_SYMBOL(printd_dump);

static void output(int stage)
{
	struct file *filp;
	ssize_t res;
	size_t len;
	loff_t pos;
	const char *buf;
	mm_segment_t old_fs;

	_printk("**********this is: %s**********game=%d*********\n", __func__,game_over);

    if(stage) {         // system finish booting,end log storing
        game_over = 1;
        destroy_workqueue(print_wq);
        return;
    }   

	if (game_over)
		return;

	buf = log_buf + log_buf_out;
	pos = PRINTD_PART_POS + part_pos;

    len = log_buf_pos - log_buf_out;
    if (part_pos + len > PRINTD_PART_LEN){         // space is nearly full, only store length of left space
        game_over = 1;
        len = PRINTD_PART_LEN - part_pos;
		destroy_workqueue(print_wq);
        }
	
	filp = filp_open(PRINTD_PART, O_RDWR, 0);
	if (IS_ERR(filp)) {
		_printk("%s filp_open err: %ld\n", PRINTD_PART, PTR_ERR(filp));
		return;
	}

	old_fs = get_fs();
	set_fs(get_ds());
	// set_fs(KERNEL_DS);

	res = vfs_write(filp, buf, len, &pos);
	set_fs(old_fs);
	if (res == len) {
		_printk("write suc: %s\n", __func__);
		log_buf_out += len;
		part_pos += len;
	} else
		_printk("write err: %s\n", __func__);

	filp_close(filp, NULL);
}

#define PRINTD_COUNT 5000
static void printd_func(struct work_struct *work)
{
	queue_work(print_wq, &print_work);

	count ++;
	if (count == PRINTD_COUNT) {
		count = 0;
		if (system_state == SYSTEM_BOOTING)
			output(0);
		else if (system_state == SYSTEM_RUNNING)
			output(1);
	}
}

static int __init printd_init(void)
{
	int ret = 0;

	_printk("start: %s\n", __func__);

	print_wq = create_singlethread_workqueue(PRINTD_WQ);
	if (!print_wq) {
		return -ENOMEM;
	}

	INIT_WORK(&print_work, printd_func);
	queue_work(print_wq, &print_work);

	return ret;
}

static void __exit printd_exit(void)
{
	_printk("stop: %s\n", __func__);
	if (print_wq) {
		destroy_workqueue(print_wq);
	}
}

module_init(printd_init);
module_exit(printd_exit);
MODULE_LICENSE("GPL");
