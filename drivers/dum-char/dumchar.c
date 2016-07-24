#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... filp_open*/
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>  /*proc*/
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/aio.h>
#include <asm/uaccess.h>   /*set_fs get_fs mm_segment_t*/
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/unistd.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mtd/mtd.h>
//#include <linux/autoconf.h>
//#include <linux/sched.h>	//show_stack(current,NULL)
//#include <mach/env.h>

//#include "partition_define.h"
//#include "dumchar.h"		/* local definitions */
//#include "pmt.h"
//#include <linux/mmc/host.h>
//#include "mt_sd.h"
//#include <linux/genhd.h>
//#include "env.h"
#include <linux/platform_device.h>
#include "JRD_partition.h"


static struct proc_dir_entry *dumchar_proc_entry=NULL;

static int dumchar_proc_read(char *page, char **start, off_t off,int count, int *eof, void *data)
{
	int i,len=0;

	len += sprintf(page+len, "Part_Name\tSize\tStartAddr\n");
	
	for(i=0;i<PART_NUM;i++){


		  	len += sprintf(page+len, "%-10s   0x%016lld   0x%016llx \n",
		    PartInfo[i].name,
		    PartInfo[i].size,
		    PartInfo[i].start_address);

	}


	len += sprintf(page+len, "Part_Name:Partition name you should open;\nSize:size of partition\nStartAddr:Start Address of partition;\n");

	return len;
}

int dumchar_probe(struct platform_device * dev)
{
	
	return 0;
}

static struct platform_device dumchar_pdevice = {
		.name = "dummy_char",

};


static struct platform_driver dumchar_driver = {
		.probe	= dumchar_probe,
		//.remove  	= dumchar_remove,
		.driver  = {
			.name  = "dummy_char",
			.owner = THIS_MODULE,
		},
};

static int __init dumchar_init(void)
{
	int result;
	printk("dumchar_int\n");
	platform_device_register(&dumchar_pdevice);
	dumchar_proc_entry = create_proc_entry("dumchar_info", S_IFREG|S_IRUGO, NULL);
	if (dumchar_proc_entry){
		dumchar_proc_entry->read_proc = dumchar_proc_read;
		printk( "dumchar: register /proc/dumchar_info success %p\n",dumchar_proc_entry->read_proc);
	}
	else{
		printk( "dumchar: unable to register /proc/dumchar_info\n");
		result = -ENOMEM;
		goto fail_create_proc;
	}
	result= platform_driver_register(&dumchar_driver);
	if (result) {
        	printk("DUMCHAR: Can't register driver\n");
        	goto fail_driver_register;
    	}

	printk( "DumChar: init USIF  Done!\n");
	
	return 0;
fail_driver_register:
	remove_proc_entry("dumchar_info", NULL);
fail_create_proc:
	return result;
}


static void __exit dumchar_cleanup(void)
{
	remove_proc_entry("dumchar_info", NULL);
	platform_driver_unregister(&dumchar_driver);
		
}


module_init(dumchar_init);
module_exit(dumchar_cleanup);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek Dummy Char Device Driver");
MODULE_AUTHOR("Kai Zhu <kai.zhu@mediatek.com>");

