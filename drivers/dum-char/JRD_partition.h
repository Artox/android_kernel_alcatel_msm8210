#ifndef __JRD_PARTITION_H__
#define __JRD_PARTITION_H__

#include <linux/module.h>

/*typedef enum  {
	EMMC = 1,
	NAND = 2,
} dev_type;*/

/*typedef enum {
	USER = 0,
	BOOT_1,
	BOOT_2,
	RPMB,
	GP_1,
	GP_2,
	GP_3,
	GP_4,
} Region;*/

struct excel_info{
	char * name;
	unsigned long long size;
	unsigned long long start_address;
	//dev_type type ;
	//unsigned int partition_idx;
	//Region region;
};

#define PART_NUM              29

struct excel_info PartInfo[PART_NUM]={
			{"modem", 65536, 0x100000},	
			{"sbl1", 512, 0x4100000},	
			{"sdi", 128, 0x4180000},
			{"DDR", 32, 0x4200000},	
			{"aboot", 512, 0x4300000},	
			{"rpm", 500, 0x4380000},	
			{"boot", 10240, 0x4400000},	
			{"tz", 500, 0x4e00000},	
			{"pad", 1024, 0x4e7d000},	
			{"modemst1", 1536, 0x4f7d000},	
			{"modemst2", 1536, 0x50fd000},	
			{"fsg", 1536, 0x5300000},	
			{"fsc", 1, 0x5500000},	
			{"ssd", 8, 0x5500400},	
			{"tunning", 1536, 0x5502400},	
			{"traceability", 8, 0x5682400},	
			{"mobile_info", 8192, 0x5700000},	
			{"splash", 3072, 0x5f00000},	
			{"custpack", 716800, 0x6200000},	
			{"abootbk", 512, 0x31e00000},	
			{"rpmbk", 512, 0x31e80000},	
			{"tzbk", 512, 0x31f00000},	
			{"fota", 8, 0x31f80000},	
			{"misc", 1024, 0x31f82000},	
			{"cache", 149504, 0x32100000},	
			{"persist", 32768, 0x3b300000},	
			{"system", 563200, 0x3d300000},	
			{"recovery", 10240, 0x5f900000},	
			{"userdata",2244600, 0x60300000},
};
EXPORT_SYMBOL(PartInfo);
#endif
