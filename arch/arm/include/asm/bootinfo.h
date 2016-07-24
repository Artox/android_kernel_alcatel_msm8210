/*
 bootinfo.h
 add by jch for watchdog ramdump
 */

#ifndef __ASMARM_BOOTINFO_H
#define __ASMARM_BOOTINFO_H

#define CONFIG_BOOTINFO 1
#if !defined(__KERNEL__) || defined(CONFIG_BOOTINFO)

/*
 * These #defines are used for the bits in powerup_reason.
 */
#define PU_REASON_USB_CABLE		1 
#define PU_REASON_FACTORY_CABLE		2
#define PU_REASON_PWR_KEY_PRESS		8
#define PU_REASON_CHARGER		       16
#define PU_REASON_POWER_CUT		32
#define PU_REASON_SW_AP_RESET		64
#define PU_REASON_WDOG_AP_RESET		128
#define PU_REASON_AP_KERNEL_PANIC	256
#define PU_REASON_INVALID		0xFFFFFFFF


/*
 * These #defines are used for the battery status at boot.
 * When no battery is present, the status is BATTERY_LO_VOLTAGE.
 */
#define BATTERY_GOOD_VOLTAGE    1
#define BATTERY_LO_VOLTAGE      2
#define BATTERY_UNKNOWN         (-1)

/*
 * /proc/bootinfo has a strict format.  Each line contains a name/value
 * pair which are separated with a colon and a single space on both
 * sides of the colon.  The following defines help you size the
 * buffers used to read the data from /proc/bootinfo.
 *
 * BOOTINFO_MAX_NAME_LEN:  maximum size in bytes of a name in the
 *                         bootinfo line.  Don't forget to add space
 *                         for the NUL if you need it.
 * BOOTINFO_MAX_VAL_LEN:   maximum size in bytes of a value in the
 *                         bootinfo line.  Don't forget to add space
 *                         for the NUL if you need it.
 * BOOTINFO_BUF_SIZE:      size in bytes of buffer that is large enough
 *                         to read a /proc/bootinfo line.  The extra
 *                         3 is for the " : ".  Don't forget to add
 *                         space for the NUL and newline if you
 *                         need them.
 */
#define BOOTINFO_MAX_NAME_LEN    32
#define BOOTINFO_MAX_VAL_LEN    128
#define BOOTINFO_BUF_SIZE       (BOOTINFO_MAX_NAME_LEN + \
					3 + BOOTINFO_MAX_VAL_LEN)

#endif


#if defined(__KERNEL__)
#if defined(CONFIG_BOOTINFO)

extern struct proc_dir_entry proc_root;

u32  bi_powerup_reason(void);
u32  bi_mbm_version(void);
void char_powerup_reason( char *pwr_on_reason);
void bi_powerup_reason_wdog(void);
void __init bi_add_bl_build_sig(char *bld_sig);

int __init bootinfo_bck_size(void);
void __init bootinfo_bck_buf_set_reserved(void);

#else /* defined(CONFIG_BOOTINFO) */

static inline u32 bi_powerup_reason(void) { return 0xFFFFFFFF; }
static inline u32 bi_mbm_version(void) { return 0xFFFFFFFF; }

#endif /* !defined(CONFIG_BOOTINFO) */
#endif /* defined(__KERNEL__) */

#endif
