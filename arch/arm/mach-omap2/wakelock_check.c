

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/wakelock.h>

#include <linux/fs.h>
#include <linux/syscalls.h>


#include <linux/rtc.h> 
#include <linux/cosmo/charger_rt9524.h>

extern int tiler_memory_free_flag;
extern void tmm_dmm_free_page_stack(void);
extern bool dss_get_mainclk_state(void);

static void wakelock_check_wq (struct work_struct *unused);
struct timer_list wakelock_check_timer;

static DECLARE_WORK(wakelock_checkwq, wakelock_check_wq);

#define LGE_ABNORMAL_WAKE_PATH "/sys/devices/platform/i2c_omap.1/i2c-1/1-0049/twl6030_bci/abnormal_wakelock_dis"

static int abnormal_wake_diable_check(void)
{
	int h_file = 0;
	int ret = 0;

	char buf[2] = {0,};
	
	h_file = sys_open(LGE_ABNORMAL_WAKE_PATH, O_RDONLY,0);

	if(h_file > 0)
	{	
		ret = sys_read( h_file, buf, 1);

		if( ret != 1 )
		{
			printk("Can't read abnormal_wake check enable.\n");
			sys_close(h_file);			
			return 0;
		}				
		sys_close(h_file);		
		if(buf[0]=='0')
		{
			//printk("\nvalue is 1\n\n");
			return 1;
		}
	}
	//printk("Can't read abnormal_wake check enable..\n");	
	return 0;
}

static void wakelock_check_wq (struct work_struct *unused)
{

#ifdef CONFIG_HAS_WAKELOCK
	if(abnormal_wake_diable_check()) 						
	{
		if (find_abnormal_wake_lock(WAKE_LOCK_SUSPEND)) {
			int ret_abnormal=0;

			ret_abnormal = kill_abnormal_active_locks(WAKE_LOCK_SUSPEND);
			if(ret_abnormal)
			{
				printk(KERN_INFO "wake lock monitoring...\n");
				abnormal_wake_unlock_call(1);
			}	
			else
			{
				abnormal_wake_unlock_call(0);
			}
		}
	}
	else
		abnormal_wake_unlock_call(0);							

	if(tiler_memory_free_flag > 1)
	{
		printk(KERN_INFO "free tmm_dmm, tiler_memory_free_flag =%d\n", tiler_memory_free_flag);
		if(dss_get_mainclk_state() == false)
		{
			tmm_dmm_free_page_stack();	 
			tiler_memory_free_flag=0;		
		}
	}

	if (tiler_memory_free_flag ==1)
	{
		tiler_memory_free_flag++;	
	}

#endif
}

static void wakelock_check_timer_func(unsigned long data)
{
        wakelock_check_timer.expires = jiffies + (HZ*50);		//50sec
        add_timer(&wakelock_check_timer);
        schedule_work(&wakelock_checkwq);
}

static int __init wakelock_check_wq_init(void)
{
        init_timer(&wakelock_check_timer);
        wakelock_check_timer.expires = jiffies + HZ;
        wakelock_check_timer.data = 0;
        wakelock_check_timer.function = &wakelock_check_timer_func;

        add_timer(&wakelock_check_timer);

        return 0;
}

static void __exit wakelock_check_wq_exit(void)
{
        del_timer_sync(&wakelock_check_timer);
}

MODULE_AUTHOR("Kibum Lee");
MODULE_LICENSE("GPL");

module_init(wakelock_check_wq_init);
module_exit(wakelock_check_wq_exit);
