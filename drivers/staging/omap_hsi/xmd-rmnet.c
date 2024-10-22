/*
 * arch/arm/mach-omap2/xmd_rmnet.c
 *
 * Copyright (C) 2011 Intel Mobile Communications. All rights reserved.
 *
 * Author[xmd/hsi related changes only]: Chaitanya <Chaitanya.Khened@intel.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/ipv6.h>
#include <linux/ip.h>
#include <asm/byteorder.h>

#include "xmd-ch.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

//#include <mach/msm_smd.h>
#include "xmd-ch.h"

/* XXX should come from smd headers */
#define SMD_PORT_ETHER0 11
#define POLL_DELAY 1000000 /* 1 second delay interval */

//#define RMNET_DEBUG
//#define RMNET_CRITICAL_DEBUG
#define RMNET_ERR

typedef enum {
	RMNET_FULL_PACKET,
	RMNET_PARTIAL_PACKET,
	RMNET_PARTIAL_HEADER,
} RMNET_PAST_STATE;

static struct {
	RMNET_PAST_STATE state;
	char buf[2500];
	int size;
	int type;
} past_packet;

static struct xmd_ch_info rmnet_channels[MAX_SMD_NET] = {
                                            {0,  "CHANNEL13",  0, XMD_NET, NULL, 0, SPIN_LOCK_UNLOCKED},
                                            {1,  "CHANNEL14",  0, XMD_NET, NULL, 0, SPIN_LOCK_UNLOCKED},
                                            {2,  "CHANNEL15",  0, XMD_NET, NULL, 0, SPIN_LOCK_UNLOCKED},
                                           };

struct rmnet_private
{
	struct xmd_ch_info *ch;
	struct net_device_stats stats;
	const char *chname;
	struct wake_lock wake_lock;
#ifdef CONFIG_MSM_RMNET_DEBUG
	ktime_t last_packet;
	short active_countdown; /* Number of times left to check */
	short restart_count; /* Number of polls seems so far */
	unsigned long wakeups_xmit;
	unsigned long wakeups_rcv;
	unsigned long timeout_us;
	unsigned long awake_time_ms;
	struct delayed_work work;
#endif
};

static int count_this_packet(void *_hdr, int len)
{
	struct ethhdr *hdr = _hdr;

	if (len >= ETH_HLEN && hdr->h_proto == htons(ETH_P_ARP))
		return 0;

	return 1;
}

#ifdef CONFIG_MSM_RMNET_DEBUG
static int in_suspend;
static unsigned long timeout_us;
static struct workqueue_struct *rmnet_wq;

static void do_check_active(struct work_struct *work)
{
	struct rmnet_private *p =
		container_of(work, struct rmnet_private, work.work);

	/*
	 * Soft timers do not wake the cpu from suspend.
	 * If we are in suspend, do_check_active is only called once at the
	 * timeout time instead of polling at POLL_DELAY interval. Otherwise the
	 * cpu will sleeps and the timer can fire much much later than POLL_DELAY
	 * casuing a skew in time calculations.
	 */
	if (in_suspend) {
		/*
		 * Assume for N packets sent durring this session, they are
		 * uniformly distributed durring the timeout window.
		 */
		int tmp = p->timeout_us * 2 -
			(p->timeout_us / (p->active_countdown + 1));
		tmp /= 1000;
		p->awake_time_ms += tmp;

		p->active_countdown = p->restart_count = 0;
		return;
	}

	/*
	 * Poll if not in suspend, since this gives more accurate tracking of
	 * rmnet sessions.
	 */
	p->restart_count++;
	if (--p->active_countdown == 0) {
		p->awake_time_ms += p->restart_count * POLL_DELAY / 1000;
		p->restart_count = 0;
	} else {
		queue_delayed_work(rmnet_wq, &p->work,
				usecs_to_jiffies(POLL_DELAY));
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/*
 * If early suspend is enabled then we specify two timeout values,
 * screen on (default), and screen is off.
 */
static unsigned long timeout_suspend_us;
static struct device *rmnet0;

/* Set timeout in us when the screen is off. */
static ssize_t timeout_suspend_store(struct device *d, struct device_attribute *attr, const char *buf, size_t n)
{
	timeout_suspend_us = simple_strtoul(buf, NULL, 10);
	return n;
}

static ssize_t timeout_suspend_show(struct device *d,
				    struct device_attribute *attr,
				    char *buf)
{
	return sprintf(buf, "%lu\n", (unsigned long) timeout_suspend_us);
}

static DEVICE_ATTR(timeout_suspend, 0664, timeout_suspend_show, timeout_suspend_store);

static void rmnet_early_suspend(struct early_suspend *handler) 
{
	if (rmnet0) {
		struct rmnet_private *p = netdev_priv(to_net_dev(rmnet0));
		p->timeout_us = timeout_suspend_us;
	}
	in_suspend = 1;
}

static void rmnet_late_resume(struct early_suspend *handler) 
{
	if (rmnet0) {
		struct rmnet_private *p = netdev_priv(to_net_dev(rmnet0));
		p->timeout_us = timeout_us;
	}
	in_suspend = 0;
}

static struct early_suspend rmnet_power_suspend = {
	.suspend = rmnet_early_suspend,
	.resume = rmnet_late_resume,
};

static int __init rmnet_late_init(void)
{
	register_early_suspend(&rmnet_power_suspend);
	return 0;
}

late_initcall(rmnet_late_init);
#endif

/* Returns 1 if packet caused rmnet to wakeup, 0 otherwise. */
static int rmnet_cause_wakeup(struct rmnet_private *p) 
{
	int ret = 0;
	ktime_t now;
	if (p->timeout_us == 0) /* Check if disabled */
		return 0;

	/* Start timer on a wakeup packet */
	if (p->active_countdown == 0) {
		ret = 1;
		now = ktime_get_real();
		p->last_packet = now;
		if (in_suspend)
			queue_delayed_work(rmnet_wq, &p->work,
					usecs_to_jiffies(p->timeout_us));
		else
			queue_delayed_work(rmnet_wq, &p->work,
					usecs_to_jiffies(POLL_DELAY));
	}

	if (in_suspend)
		p->active_countdown++;
	else
		p->active_countdown = p->timeout_us / POLL_DELAY;

	return ret;
}

static ssize_t wakeups_xmit_show(struct device *d,
				 struct device_attribute *attr,
				 char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	return sprintf(buf, "%lu\n", p->wakeups_xmit);
}

DEVICE_ATTR(wakeups_xmit, 0444, wakeups_xmit_show, NULL);

static ssize_t wakeups_rcv_show(struct device *d, struct device_attribute *attr,
		char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	return sprintf(buf, "%lu\n", p->wakeups_rcv);
}

DEVICE_ATTR(wakeups_rcv, 0444, wakeups_rcv_show, NULL);

/* Set timeout in us. */
static ssize_t timeout_store(struct device *d, struct device_attribute *attr,
		const char *buf, size_t n)
{
#ifndef CONFIG_HAS_EARLYSUSPEND
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	p->timeout_us = timeout_us = simple_strtoul(buf, NULL, 10);
#else
/* If using early suspend/resume hooks do not write the value on store. */
	timeout_us = simple_strtoul(buf, NULL, 10);
#endif
	return n;
}

static ssize_t timeout_show(struct device *d, struct device_attribute *attr,
			    char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	p = netdev_priv(to_net_dev(d));
	return sprintf(buf, "%lu\n", timeout_us);
}

DEVICE_ATTR(timeout, 0664, timeout_show, timeout_store);

/* Show total radio awake time in ms */
static ssize_t awake_time_show(struct device *d, struct device_attribute *attr,
				char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	return sprintf(buf, "%lu\n", p->awake_time_ms);
}
DEVICE_ATTR(awake_time_ms, 0444, awake_time_show, NULL);

#endif


#define RMNET_IPV6_VER 0x6
#define RMNET_IPV4_VER 0x4

//give the packet to TCP/IP
static void xmd_trans_packet(struct net_device *dev, int type, void *buf, int sz)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct sk_buff *skb;
	void *ptr = NULL;
  
	sz += 14; //14byte ethernet header should be added
  
#if defined (RMNET_CRITICAL_DEBUG)
	printk("\nRMNET: %d<\n",sz);
#endif

	if (sz > 1514) {
		pr_err("rmnet_recv() discarding %d len\n", sz);
		ptr = 0;
	} else {
		skb = dev_alloc_skb(sz + NET_IP_ALIGN);
		if (skb == NULL) {
			pr_err("rmnet_recv() cannot allocate skb\n");
		} else {
			skb->dev = dev;
			skb_reserve(skb, NET_IP_ALIGN);
			ptr = skb_put(skb, sz);
			wake_lock_timeout(&p->wake_lock, HZ / 2);
        
			/* adding ethernet header */
			{
				//struct ethhdr eth_hdr = {0xB6,0x91,0x24,0xa8,0x14,0x72,0xb6,0x91,0x24,0xa8,0x14,0x72,0x08,0x0};
				char temp[] = {0xB6,0x91,0x24,0xa8,0x14,0x72,0xb6,0x91,0x24,0xa8,0x14,0x72,0x08,0x0};
				struct ethhdr *eth_hdr = (struct ethhdr *) temp;
            
				if (type == RMNET_IPV6_VER) {
					eth_hdr->h_proto = 0x08DD;
					eth_hdr->h_proto = htons(eth_hdr->h_proto);
				}
              
				memcpy((void *)eth_hdr->h_dest, (void*)dev->dev_addr, sizeof(eth_hdr->h_dest));					  
				memcpy((void *)ptr, (void *)eth_hdr, sizeof(struct ethhdr));										  
			}
			memcpy(ptr + 14, buf, sz - 14);
			skb->protocol = eth_type_trans(skb, dev);
			if (count_this_packet(ptr, skb->len)) {
#ifdef CONFIG_MSM_RMNET_DEBUG
				p->wakeups_rcv += rmnet_cause_wakeup(p);
#endif
				p->stats.rx_packets++;
				p->stats.rx_bytes += skb->len;
			}
			netif_rx(skb);
		}
	}
}

/* Called in wq context */
static void xmd_net_notify(int chno)
{
	int i;
	struct net_device *dev = NULL;
	void *buf = 0;
	int tot_sz = 0;

	struct rmnet_private *p = NULL;
	struct xmd_ch_info *info = NULL;

  
	for (i=0; i<ARRAY_SIZE(rmnet_channels); i++) {
		if (rmnet_channels[i].chno == chno)
			dev = (struct net_device *)rmnet_channels[i].priv;
	}
  
	if (!dev)
		return; 

	p = netdev_priv(dev);
	if (!p)
		return;
	
	info = p->ch;
	if (!info)
		return;

	buf = xmd_ch_read(info->chno, &tot_sz); //contains the full data read from hsi channel.
  
	if (!buf)
		return;
#if defined (RMNET_CRITICAL_DEBUG)
	printk("\nRMNET: total size read = %d\n",tot_sz);
#endif
  
	switch (past_packet.state)
	{
	case RMNET_FULL_PACKET:
		// no need to do anything
	break;
    
	case RMNET_PARTIAL_PACKET:
	{
		void *ip_hdr = (void *)past_packet.buf;
		int sz;
		int copy_size;
      
#if defined (RMNET_DEBUG)
		printk("\nRMNET: past partial packet\n");
#endif
		if (past_packet.type == RMNET_IPV4_VER)
			sz = ntohs(((struct iphdr*) ip_hdr)->tot_len); 
		else if (past_packet.type == RMNET_IPV6_VER)
			sz = ntohs(((struct ipv6hdr*) ip_hdr)->payload_len) + sizeof(struct ipv6hdr); 
		else
		{
#if defined (RMNET_ERR)      
			printk("\nRMNET: Invalid past version,%d\n",past_packet.type);
#endif      
			past_packet.state = RMNET_FULL_PACKET;
			return;
		}
      
		copy_size = sz - past_packet.size;
      
		if (tot_sz >= copy_size) //if read size if > then copy size, copy full packet.
			memcpy(past_packet.buf + past_packet.size,buf,copy_size);
		else {
			memcpy(past_packet.buf + past_packet.size,buf,tot_sz); //copy whatever read if read size < packet size.
			past_packet.size += tot_sz;
			return;
		}
      
		xmd_trans_packet(dev,past_packet.type,(void*)past_packet.buf,sz);
      
		buf = buf + copy_size;
		tot_sz = tot_sz - copy_size;
	}
	break;
  
	case RMNET_PARTIAL_HEADER:
	{
		void *ip_hdr = (void *)past_packet.buf;
		int sz;
		int copy_size;
		int hdr_size = 0;
      
#if defined (RMNET_DEBUG)
		printk("\nRMNET: past partial header packet\n");
#endif
		if (past_packet.type == RMNET_IPV4_VER)
			hdr_size = sizeof(struct iphdr);
		else if (past_packet.type  == RMNET_IPV6_VER)
			hdr_size = sizeof(struct ipv6hdr);
		else {
#if defined (RMNET_ERR)      
			printk("\nRMNET: Invalid past version, %d\n",past_packet.type);
#endif        
			past_packet.state = RMNET_FULL_PACKET;
			break;
		}
      
		copy_size = hdr_size - past_packet.size;
      
		if(tot_sz >= copy_size)
			memcpy(past_packet.buf + past_packet.size,buf,copy_size);
		else {
			memcpy(past_packet.buf + past_packet.size,buf,tot_sz); //copy whatever read if read size < packet size.
			past_packet.size += tot_sz;
			return;
		}
      
		buf = buf + copy_size;
		tot_sz = tot_sz - copy_size;
		past_packet.size = past_packet.size + copy_size;
      
		if (past_packet.type == RMNET_IPV4_VER)
			sz = ntohs(((struct iphdr*) ip_hdr)->tot_len); 
		else if (past_packet.type == RMNET_IPV6_VER)
			sz = ntohs(((struct ipv6hdr*) ip_hdr)->payload_len) + sizeof(struct ipv6hdr); 
		else {
#if defined (RMNET_ERR)      
			printk("\nRMNET: Invalid past version, %d\n",past_packet.type);
#endif        
			past_packet.state = RMNET_FULL_PACKET;
			return;
		}
      
		copy_size = sz - past_packet.size;
      
		if (tot_sz >= copy_size) //if read size if > then copy size, copy full packet.
			memcpy(past_packet.buf + past_packet.size,buf,copy_size);
		else {
			memcpy(past_packet.buf + past_packet.size,buf,tot_sz); //copy whatever read if read size < packet size.
			past_packet.size += tot_sz;
			past_packet.state = RMNET_PARTIAL_PACKET;
			return;
		}
      
		xmd_trans_packet(dev,past_packet.type,(void *)past_packet.buf,sz);
      
		buf = buf + copy_size;
		tot_sz = tot_sz - copy_size;
      
	}
	break;
    
	default:
#if defined (RMNET_ERR)    
	printk("\nRMNET: Invalid past state %d\n",(int)past_packet.state);
#endif    
	past_packet.state = RMNET_FULL_PACKET;
	break;
	}
  
	while (tot_sz) {
		int hdr_size = 0;
		int ver = 0;
		void *ip_hdr = (void *)buf;
		int sz = 0;

#if defined(__BIG_ENDIAN_BITFIELD)
		ver = ((char *)buf)[0] & 0x0F; 
#elif defined(__LITTLE_ENDIAN_BITFIELD)
		ver = (((char *)buf)[0] & 0xF0)>>4;
#endif
    
#if 0//defined (RMNET_DEBUG)
  printk("\nRMNET: ver = 0x%x, buf[0] = 0x%x\n",ver,((char *)buf)[0]);
#endif

		if (ver == RMNET_IPV4_VER)
			hdr_size = sizeof(struct iphdr);
		else if (ver == RMNET_IPV6_VER)
			hdr_size = sizeof(struct ipv6hdr);
		else {
#if defined (RMNET_ERR)    
			printk("\nRMNET: Invalid version, 0x%x\n",ver);
#endif      
			break;
		}
      
		if (tot_sz < hdr_size) {
			past_packet.state = RMNET_PARTIAL_HEADER;
			past_packet.size = tot_sz;
			memcpy(past_packet.buf, buf, tot_sz);
			past_packet.type = ver; 
#if defined (RMNET_DEBUG)
			printk("\nRMNET: partial header packet, sz = %d\n",tot_sz);
#endif
			return;
		}

		if (ver == RMNET_IPV4_VER)
			sz = ntohs(((struct iphdr*) ip_hdr)->tot_len); 
		else if (ver == RMNET_IPV6_VER)
			sz = ntohs(((struct ipv6hdr*) ip_hdr)->payload_len) + sizeof(struct ipv6hdr); 
		else {
#if defined (RMNET_ERR)    
			printk("\nRMNET: Invalid version, %d\n",ver);
#endif      
			break;
		}
#if defined (RMNET_DEBUG)
		printk("\nRMNET: size = %d\n",sz);
#endif
      
		if (tot_sz < sz) {
			past_packet.state = RMNET_PARTIAL_PACKET;
			past_packet.size = tot_sz;
			memcpy(past_packet.buf, buf, tot_sz);
			past_packet.type = ver; 
#if defined (RMNET_DEBUG)
			printk("\nRMNET: partial packet, sz = %d\n",tot_sz);
#endif
			return;
		}
        
		xmd_trans_packet(dev,ver,buf,sz);
    
		tot_sz = tot_sz - sz;
		buf = buf + sz;
#if defined (RMNET_DEBUG)
		printk("\nRMNET: looping for another packet sz = %d\n",tot_sz);
#endif
	}
  
	past_packet.state = RMNET_FULL_PACKET; 
}


static int rmnet_open(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct xmd_ch_info *ch = p->ch;
	
	dev->flags&=~IFF_MULTICAST;
	dev->flags&=~IFF_BROADCAST;

	pr_info("rmnet_open()\n");

	if (p->ch) {
		ch->chno = xmd_ch_open(ch, xmd_net_notify);

		if (ch->chno < 0) {
			pr_err("error opening channel\n");
			ch->chno = 0;
			return -ENODEV;
		}
	}

	netif_start_queue(dev);
	return 0;
}

static int rmnet_stop(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct xmd_ch_info *info = p->ch;
  
	pr_info("rmnet_stop()\n");
	netif_stop_queue(dev);
	xmd_ch_close(info->chno);
	return 0;
}

static int rmnet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct xmd_ch_info *info = p->ch;

#if defined (RMNET_CRITICAL_DEBUG)  
	printk("\nRMNET[%d]: %d>\n",info->chno, skb->len);
#endif

	if (xmd_ch_write(info->chno,(void *)((char *) skb->data + 14), skb->len - 14) != 0) { /* 14 is the size of ethernet header which is being stripped */
		pr_err("rmnet fifo full, dropping packet\n");
	} else {
		if (count_this_packet(skb->data, skb->len)) {
			p->stats.tx_packets++;
			p->stats.tx_bytes += skb->len;
#ifdef CONFIG_MSM_RMNET_DEBUG
			p->wakeups_xmit += rmnet_cause_wakeup(p);
#endif
		}
	}

	dev_kfree_skb_irq(skb);
	return 0;
}

static struct net_device_stats *rmnet_get_stats(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	return &p->stats;
}

static void rmnet_set_multicast_list(struct net_device *dev)
{
}

static void rmnet_tx_timeout(struct net_device *dev)
{
	pr_info("rmnet_tx_timeout()\n");
}

static struct net_device_ops rmnet_ops = {
	.ndo_open = rmnet_open,
	.ndo_stop = rmnet_stop,
	.ndo_start_xmit = rmnet_xmit,
	.ndo_get_stats = rmnet_get_stats,
	.ndo_set_multicast_list = rmnet_set_multicast_list,
	.ndo_tx_timeout = rmnet_tx_timeout,
};

static void __init rmnet_setup(struct net_device *dev)
{
	dev->netdev_ops = &rmnet_ops;

	dev->watchdog_timeo = 20; /* ??? */

	ether_setup(dev);

	dev->mtu = 1514;
	dev->flags &= ~IFF_MULTICAST;
	dev->flags &= ~IFF_BROADCAST;
	
	random_ether_addr(dev->dev_addr);
}

static int __init rmnet_init(void)
{
	int ret;
	struct device *d;
	struct net_device *dev;
	struct rmnet_private *p;
	unsigned n;

#ifdef CONFIG_MSM_RMNET_DEBUG
	timeout_us = 0;
#ifdef CONFIG_HAS_EARLYSUSPEND
	timeout_suspend_us = 0;
#endif
#endif

#ifdef CONFIG_MSM_RMNET_DEBUG
	rmnet_wq = create_workqueue("rmnet");
#endif

	for (n = 0; n < MAX_SMD_NET; n++) {
		dev = alloc_netdev(sizeof(struct rmnet_private),
				   "rmnet%d", rmnet_setup);

		if (!dev)
			return -ENOMEM;

		d = &(dev->dev);
		p = netdev_priv(dev);
		rmnet_channels[n].priv = (void *)dev;
		p->ch = rmnet_channels + n;
		p->chname = rmnet_channels[n].name;
		wake_lock_init(&p->wake_lock, WAKE_LOCK_SUSPEND, rmnet_channels[n].name);
#ifdef CONFIG_MSM_RMNET_DEBUG
		p->timeout_us = timeout_us;
		p->awake_time_ms = p->wakeups_xmit = p->wakeups_rcv = 0;
		p->active_countdown = p->restart_count = 0;
		INIT_DELAYED_WORK_DEFERRABLE(&p->work, do_check_active);
#endif

		ret = register_netdev(dev);
		if (ret) {
			free_netdev(dev);
			return ret;
		}

#ifdef CONFIG_MSM_RMNET_DEBUG
		if (device_create_file(d, &dev_attr_timeout))
			continue;
		if (device_create_file(d, &dev_attr_wakeups_xmit))
			continue;
		if (device_create_file(d, &dev_attr_wakeups_rcv))
			continue;
		if (device_create_file(d, &dev_attr_awake_time_ms))
			continue;
#ifdef CONFIG_HAS_EARLYSUSPEND
		if (device_create_file(d, &dev_attr_timeout_suspend))
			continue;

		/* Only care about rmnet0 for suspend/resume tiemout hooks. */
		if (n == 0)
			rmnet0 = d;
#endif
#endif
	}
  
	past_packet.size = 0;
	past_packet.type = RMNET_FULL_PACKET;

	return 0;
}

module_init(rmnet_init);

