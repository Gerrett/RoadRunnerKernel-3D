/*
 *  Shared Transport Low Level (ST LL)
 *
 *  Copyright (C) 2009 Texas Instruments
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef ST_LL_H
#define ST_LL_H

#include <linux/skbuff.h>
#include "st.h"
#include "st_core.h"

/* ST LL receiver states */
#define ST_W4_PACKET_TYPE       0
#define ST_W4_HEADER			1
#define ST_W4_DATA				2

/* ST LL state machines */
#define ST_LL_ASLEEP               0
#define ST_LL_ASLEEP_TO_AWAKE      1
#define ST_LL_AWAKE                2
#define ST_LL_AWAKE_TO_ASLEEP      3
#define ST_LL_INVALID		   4

#define LL_SLEEP_IND	0x30
#define LL_SLEEP_ACK	0x31
#define LL_WAKE_UP_IND	0x32
#define LL_WAKE_UP_ACK	0x33

/* ST LL private information */
struct ll_struct_s {
	unsigned long ll_state;	/* ST LL power state */
};

/* initialize and de-init ST LL */
long st_ll_init(void);
long st_ll_deinit(void);

/* enable/disable ST LL along with KIM start/stop
 * called by ST Core
 */
void st_ll_enable(void);
void st_ll_disable(void);

unsigned long st_ll_getstate(void);
unsigned long st_ll_sleep_state(unsigned char);
void st_ll_wakeup(void);
#endif /* ST_LL_H */
