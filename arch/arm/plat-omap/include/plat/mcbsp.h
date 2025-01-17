/*
 * arch/arm/plat-omap/include/mach/mcbsp.h
 *
 * Defines for Multi-Channel Buffered Serial Port
 *
 * Copyright (C) 2002 RidgeRun, Inc.
 * Author: Steve Johnson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#ifndef __ASM_ARCH_OMAP_MCBSP_H
#define __ASM_ARCH_OMAP_MCBSP_H

#include <linux/completion.h>
#include <linux/spinlock.h>

#include <mach/hardware.h>
#include <plat/clock.h>
#include <plat/dma.h>

/* macro for building platform_device for McBSP ports */
#define OMAP_MCBSP_PLATFORM_DEVICE(port_nr)		\
static struct platform_device omap_mcbsp##port_nr = {	\
	.name	= "omap-mcbsp-dai",			\
	.id	= OMAP_MCBSP##port_nr,			\
}

#define OMAP7XX_MCBSP1_BASE	0xfffb1000
#define OMAP7XX_MCBSP2_BASE	0xfffb1800

#define OMAP1510_MCBSP1_BASE	0xe1011800
#define OMAP1510_MCBSP2_BASE	0xfffb1000
#define OMAP1510_MCBSP3_BASE	0xe1017000

#define OMAP1610_MCBSP1_BASE	0xe1011800
#define OMAP1610_MCBSP2_BASE	0xfffb1000
#define OMAP1610_MCBSP3_BASE	0xe1017000

#define OMAP24XX_MCBSP1_BASE	0x48074000
#define OMAP24XX_MCBSP2_BASE	0x48076000
#define OMAP2430_MCBSP3_BASE	0x4808c000
#define OMAP2430_MCBSP4_BASE	0x4808e000
#define OMAP2430_MCBSP5_BASE	0x48096000

#define OMAP34XX_MCBSP1_BASE	0x48074000
#define OMAP34XX_MCBSP2_BASE	0x49022000
#define OMAP34XX_MCBSP2_ST_BASE	0x49028000
#define OMAP34XX_MCBSP3_BASE	0x49024000
#define OMAP34XX_MCBSP3_ST_BASE	0x4902A000
#define OMAP34XX_MCBSP3_BASE	0x49024000
#define OMAP34XX_MCBSP4_BASE	0x49026000
#define OMAP34XX_MCBSP5_BASE	0x48096000

#define OMAP44XX_MCBSP1_BASE	0x49022000
#define OMAP44XX_MCBSP2_BASE	0x49024000
#define OMAP44XX_MCBSP3_BASE	0x49026000
#define OMAP44XX_MCBSP4_BASE	0x48096000

#if defined(CONFIG_ARCH_OMAP15XX) || defined(CONFIG_ARCH_OMAP16XX) || defined(CONFIG_ARCH_OMAP730) || defined(CONFIG_ARCH_OMAP850)

#define OMAP_MCBSP_REG_DRR2	0x00
#define OMAP_MCBSP_REG_DRR1	0x02
#define OMAP_MCBSP_REG_DXR2	0x04
#define OMAP_MCBSP_REG_DXR1	0x06
#define OMAP_MCBSP_REG_SPCR2	0x08
#define OMAP_MCBSP_REG_SPCR1	0x0a
#define OMAP_MCBSP_REG_RCR2	0x0c
#define OMAP_MCBSP_REG_RCR1	0x0e
#define OMAP_MCBSP_REG_XCR2	0x10
#define OMAP_MCBSP_REG_XCR1	0x12
#define OMAP_MCBSP_REG_SRGR2	0x14
#define OMAP_MCBSP_REG_SRGR1	0x16
#define OMAP_MCBSP_REG_MCR2	0x18
#define OMAP_MCBSP_REG_MCR1	0x1a
#define OMAP_MCBSP_REG_RCERA	0x1c
#define OMAP_MCBSP_REG_RCERB	0x1e
#define OMAP_MCBSP_REG_XCERA	0x20
#define OMAP_MCBSP_REG_XCERB	0x22
#define OMAP_MCBSP_REG_PCR0	0x24
#define OMAP_MCBSP_REG_RCERC	0x26
#define OMAP_MCBSP_REG_RCERD	0x28
#define OMAP_MCBSP_REG_XCERC	0x2A
#define OMAP_MCBSP_REG_XCERD	0x2C
#define OMAP_MCBSP_REG_RCERE	0x2E
#define OMAP_MCBSP_REG_RCERF	0x30
#define OMAP_MCBSP_REG_XCERE	0x32
#define OMAP_MCBSP_REG_XCERF	0x34
#define OMAP_MCBSP_REG_RCERG	0x36
#define OMAP_MCBSP_REG_RCERH	0x38
#define OMAP_MCBSP_REG_XCERG	0x3A
#define OMAP_MCBSP_REG_XCERH	0x3C

/* Dummy defines, these are not available on omap1 */
#define OMAP_MCBSP_REG_XCCR	0x00
#define OMAP_MCBSP_REG_RCCR	0x00

#define AUDIO_MCBSP_DATAWRITE	(OMAP1510_MCBSP1_BASE + OMAP_MCBSP_REG_DXR1)
#define AUDIO_MCBSP_DATAREAD	(OMAP1510_MCBSP1_BASE + OMAP_MCBSP_REG_DRR1)

#define AUDIO_MCBSP		OMAP_MCBSP1
#define AUDIO_DMA_TX		OMAP_DMA_MCBSP1_TX
#define AUDIO_DMA_RX		OMAP_DMA_MCBSP1_RX

#else

#define OMAP_MCBSP_REG_DRR2	0x00
#define OMAP_MCBSP_REG_DRR1	0x04
#define OMAP_MCBSP_REG_DXR2	0x08
#define OMAP_MCBSP_REG_DXR1	0x0C
#define OMAP_MCBSP_REG_DRR	0x00
#define OMAP_MCBSP_REG_DXR	0x08
#define OMAP_MCBSP_REG_SPCR2	0x10
#define OMAP_MCBSP_REG_SPCR1	0x14
#define OMAP_MCBSP_REG_RCR2	0x18
#define OMAP_MCBSP_REG_RCR1	0x1C
#define OMAP_MCBSP_REG_XCR2	0x20
#define OMAP_MCBSP_REG_XCR1	0x24
#define OMAP_MCBSP_REG_SRGR2	0x28
#define OMAP_MCBSP_REG_SRGR1	0x2C
#define OMAP_MCBSP_REG_MCR2	0x30
#define OMAP_MCBSP_REG_MCR1	0x34
#define OMAP_MCBSP_REG_RCERA	0x38
#define OMAP_MCBSP_REG_RCERB	0x3C
#define OMAP_MCBSP_REG_XCERA	0x40
#define OMAP_MCBSP_REG_XCERB	0x44
#define OMAP_MCBSP_REG_PCR0	0x48
#define OMAP_MCBSP_REG_RCERC	0x4C
#define OMAP_MCBSP_REG_RCERD	0x50
#define OMAP_MCBSP_REG_XCERC	0x54
#define OMAP_MCBSP_REG_XCERD	0x58
#define OMAP_MCBSP_REG_RCERE	0x5C
#define OMAP_MCBSP_REG_RCERF	0x60
#define OMAP_MCBSP_REG_XCERE	0x64
#define OMAP_MCBSP_REG_XCERF	0x68
#define OMAP_MCBSP_REG_RCERG	0x6C
#define OMAP_MCBSP_REG_RCERH	0x70
#define OMAP_MCBSP_REG_XCERG	0x74
#define OMAP_MCBSP_REG_XCERH	0x78
#define OMAP_MCBSP_REG_SYSCON	0x8C
#define OMAP_MCBSP_REG_THRSH2	0x90
#define OMAP_MCBSP_REG_THRSH1	0x94
#define OMAP_MCBSP_REG_IRQST	0xA0
#define OMAP_MCBSP_REG_IRQEN	0xA4
#define OMAP_MCBSP_REG_WAKEUPEN	0xA8
#define OMAP_MCBSP_REG_XCCR	0xAC
#define OMAP_MCBSP_REG_RCCR	0xB0
#define OMAP_MCBSP_REG_XBUFFSTAT	0xB4
#define OMAP_MCBSP_REG_RBUFFSTAT	0xB8
#define OMAP_MCBSP_REG_SSELCR	0xBC

#define OMAP_ST_REG_REV		0x00
#define OMAP_ST_REG_SYSCONFIG	0x10
#define OMAP_ST_REG_IRQSTATUS	0x18
#define OMAP_ST_REG_IRQENABLE	0x1C
#define OMAP_ST_REG_SGAINCR	0x24
#define OMAP_ST_REG_SFIRCR	0x28
#define OMAP_ST_REG_SSELCR	0x2C

#define AUDIO_MCBSP_DATAWRITE	(OMAP24XX_MCBSP2_BASE + OMAP_MCBSP_REG_DXR1)
#define AUDIO_MCBSP_DATAREAD	(OMAP24XX_MCBSP2_BASE + OMAP_MCBSP_REG_DRR1)

#define AUDIO_MCBSP		OMAP_MCBSP2
#define AUDIO_DMA_TX		OMAP24XX_DMA_MCBSP2_TX
#define AUDIO_DMA_RX		OMAP24XX_DMA_MCBSP2_RX

#endif

/************************** McBSP SPCR1 bit definitions ***********************/
#define RRST			0x0001
#define RRDY			0x0002
#define RFULL			0x0004
#define RSYNC_ERR		0x0008
#define RINTM(value)		((value)<<4)	/* bits 4:5 */
#define ABIS			0x0040
#define DXENA			0x0080
#define CLKSTP(value)		((value)<<11)	/* bits 11:12 */
#define RJUST(value)		((value)<<13)	/* bits 13:14 */
#define ALB			0x8000
#define DLB			0x8000

/************************** McBSP SPCR2 bit definitions ***********************/
#define XRST		0x0001
#define XRDY		0x0002
#define XEMPTY		0x0004
#define XSYNC_ERR	0x0008
#define XINTM(value)	((value)<<4)		/* bits 4:5 */
#define GRST		0x0040
#define FRST		0x0080
#define SOFT		0x0100
#define FREE		0x0200

/************************** McBSP PCR bit definitions *************************/
#define CLKRP		0x0001
#define CLKXP		0x0002
#define FSRP		0x0004
#define FSXP		0x0008
#define DR_STAT		0x0010
#define DX_STAT		0x0020
#define CLKS_STAT	0x0040
#define SCLKME		0x0080
#define CLKRM		0x0100
#define CLKXM		0x0200
#define FSRM		0x0400
#define FSXM		0x0800
#define RIOEN		0x1000
#define XIOEN		0x2000
#define IDLE_EN		0x4000

/************************** McBSP RCR1 bit definitions ************************/
#define RWDLEN1(value)		((value)<<5)	/* Bits 5:7 */
#define RFRLEN1(value)		((value)<<8)	/* Bits 8:14 */

/************************** McBSP XCR1 bit definitions ************************/
#define XWDLEN1(value)		((value)<<5)	/* Bits 5:7 */
#define XFRLEN1(value)		((value)<<8)	/* Bits 8:14 */

/*************************** McBSP RCR2 bit definitions ***********************/
#define RDATDLY(value)		(value)		/* Bits 0:1 */
#define RFIG			0x0004
#define RCOMPAND(value)		((value)<<3)	/* Bits 3:4 */
#define RWDLEN2(value)		((value)<<5)	/* Bits 5:7 */
#define RFRLEN2(value)		((value)<<8)	/* Bits 8:14 */
#define RPHASE			0x8000

/*************************** McBSP XCR2 bit definitions ***********************/
#define XDATDLY(value)		(value)		/* Bits 0:1 */
#define XFIG			0x0004
#define XCOMPAND(value)		((value)<<3)	/* Bits 3:4 */
#define XWDLEN2(value)		((value)<<5)	/* Bits 5:7 */
#define XFRLEN2(value)		((value)<<8)	/* Bits 8:14 */
#define XPHASE			0x8000

/************************* McBSP SRGR1 bit definitions ************************/
#define CLKGDV(value)		(value)		/* Bits 0:7 */
#define FWID(value)		((value)<<8)	/* Bits 8:15 */

/************************* McBSP SRGR2 bit definitions ************************/
#define FPER(value)		(value)		/* Bits 0:11 */
#define FSGM			0x1000
#define CLKSM			0x2000
#define CLKSP			0x4000
#define GSYNC			0x8000

/************************* McBSP MCR1 bit definitions *************************/
#define RMCM			0x0001
#define RCBLK(value)		((value)<<2)	/* Bits 2:4 */
#define RPABLK(value)		((value)<<5)	/* Bits 5:6 */
#define RPBBLK(value)		((value)<<7)	/* Bits 7:8 */

/************************* McBSP MCR2 bit definitions *************************/
#define XMCM(value)		(value)		/* Bits 0:1 */
#define XCBLK(value)		((value)<<2)	/* Bits 2:4 */
#define XPABLK(value)		((value)<<5)	/* Bits 5:6 */
#define XPBBLK(value)		((value)<<7)	/* Bits 7:8 */

/*********************** McBSP XCCR bit definitions *************************/
#define EXTCLKGATE		0x8000
#define PPCONNECT		0x4000
#define DXENDLY(value)		((value)<<12)	/* Bits 12:13 */
#define XFULL_CYCLE		0x0800
#define DILB			0x0020
#define XDMAEN			0x0008
#define XDISABLE		0x0001

/********************** McBSP RCCR bit definitions *************************/
#define RFULL_CYCLE		0x0800
#define RDMAEN			0x0008
#define RDISABLE		0x0001

/********************** McBSP SYSCONFIG bit definitions ********************/
#define CLOCKACTIVITY(value)	((value)<<8)
#define SIDLEMODE(value)	((value)<<3)
#define ENAWAKEUP		0x0004
#define SOFTRST			0x0002
#define OMAP_MCBSP_WORDLEN_NONE 255

#define OMAP_MCBSP_MASTER                       1
#define OMAP_MCBSP_SLAVE                        0

/* McBSP interface operating mode */
#define OMAP_MCBSP_MASTER                       1
#define OMAP_MCBSP_SLAVE                        0
#define OMAP_MCBSP_AUTO_RST_NONE                (0x0)
#define OMAP_MCBSP_AUTO_RRST                    (0x1<<1)
#define OMAP_MCBSP_AUTO_XRST                    (0x1<<2)

/* SRG ENABLE/DISABLE state */
#define OMAP_MCBSP_ENABLE_FSG_SRG               1
#define OMAP_MCBSP_DISABLE_FSG_SRG              2
/* mono to mono mode*/
#define OMAP_MCBSP_SKIP_NONE                    (0x0)
/* mono to stereo mode */
#define OMAP_MCBSP_SKIP_FIRST                   (0x1<<1)
#define OMAP_MCBSP_SKIP_SECOND                  (0x1<<2)
/* RRST STATE */
#define OMAP_MCBSP_RRST_DISABLE                 0
#define OMAP_MCBSP_RRST_ENABLE                  1
/*XRST STATE */
#define OMAP_MCBSP_XRST_DISABLE                 0
#define OMAP_MCBSP_XRST_ENABLE                  1

#define OMAP_MCBSP_FRAME_SINGLEPHASE            1
#define OMAP_MCBSP_FRAME_DUALPHASE              2

/* Sample Rate Generator Clock source */
#define OMAP_MCBSP_SRGCLKSRC_CLKS               1
#define OMAP_MCBSP_SRGCLKSRC_FCLK               2
#define OMAP_MCBSP_SRGCLKSRC_CLKR               3
#define OMAP_MCBSP_SRGCLKSRC_CLKX               4
/* SRG input clock polarity */
#define OMAP_MCBSP_CLKS_POLARITY_RISING         1
#define OMAP_MCBSP_CLKS_POLARITY_FALLING        2

#define OMAP_MCBSP_CLKX_POLARITY_RISING         1
#define OMAP_MCBSP_CLKX_POLARITY_FALLING        2

#define OMAP_MCBSP_CLKR_POLARITY_RISING         1
#define OMAP_MCBSP_CLKR_POLARITY_FALLING        2

/* SRG Clock synchronization mode */
#define OMAP_MCBSP_SRG_FREERUNNING              1
#define OMAP_MCBSP_SRG_RUNNING                  2

/* Frame Sync Source */
#define OMAP_MCBSP_TXFSYNC_EXTERNAL             0
#define OMAP_MCBSP_TXFSYNC_INTERNAL             1

#define OMAP_MCBSP_RXFSYNC_EXTERNAL             0
#define OMAP_MCBSP_RXFSYNC_INTERNAL             1

#define OMAP_MCBSP_CLKRXSRC_EXTERNAL            1
#define OMAP_MCBSP_CLKRXSRC_INTERNAL            2

#define OMAP_MCBSP_CLKTXSRC_EXTERNAL            1
#define OMAP_MCBSP_CLKTXSRC_INTERNAL            2

/* Justification */
#define OMAP_MCBSP_RJUST_ZEROMSB                0
#define OMAP_MCBSP_RJUST_SIGNMSB                1
#define OMAP_MCBSP_LJUST_ZEROLSB                2

#define OMAP_MCBSP_DATADELAY0                   0
#define OMAP_MCBSP_DATADELAY1                   1
#define OMAP_MCBSP_DATADELAY2                   2

/* Reverse mode  */
#define OMAP_MCBSP_MSBFIRST                     0
#define OMAP_MCBSP_LSBFIRST                     1

/* Multi-Channel partition mode */
#define OMAP_MCBSP_TWOPARTITION_MODE            0
#define OMAP_MCBSP_EIGHTPARTITION_MODE          1

/* Rx Multichannel selection */
#define OMAP_MCBSP_RXMUTICH_DISABLE             0
#define OMAP_MCBSP_RXMUTICH_ENABLE              1

/* Tx Multichannel selection */
#define OMAP_MCBSP_TXMUTICH_DISABLE             0
#define OMAP_MCBSP_TXMUTICH_ENABLE              1

#define OMAP_MCBSP_FRAMELEN_N(NUM_WORDS)        ((NUM_WORDS - 1) & 0x7F)

/********************** McBSP SSELCR bit definitions ***********************/
#define SIDETONEEN		0x0400

/********************** McBSP Sidetone SYSCONFIG bit definitions ***********/
#define ST_AUTOIDLE		0x0001

/********************** McBSP Sidetone SGAINCR bit definitions *************/
#define ST_CH1GAIN(value)	((value<<16))	/* Bits 16:31 */
#define ST_CH0GAIN(value)	(value)		/* Bits 0:15 */

/********************** McBSP Sidetone SFIRCR bit definitions **************/
#define ST_FIRCOEFF(value)	(value)		/* Bits 0:15 */

/********************** McBSP Sidetone SSELCR bit definitions **************/
#define ST_COEFFWRDONE		0x0004
#define ST_COEFFWREN		0x0002
#define ST_SIDETONEEN		0x0001

/********************** McBSP DMA operating modes **************************/
#define MCBSP_DMA_MODE_ELEMENT		0
#define MCBSP_DMA_MODE_THRESHOLD	1
#define MCBSP_DMA_MODE_FRAME		2

/********************** McBSP WAKEUPEN bit definitions *********************/
#define XEMPTYEOFEN		0x4000
#define XRDYEN			0x0400
#define XEOFEN			0x0200
#define XFSXEN			0x0100
#define XSYNCERREN		0x0080
#define RRDYEN			0x0008
#define REOFEN			0x0004
#define RFSREN			0x0002
#define RSYNCERREN		0x0001

/* we don't do multichannel for now */
struct omap_mcbsp_reg_cfg {
	u16 spcr2;
	u16 spcr1;
	u16 rcr2;
	u16 rcr1;
	u16 xcr2;
	u16 xcr1;
	u16 srgr2;
	u16 srgr1;
	u16 mcr2;
	u16 mcr1;
	u16 pcr0;
	u16 rcerc;
	u16 rcerd;
	u16 xcerc;
	u16 xcerd;
	u16 rcere;
	u16 rcerf;
	u16 xcere;
	u16 xcerf;
	u16 rcerg;
	u16 rcerh;
	u16 xcerg;
	u16 xcerh;
	u16 xccr;
	u16 rccr;
};

typedef enum {
	OMAP_MCBSP1 = 0,
	OMAP_MCBSP2,
	OMAP_MCBSP3,
	OMAP_MCBSP4,
	OMAP_MCBSP5
} omap_mcbsp_id;

typedef int __bitwise omap_mcbsp_io_type_t;
#define OMAP_MCBSP_IRQ_IO ((__force omap_mcbsp_io_type_t) 1)
#define OMAP_MCBSP_POLL_IO ((__force omap_mcbsp_io_type_t) 2)
#define omap_mcbsp_check_valid_id(id)   (id < omap_mcbsp_count)
#define id_to_mcbsp_ptr(id)             mcbsp_ptr[id];

typedef void (*omap_mcbsp_dma_cb) (u32 ch_status, void *arg);

typedef struct omap_mcbsp_dma_transfer_parameters {

	/* Skip the alternate element use fro stereo mode */
	u8 skip_alt;
	/* Automagically handle Transfer [XR]RST? */
	u8   auto_reset;
	/* callback function executed for every tx/rx completion */
	omap_mcbsp_dma_cb callback;
	/* word length of data */
	u32 word_length1;
} omap_mcbsp_dma_transfer_params;

typedef enum {
	OMAP_MCBSP_WORD_8 = 0,
	OMAP_MCBSP_WORD_12,
	OMAP_MCBSP_WORD_16,
	OMAP_MCBSP_WORD_20,
	OMAP_MCBSP_WORD_24,
	OMAP_MCBSP_WORD_32,
} omap_mcbsp_word_length;

typedef enum {
	OMAP_MCBSP_CLK_RISING = 0,
	OMAP_MCBSP_CLK_FALLING,
} omap_mcbsp_clk_polarity;

typedef enum {
	OMAP_MCBSP_FS_ACTIVE_HIGH = 0,
	OMAP_MCBSP_FS_ACTIVE_LOW,
} omap_mcbsp_fs_polarity;

typedef enum {
	OMAP_MCBSP_CLK_STP_MODE_NO_DELAY = 0,
	OMAP_MCBSP_CLK_STP_MODE_DELAY,
} omap_mcbsp_clk_stp_mode;


/******* SPI specific mode **********/
typedef enum {
	OMAP_MCBSP_SPI_MASTER = 0,
	OMAP_MCBSP_SPI_SLAVE,
} omap_mcbsp_spi_mode;

struct omap_mcbsp_spi_cfg {
	omap_mcbsp_spi_mode		spi_mode;
	omap_mcbsp_clk_polarity		rx_clock_polarity;
	omap_mcbsp_clk_polarity		tx_clock_polarity;
	omap_mcbsp_fs_polarity		fsx_polarity;
	u8				clk_div;
	omap_mcbsp_clk_stp_mode		clk_stp_mode;
	omap_mcbsp_word_length		word_length;
};

/* Platform specific configuration */
struct omap_mcbsp_ops {
	void (*request)(unsigned int);
	void (*free)(unsigned int);
};

struct omap_mcbsp_platform_data {
	struct omap_mcbsp_ops *ops;
	unsigned long phys_base_st;
	u16 buffer_size;
	int dma_op_mode;
	struct clk *fclk;
	struct omap_hwmod **oh;
};

struct omap_mcbsp_st_data {
	void __iomem *io_base_st;
	bool running;
	bool enabled;
	s16 taps[128];	/* Sidetone filter coefficients */
	int nr_taps;	/* Number of filter coefficients in use */
	s16 ch0gain;
	s16 ch1gain;
};

struct omap_mcbsp {
	struct device *dev;
	unsigned long phys_base;
	void __iomem *io_base;
	u8 id;
	u8 free;
	omap_mcbsp_word_length rx_word_length;
	omap_mcbsp_word_length tx_word_length;

	struct clk *fclk;
	omap_mcbsp_io_type_t io_type; /* IRQ or poll */
	/* IRQ based TX/RX */
	int rx_irq;
	int tx_irq;

	/* DMA stuff */
	u8 dma_rx_sync;
	short dma_rx_lch;
	u8 dma_tx_sync;
	short dma_tx_lch;

	/* Completion queues */
	struct completion tx_irq_completion;
	struct completion rx_irq_completion;
	struct completion tx_dma_completion;
	struct completion rx_dma_completion;

	/* Protect the field .free, while checking if the mcbsp is in use */
	spinlock_t lock;
	struct omap_mcbsp_platform_data *pdata;

	u8  auto_reset; /* Auto Reset */
	u8  txskip_alt; /* Tx skip flags */
	u8  rxskip_alt; /* Rx skip flags */
	void  *rx_cb_arg;
	void  *tx_cb_arg;
	omap_mcbsp_dma_cb  rx_callback;
	omap_mcbsp_dma_cb  tx_callback;
	int  rx_dma_chain_state;
	int  tx_dma_chain_state;
	int  interface_mode; /* Master / Slave */
	struct omap_dma_channel_params rx_params; /* Used For Rx FIFO */
	int rx_config_done;
	struct omap_mcbsp_st_data *st_data;
	int dma_op_mode;
	u16 max_tx_thres;
	u16 max_rx_thres;
	void *reg_cache;
	struct omap_hwmod **oh;
};
struct omap_mcbsp_dma_transfer_params {
	/* Skip the alternate element use fro stereo mode */
	u8 skip_alt;
	/* Automagically handle Transfer [XR]RST? */
	u8   auto_reset;
	/* callback function executed for every tx/rx completion */
	omap_mcbsp_dma_cb callback;
	/* word length of data */
	u32 word_length1;
};

struct omap_mcbsp_cfg_param {
	u8 fsync_src;
	u8 fs_polarity;
	u8 clk_polarity;
	u8 clk_mode;
	u8 frame_length1;
	u8 frame_length2;
	u8 word_length1;
	u8 word_length2;
	u8 justification;
	u8 reverse_compand;
	u8 phase;
	u8 data_delay;
};

struct omap_mcbsp_srg_fsg_cfg {
	u32 period;     /* Frame period */
	u32 pulse_width; /* Frame width */
	u8 fsgm;
	u32 sample_rate;
	u32 bits_per_sample;
	u32 srg_src;
	u8 sync_mode;   /* SRG free running mode */
	u8 polarity;
	u8 dlb;		 /* digital loopback mode */
};
extern struct omap_mcbsp **mcbsp_ptr;
extern int omap_mcbsp_count, omap_mcbsp_cache_size;

int omap_mcbsp_init(void);
void omap_mcbsp_register_board_cfg(struct omap_mcbsp_platform_data *config,
					int size);
void omap_mcbsp_config(unsigned int id, const struct omap_mcbsp_reg_cfg * config);
#ifdef CONFIG_ARCH_OMAP3
void omap_mcbsp_set_tx_threshold(unsigned int id, u16 threshold);
void omap_mcbsp_set_rx_threshold(unsigned int id, u16 threshold);
u16 omap_mcbsp_get_max_tx_threshold(unsigned int id);
u16 omap_mcbsp_get_max_rx_threshold(unsigned int id);
u16 omap_mcbsp_get_fifo_size(unsigned int id);
u16 omap_mcbsp_get_tx_delay(unsigned int id);
u16 omap_mcbsp_get_rx_delay(unsigned int id);
int omap_mcbsp_get_dma_op_mode(unsigned int id);
#else
static inline void omap_mcbsp_set_tx_threshold(unsigned int id, u16 threshold)
{ }
static inline void omap_mcbsp_set_rx_threshold(unsigned int id, u16 threshold)
{ }
static inline u16 omap_mcbsp_get_max_tx_threshold(unsigned int id) { return 0; }
static inline u16 omap_mcbsp_get_max_rx_threshold(unsigned int id) { return 0; }
static inline u16 omap_mcbsp_get_fifo_size(unsigned int id) { return 0; }
static inline u16 omap_mcbsp_get_tx_delay(unsigned int id) { return 0; }
static inline u16 omap_mcbsp_get_rx_delay(unsigned int id) { return 0; }
static inline int omap_mcbsp_get_dma_op_mode(unsigned int id) { return 0; }
#endif
int omap_mcbsp_request(unsigned int id);
unsigned int omap_mcbsp_get_irqstatus(unsigned int id);
int omap_mcbsp_recover_tx_underflow(unsigned int id);
void omap_mcbsp_free(unsigned int id);
void omap_mcbsp_start(unsigned int id, int tx, int rx);
void omap_mcbsp_stop(unsigned int id, int tx, int rx);
void omap_mcbsp_xmit_word(unsigned int id, u32 word);
u32 omap_mcbsp_recv_word(unsigned int id);

int omap_mcbsp_xmit_buffer(unsigned int id, dma_addr_t buffer, unsigned int length);
int omap_mcbsp_recv_buffer(unsigned int id, dma_addr_t buffer, unsigned int length);
int omap_mcbsp_spi_master_xmit_word_poll(unsigned int id, u32 word);
int omap_mcbsp_spi_master_recv_word_poll(unsigned int id, u32 * word);


int omap2_mcbsp_stop_datatx(unsigned int id);
int omap2_mcbsp_stop_datarx(u32 id);
int omap2_mcbsp_reset(unsigned int id);
int omap2_mcbsp_transmitter_index(int id, int *ei, int *fi);
int omap2_mcbsp_receiver_index(int id, int *ei, int *fi);
extern int omap2_mcbsp_set_xrst(unsigned int id, u8 state);
extern int omap2_mcbsp_set_rrst(unsigned int id, u8 state);
extern int omap2_mcbsp_dma_recv_params(unsigned int id,
			struct	omap_mcbsp_dma_transfer_params *rp);
extern int omap2_mcbsp_dma_trans_params(unsigned int id,
			struct	omap_mcbsp_dma_transfer_params *tp);
extern int omap2_mcbsp_receive_data(unsigned int id, void *cbdata,
				dma_addr_t buf_start_addr, u32 buf_size);
extern int omap2_mcbsp_send_data(unsigned int id, void *cbdata,
				dma_addr_t buf_start_addr, u32 buf_size);

extern void omap_mcbsp_write(struct omap_mcbsp *mcbsp, u16 reg, u32 val);
extern int omap_mcbsp_read(struct omap_mcbsp *mcbsp, u16 reg, bool from_cache);

extern void omap2_mcbsp_params_cfg(unsigned int id, int interface_mode,
				struct omap_mcbsp_cfg_param *rp,
				struct omap_mcbsp_cfg_param *tp,
				struct omap_mcbsp_srg_fsg_cfg *param);
/* SPI specific API */
void omap_mcbsp_set_spi_mode(unsigned int id, const struct omap_mcbsp_spi_cfg * spi_cfg);

/* Polled read/write functions */
int omap_mcbsp_pollread(unsigned int id, u16 * buf);
int omap_mcbsp_pollwrite(unsigned int id, u16 buf);
int omap_mcbsp_set_io_type(unsigned int id, omap_mcbsp_io_type_t io_type);

#ifdef CONFIG_ARCH_OMAP3
/* Sidetone specific API */
int omap_st_set_chgain(unsigned int id, int channel, s16 chgain);
int omap_st_get_chgain(unsigned int id, int channel, s16 *chgain);
int omap_st_enable(unsigned int id);
int omap_st_disable(unsigned int id);
int omap_st_is_enabled(unsigned int id);
#else
static inline int omap_st_set_chgain(unsigned int id, int channel,
				     s16 chgain) { return 0; }
static inline int omap_st_get_chgain(unsigned int id, int channel,
				     s16 *chgain) { return 0; }
static inline int omap_st_enable(unsigned int id) { return 0; }
static inline int omap_st_disable(unsigned int id) { return 0; }
static inline int omap_st_is_enabled(unsigned int id) {  return 0; }
#endif

#endif
