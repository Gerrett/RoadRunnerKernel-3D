/*
 * ifx_n721_spi.h -- Serial peheripheral interface framing layer for IFX modem.
 *
 * Copyright (C) 2009 Texas Instruments
 * Authors:	Umesh Bysani <bysani@ti.com> and
 *		Shreekanth D.H <sdh@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef IFX_N721_SPI_H
#define IFX_N721_SPI_H

#define IFX_SPI_MAJOR			153	/* assigned */
#define IFX_N_SPI_MINORS		4	/* ... up to 256 */



#define MRDY_CFG_REG     		AF9_3430_GPIO_22	/* Used from Schematic file */
#define SRDY_CFG_REG     		AF11_3430_GPIO_21	/* Used from Schematic file */




#if defined(CONFIG_MACH_LGE_E920)
/* nothing */
#else

#define IFX_MRDY_GPIO			120	/* MRDY GPIO pin for IFX - According to Windows Mobile */
#define IFX_SRDY_GPIO			119	/* SRDY GPIO pin for IFX - According to Windows Mobile */



#define MODEM_GPIO_AUDIO		95

#define MODEM_GPIO_PWRON		164

#define MODEM_GPIO_PWRON_SW		3   	

#endif


#define IFX_SPI_MAX_BUF_SIZE		2044	/* Max buffer size */
#define IFX_SPI_DEFAULT_BUF_SIZE	2044 	/* Default buffer size*/



#define IFX_SPI_HEADER_SIZE		4

#define SPI_MODE_0			(0|0)
#define SPI_MODE_1			(0|SPI_CPHA)
#define SPI_MODE_2			(SPI_CPOL|0)
#define SPI_MODE_3			(SPI_CPOL|SPI_CPHA)

#define SPI_CPHA			0x01
#define SPI_CPOL			0x02
#define SPI_CS_HIGH			0x04
#define SPI_LSB_FIRST			0x08
#define SPI_3WIRE			0x10
#define SPI_LOOP			0x20

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for CS_HIGH and 3WIRE can cause *lots* of trouble for other
 * devices on a shared bus:  CS_HIGH, because this device will be
 * active when it shouldn't be;  3WIRE, because when active it won't
 * behave as it should.
 *
 * REVISIT should changing those two modes be privileged?
 */
#define SPI_MODE_MASK			(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
					| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP)

#endif /* IFX_N721_SPI_H */
