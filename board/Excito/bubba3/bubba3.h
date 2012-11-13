/*
 * (C) Copyright 2010
 * Excito elektronik i Skåne AB <www.excito.com>
 * by: Tor Krill <tor@excito.com>
 * based on
 * (C) Copyright 2009
 * kernel concepts <www.kernelconcepts.de>
 * by: Nils Faerber <nils.faerber@kernelconcepts.de>
 * based on
 * (C) Copyright 2009
 * Marvell Semiconductor <www.marvell.com>
 * Written-by: Prafulla Wadaskar <prafulla@marvell.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA
 */

#ifndef __BUBBA3_H
#define __BUBBA3_H

/* set gpio0 and gpio1 output enabled pins*/
#define BUBBA3_OE_LOW		0xcff2efff
#define BUBBA3_OE_HIGH		0xfffc3883
/* output value is zer0 for all pins by default */
#define BUBBA3_OE_VAL_LOW	0x00000000
#define BUBBA3_OE_VAL_HIGH	0x00000000

/* PHY related */
#define MV88E1116_LED_FCTRL_REG		10
#define MV88E1116_CPRSP_CR3_REG		21
#define MV88E1116_MAC_CTRL_REG		21
#define MV88E1116_PGADR_REG		22
#define MV88E1116_RGMII_TXTM_CTRL	(1 << 4)
#define MV88E1116_RGMII_RXTM_CTRL	(1 << 5)

/*
 * GPIO definitions
 */
#define B3_LED_INTERVAL		(1<<5)
#define B3_FRONT_LED_GREEN	(1<<6)
#define B3_POWER_BUTTON		(1<<7)
#define B3_BUZZER_ENABLE	(1<<8)
#define B3_FRONT_LED_RED	(1<<9)
#define B3_FRONT_LED_BLUE	(1<<10)
#define B3_HW_ID0		(1<<11)
#define B3_HW_ID1		(1<<12)
#define B3_HW_ID2		(1<<13)
#define B3_BUZ_4KHZ		(1<<14)

/*
 * B3 specific functional defines
 */
#define MAGIC_OFFSET 	0x1f0000
#define MAGIC_BLOCKSIZE	0x10000
#define REBOOT_MAGIC 	0xdeadbeef

/* Forward declarations */

int b3_probe_button(void);
void b3_suspend(void);
void b3_wait_for_interrupt(void);

#endif /* __BUBBA3_H */
