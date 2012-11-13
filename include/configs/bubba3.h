/*
 * (c) copyright 2010
 * Excito elektronik i Skåne AB <www.excito.com>
 * Adaptions by: Tor Krill <tor@excito.com>
 *
 * (C) Copyright 2009
 * kernel concepts, Nils Faerber <nils.faerber@kernelconcepts.de>
 * based on tk71.h which was based on rd6281a.h
 *
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

#ifndef _CONFIG_BUBBA3_H
#define _CONFIG_BUBBA3_H


/*
 * Version number information
 */
#define CONFIG_IDENT_STRING	"\nBUBBA|3"

/*
 * High Level Configuration Options (easy to change)
 */
#define CONFIG_MARVELL		1
#define CONFIG_ARM926EJS	1	/* Basic Architecture */
#define CONFIG_FEROCEON_88FR131	1	/* CPU Core subversion */
#define CONFIG_KIRKWOOD		1	/* SOC Family Name */
#define CONFIG_KW88F6281	1	/* SOC Name */
#define CONFIG_MACH_BUBBA3		/* Machine type */

#define CONFIG_MD5	/* get_random_hex on krikwood needs MD5 support */
#define CONFIG_SKIP_LOWLEVEL_INIT	/* disable board lowlevel_init */
#define CONFIG_KIRKWOOD_EGIGA_INIT	/* Enable GbePort0/1 for kernel */
#define CONFIG_KIRKWOOD_RGMII_PAD_1V8	/* Set RGMII Pad voltage to 1.8V */
#define CONFIG_KIRKWOOD_PCIE_INIT

/*
 * CLKs configurations
 */
#define CONFIG_SYS_HZ		1000
#define CONFIG_SYS_TCLK		166666667 /* 166MHz */

#include "asm/arch/config.h"

/*
 * NS16550 Configuration
 */
#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_SYS_NS16550_CLK		CONFIG_SYS_TCLK
#define CONFIG_SYS_NS16550_COM1		KW_UART0_BASE

/*
 * Serial Port configuration
 * The following definitions let you select what serial you want to use
 * for your console driver.
 */

#define CONFIG_CONS_INDEX	1	/*Console on UART0 */
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, \
					  115200,230400, 460800, 921600 }
/* auto boot */
#define CONFIG_BOOTDELAY	1

/* Run from ram */
/*FIXME, was previously off, so recheck*/
#define CONFIG_SKIP_RELOCATE_UBOOT	1


/*
 * For booting Linux, the board info and command line data
 * have to be in the first 8 MB of memory, since this is
 * the maximum mapped by the Linux kernel during initialization.
 */
#define CONFIG_CMDLINE_TAG	1	/* enable passing of ATAGs */
#define CONFIG_INITRD_TAG	1	/* enable INITRD tag */
#define CONFIG_SETUP_MEMORY_TAGS 1	/* enable memory tag */

#define	CONFIG_SYS_PROMPT	"B3> "	/* Command Prompt */
#define	CONFIG_SYS_CBSIZE	1024	/* Console I/O Buff Size */
#define	CONFIG_SYS_PBSIZE	(CONFIG_SYS_CBSIZE \
		+sizeof(CONFIG_SYS_PROMPT) + 16)	/* Print Buff */

/*
 * Use Hush parser
 */
#define CONFIG_SYS_HUSH_PARSER		1
#define CONFIG_SYS_PROMPT_HUSH_PS2	">"

/*
 * Support for FIT images
 */
#define CONFIG_FIT	1

/*
 * Commands configuration
 */
#define CONFIG_SYS_NO_FLASH		/* Declare no flash (NOR/SPI) */
#include <config_cmd_default.h>
#define CONFIG_CMD_AUTOSCRIPT
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_ENV
#define CONFIG_CMD_FAT
#define CONFIG_CMD_SPI
#define CONFIG_CMD_MII
#define CONFIG_CMD_SF
#define CONFIG_CMD_PING
#define CONFIG_CMD_USB
#define CONFIG_SYS_LONGHELP		/* undef to save memory		*/
#define CONFIG_CMDLINE_EDITING
#define CONFIG_AUTO_COMPLETE

/*
 * SPI config
 */
#ifdef CONFIG_CMD_SPI
#define CONFIG_KIRKWOOD_SPI
#define CONFIG_SPI_FLASH	1
#define CONFIG_HARD_SPI		1
#define CONFIG_SPI_FLASH_STMICRO	1
#define CONFIG_ENV_SPI_BUS	0
#define CONFIG_ENV_SPI_CS	0
#define CONFIG_ENV_SPI_MAX_HZ	40000000        /*40Mhz */
#endif


/*
 * Disk configuration
 */
#define	CONFIG_SATA_MV
#define CONFIG_SYS_SATA_MAX_DEVICE	2

#define CONFIG_SYS_64BIT_LBA

#ifdef CONFIG_SATA_MV
#define CONFIG_LBA48
#define CONFIG_LIBATA
#define CONFIG_CMD_SATA
#define CONFIG_DOS_PARTITION
#define CONFIG_EFI_PARTITION
#define CONFIG_CMD_EXT2
#endif

#define CONFIG_MISC_INIT_R	/* call board specific misc_init_r */
#define CONFIG_PREBOOT		/* possibly run a preboot cmd */

/*
 * Environment variables configurations
 */
#ifdef CONFIG_SPI_FLASH
#define CONFIG_ENV_IS_IN_SPI_FLASH	1
#define CONFIG_ENV_SECT_SIZE		0x10000 /* 64K */
#else
#define CONFIG_ENV_IS_NOWHERE		1	/* if env in SDRAM */
#endif
#define CONFIG_ENV_SIZE			0x010000 /* 64K */
#define CONFIG_ENV_ADDR			0x0c0000
#define CONFIG_ENV_OFFSET		0x0c0000 /* env starts here */
#define CONFIG_LOADADDR			0x800000 /* default load addr for tftp etc */

#define XMK_STR(x)	#x
#define MK_STR(x)	XMK_STR(x)

/*
 * Default environment variables
 */
#define CONFIG_EXTRA_ENV_SETTINGS 			\
	"console=ttyS0\0" 				\
	"bootfile=uImage\0"				\
	"flashfile=u-boot.kwb\0"			\
	"installfile=install.itb\0"			\
	"setdiskargs=setenv bootargs root=$diskdev "	\
		"console=$console,$baudrate serial=${serial#} key=$key " \
		"button=$button\0"			\
	"setbootargs=setenv bootargs "			\
		"root=$rootdev rw console=$console,$baudrate $othbootargs\0" \
	"usbbootroot=/dev/sda1\0"				\
	"usbbootdev=usb 0:1\0"				\
	"usbboot=usb start; "				\
		"setenv diskdev $usbbootroot; "		\
		"run setdiskargs; " 			\
		"ext2load $usbbootdev $loadaddr /boot/$bootfile; "\
		"bootm\0"				\
	"usbflashdev=usb 0:1\0"				\
	"usbflash=fatload $usbflashdev $loadaddr /install/$flashfile && " \
		"sf probe 0:0 && "			\
		"sf erase 0 80000; "			\
		"sf write $loadaddr 0 $filesize\0"	\
	"satadev=sata 0:1\0"				\
	"sataroot=/dev/sda1\0"				\
	"sataboot=setenv diskdev $sataroot; "		\
		"run setdiskargs; "			\
		"ext2load $satadev $loadaddr /boot/$bootfile; "\
		"bootm\0"				\
	"usbinstallroot=/dev/ram0\0"			\
	"usbinstalldev=usb 0:1\0"			\
	"usbinstall=usb start; "			\
		"setenv diskdev $usbinstallroot; "	\
		"run setdiskargs; " 			\
		"fatload $usbinstalldev $loadaddr /install/$installfile; " \
		"bootm\0"				\
	"bootalt1=run sataboot || reset\0" \
	"bootalt2=run usbinstall || run usbflash || run sataboot || reset\0"

/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN	(1024 * 512) /* 512kB for malloc() */
/* size in bytes reserved for initial data */
#define CONFIG_SYS_GBL_DATA_SIZE	128

/*
 * Other required minimal configurations
 */
#define CONFIG_CONSOLE_INFO_QUIET	/* some code reduction */
#define CONFIG_ARCH_CPU_INIT	/* call arch_cpu_init() */
#define CONFIG_ARCH_MISC_INIT	/* call arch_misc_init() */
#define CONFIG_DISPLAY_CPUINFO	/* Display cpu info */
#define CONFIG_NR_DRAM_BANKS	2
#define CONFIG_STACKSIZE	0x00100000	/* regular stack- 1M */
#define CONFIG_SYS_LOAD_ADDR	0x00800000	/* default load adr- 8M */
#define CONFIG_SYS_MEMTEST_START 0x00400000	/* 4M */
#define CONFIG_SYS_MEMTEST_END	0x007fffff	/*(_8M -1) */
#define CONFIG_SYS_RESET_ADDRESS 0xffff0000	/* Rst Vector Adr */
#define CONFIG_SYS_MAXARGS	16	/* max number of command args */

/*
 * Ethernet Driver configuration
 */
#ifdef CONFIG_CMD_NET
#define CONFIG_NET_MULTI	/* specify more that one ports available */
#define CONFIG_MII		/* expose smi over miiphy interface */
#define CONFIG_MVGBE_EGIGA	/* Enable kirkwood Gbe Controller Driver */
#define CONFIG_MVGBE_EGIGA_PORTS	{1,1}	/* enable first port */
#define CONFIG_PHY_ADDRS	{0x08,0x18}
#define CONFIG_ENV_OVERWRITE	/* ethaddr can be reprogrammed */
#define CONFIG_RESET_PHY_R	/* use reset_phy() to init switch and PHY */
#endif /* CONFIG_CMD_NET */

/*
 * USB/EHCI
 */
#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI			/* Enable EHCI USB support */
#define CONFIG_USB_EHCI_KIRKWOOD	/* on Kirkwood platform	*/
#define CONFIG_EHCI_IS_TDI
#define CONFIG_USB_STORAGE
#define CONFIG_DOS_PARTITION
#define CONFIG_ISO_PARTITION
#define CONFIG_SUPPORT_VFAT
#endif /* CONFIG_CMD_USB */


#define	CONFIG_SYS_TEXT_BASE	0x00400000

/* additions for new relocation code, must be added to all boards */
#define CONFIG_SYS_SDRAM_BASE		0x00000000
/* Kirkwood has 2k of Security SRAM, use it for SP */
#define CONFIG_SYS_INIT_SP_ADDR		0xC8012000
/* Do early setups now in board_init_f() */
#define CONFIG_BOARD_EARLY_INIT_F
/* Use irq detection for deep sleep mode */
#define CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	(4*1024)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	(4*1024)	/* FIQ stack */

#endif /* _CONFIG_BUBBA3_H */
