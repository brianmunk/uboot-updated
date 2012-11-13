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

#include <common.h>
#include <spi_flash.h>
#include <sata.h>
/*#ifdef CONFIG_CMD_NET*/
#include <miiphy.h>
#include <netdev.h>
/*#endif*/
#include <asm/arch-kirkwood/kirkwood.h>
#include <asm/arch-kirkwood/cpu.h>
#include <asm/arch/kirkwood.h>
#include <asm/arch/mpp.h>
#include <asm/io.h>
#include "bubba3.h"
#include "bubba3_led.h"

DECLARE_GLOBAL_DATA_PTR;

/* Extern functions used by suspend */
int mv_sata_spin_down(int dev);
int mv_sata_spin_up(int dev);

int board_early_init_f(void) {
	return 0;
}

int board_init(void)
{
	/*
	 * default gpio configuration
	 * There are maximum 64 gpios controlled through 2 sets of registers
	 * the  below configuration configures mainly initial LED status
	 */
	kw_config_gpio(
					BUBBA3_OE_VAL_LOW,
					BUBBA3_OE_VAL_HIGH,
					BUBBA3_OE_LOW,
					BUBBA3_OE_HIGH);

	/* Multi-Purpose Pins Functionality configuration */
	u32 kwmpp_config[] = {
		MPP0_SPI_SCn,
		MPP1_SPI_MOSI,
		MPP2_SPI_SCK,
		MPP3_SPI_MISO,
		MPP4_NF_IO6,
		MPP5_NF_IO7,
		MPP6_SYSRST_OUTn,
		MPP7_GPO,
		MPP8_TW_SDA,
		MPP9_TW_SCK,
		MPP10_UART0_TXD,
		MPP11_UART0_RXD,
		MPP12_GPO,
		MPP13_UART1_TXD,
		MPP14_UART1_RXD,
		MPP15_SATA0_ACTn,
		MPP16_SATA1_ACTn,
		MPP17_SATA0_PRESENTn,
		MPP18_GPO,
		MPP19_GPO,
		MPP20_GE1_0,
		MPP21_GE1_1,
		MPP22_GE1_2,
		MPP23_GE1_3,
		MPP24_GE1_4,
		MPP25_GE1_5,
		MPP26_GE1_6,
		MPP27_GE1_7,
		MPP28_GPIO,
		MPP29_GPIO,
		MPP30_GE1_10,
		MPP31_GE1_11,
		MPP32_GE1_12,
		MPP33_GE1_13,
		MPP34_GPIO,
		MPP35_GPIO,
		MPP36_GPIO,
		MPP37_GPIO,
		MPP38_GPIO,
		MPP39_GPIO,
		MPP40_GPIO,
		MPP41_GPIO,
		MPP42_GPIO,
		MPP43_GPIO,
		MPP44_GPIO,
		MPP45_GPIO,
		MPP46_GPIO,
		MPP47_GPIO,
		MPP48_GPIO,
		MPP49_GPIO,
		0
	};
	kirkwood_mpp_conf(kwmpp_config, NULL);

	/*
	 * arch number of board
	 */
	gd->bd->bi_arch_number = MACH_TYPE_BUBBA3;

	/* adress of boot parameters */
	gd->bd->bi_boot_params = kw_sdram_bar(0) + 0x100;

	return 0;
}


#ifdef CONFIG_RESET_PHY_R
void mv_phy_88e1116_init(char *name)
{
	u16 reg;
	u16 devadr;

	printf("88E1116 Initializing on %s", name);
	if (miiphy_set_current_dev(name))
		return;

	/* command to read PHY dev address */
	if (miiphy_read(name, 0xEE, 0xEE, &devadr)) {
		printf("\n Err..%s could not read PHY dev address\n",
			__FUNCTION__);
		return;
	}

	printf(" @%02x", devadr);

	/*
	 * Enable RGMII delay on Tx and Rx for CPU port
	 * Ref: sec 4.7.2 of chip datasheet
	 */
	miiphy_write(name, devadr, MV88E1116_PGADR_REG, 2);
	miiphy_read(name, devadr, MV88E1116_MAC_CTRL_REG, &reg);
	reg |= (MV88E1116_RGMII_RXTM_CTRL | MV88E1116_RGMII_TXTM_CTRL);
	miiphy_write(name, devadr, MV88E1116_MAC_CTRL_REG, reg);
	miiphy_write(name, devadr, MV88E1116_PGADR_REG, 0);

	/* Invert PHY led action */
	miiphy_write(name, devadr, MV88E1116_PGADR_REG, 3);
	miiphy_write(name, devadr, 0x10, 0x0071);
	miiphy_write(name, devadr, 0x11, 0x4415);

	/* reset the phy */
	miiphy_reset(name, devadr);
	printf(" done\n");
}


/* Configure and enable Switch and PHY */
void reset_phy(void)
{

	/* configure and initialize PHY */
	mv_phy_88e1116_init("egiga0");
	mv_phy_88e1116_init("egiga1");

}

/* Button logic */
int b3_probe_button(void) {
	struct kwgpio_registers *gpio1reg =
			(struct kwgpio_registers *)KW_GPIO1_BASE;

	return readl(&gpio1reg->din) & B3_POWER_BUTTON;
}


/*
 * spawn a cyclic 20sec timer
 */
int b3_timer_init (void) {
	struct kwtimer_registers *timerreg =
			(struct kwtimer_registers *)KW_TIMER_BASE;

	/* internal clk cycle is 200MHz */
 	writel (&timerreg->ctl, readl(&timerreg->ctl) | 0x03); /* enable + auto reload of Timer0 */
 	writel (&timerreg->rel0, 0x2FFFFFFF); /* CPUTimer0Rel - 4.03s repetitive delay */
	writel (&timerreg->val0, 0x0FFFFFFF); /* CPUTimer0 - 1.36s */

 	writel (&timerreg->ctl, readl(&timerreg->ctl) | (0x03<<2)); /* enable + auto reload of Timer1 */
 	writel (&timerreg->rel1, 0x2FFFFFFF); /* CPUTimer0Rel - 4.03s repetitive delay */
	writel (&timerreg->val1, 0x2FFFFFFF); /* CPUTimer0 - 4.03s */

 	writel (&timerreg->ctl, 0x0); /* disable watchdog */

}




/*
 * enable irq for a specific gpio pin
 */
int b3_gpio1_irq_enable (u32 mask, u8 enable) {
 	struct kwgpio_registers *gpio1reg =
			(struct kwgpio_registers *)KW_GPIO1_BASE;

	/* disable level interrupt */
	writel(readl(&gpio1reg->irq_level) & ~B3_POWER_BUTTON, &gpio1reg->irq_level);

	/* unmask edge event interrupt*/
	if (enable)
		writel(readl(&gpio1reg->irq_mask) | B3_POWER_BUTTON, &gpio1reg->irq_mask);
	else
		writel(readl(&gpio1reg->irq_mask) & ~B3_POWER_BUTTON, &gpio1reg->irq_mask);
	return readl(&gpio1reg->irq_mask) & B3_POWER_BUTTON;
 }


/*
 * use the CP15 instruction for deepsleep (wait-for-interrupt) and power saveings
 */
inline void b3_wait_for_interrupt (void) {
	printf ("Jag drivar runt nu!\n");

#if 0
	if (b3_gpio1_irq_enable (B3_POWER_BUTTON, 1))
		printf ("successfully unmasked power button mask.\n");
	else
		printf ("failed to unmask power button mask.\n");
#endif
	__asm__ __volatile__ (
						"MOV R0, #0\n"
						"MCR p15, 0, r0, c7, c0, 4"
						 : : : "r0");
	printf ("hey! ho! - let's go!\n");
}

/*
 * Make sure magic value is cleared
 *
 * This is done to make sure that a powercycle
 * while in suspend does start up as a normal
 * power on
 */
static int b3_zero_magic (void)
{
	struct spi_flash *flash;
	int ret;

	flash = spi_flash_probe(CONFIG_ENV_SPI_BUS,
			CONFIG_ENV_SPI_CS, CONFIG_ENV_SPI_MAX_HZ, SPI_MODE_3);

	if (!flash) {
		printf("Failed to initialize SPI flash for magic erase\n");
		return 1;
	}

	ret = spi_flash_erase(flash, MAGIC_OFFSET, MAGIC_BLOCKSIZE);
	if (ret) {
		printf("Erase flash for magic failed\n");
		return 1;
	}

	spi_flash_free(flash); //FIXME I think we leak a reference to spi_flash above if flash_erase fails
	return ret;
}

/*
 * This function checks for a magic set by fx linux
 * It's used to determine if the system should start
 * or if this is a controlled "shutdown"
 * In the case of a shutdown. BUBBATWO hangs in the
 * bootloader waiting for a buttonpress to start again.
 *
 * When magic is read out the register i zeroed to
 * avoid the value being a leftover from a powercycle.
 */
static int b3_checkreboot (void)
{
	static int res = -1;

	if (res==-1){

		if ( in_le32(KW_DEFADR_SPIF+MAGIC_OFFSET) == REBOOT_MAGIC){
			b3_zero_magic();
			res=1;
		}else{
			res=0;
		}

	}

	return res;
}

/*
 * "suspend" Bubba. Shutdown as much as possible
 * and wait for keypress.
 */
void b3_suspend (void)
{
#if 0
	struct kwcpu_registers *cpureg =
	    (struct kwcpu_registers *)KW_CPU_REG_BASE;
#endif

	/*
	 * We need satahc up to be able to spin down for suspend
	 */
	sata_initialize();

	b3_led_off ();
	mv_sata_spin_down(0);
	mv_sata_spin_down(1);

	printf ("\nSuspending. Press button to restart\n");
	while (b3_probe_button()){
		b3_wait_for_interrupt();
	}

	b3_set_led_color(LED_BOOT);
	b3_led_on(0,0,0,0,0,0);

	mv_sata_spin_up(0);
	mv_sata_spin_up(1);
}

/* Probe and set bootalternatives */
int misc_init_r (void)
{
	char *cmd=NULL;

	if(b3_checkreboot()){
		/* This is a shutdown
		 * Run suspend as preboot
		 */
		 setenv("preboot","bubbacmd suspend");
		 cmd = getenv ("bootalt1");
	}else{

		if(b3_probe_button()){
			cmd = getenv ("bootalt1");
			b3_set_led_color(LED_BOOT);
			b3_led_on(0,0,0,0,0,0);
			setenv("button","0");
		}else{
			cmd = getenv ("bootalt2");
			b3_set_led_color(LED_GREEN);
			b3_led_on(0,0,0,0,0,0);
			setenv("button","1");
		}

		/*
		 * Initialize sata here since our shell
		 * is unaware of its previous state.
		 */
		sata_initialize();

	}

	if(cmd){
		setenv("bootcmd",cmd);
	}
	return 0;
}

void do_irq (void) {
	printf ("foobarz - IRQ detected\n");
}


inline int arch_interrupt_init (void) {
	return 0;
}
#endif
