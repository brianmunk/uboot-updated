/*
 * (C) Copyright 2010
 * Excito elektronik i Skåne AB <www.excito.com>
 * Written-by: PA Nilsson <pa@excito.com>
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
#include <asm/arch-kirkwood/kirkwood.h>
#include <asm/arch-kirkwood/cpu.h>
#include <asm/arch/kirkwood.h>
#include <asm/arch/mpp.h>
#include <asm/io.h>
#include "bubba3.h"
#include "bubba3_led.h"

static int led_color = LED_RED;


void b3_set_led_color(int rgb) {

	struct kwgpio_registers *gpio1reg =
			(struct kwgpio_registers *)KW_GPIO1_BASE;

	u32 tmp = readl(&gpio1reg->dout);
	tmp &= ~(B3_FRONT_LED_RED|B3_FRONT_LED_GREEN|B3_FRONT_LED_BLUE);
	if(rgb & LED_RED){
		tmp |= B3_FRONT_LED_RED;
	}
	if(rgb & LED_GREEN){
		tmp |= B3_FRONT_LED_GREEN;
	}
	if(rgb & LED_BLUE){
		tmp |= B3_FRONT_LED_BLUE;
	}
	writel(tmp,&gpio1reg->dout);
}

static void set_led_interval(int s){
	struct kwgpio_registers *gpio1reg =
			(struct kwgpio_registers *)KW_GPIO1_BASE;

	u32 tmp = readl(&gpio1reg->dout);
	if(s & 0x01){
		tmp |= B3_LED_INTERVAL;
	}else{
		tmp &= ~B3_LED_INTERVAL;
	}
	writel(tmp,&gpio1reg->dout);
}

static void led_reset(void){
	set_led_interval(0);
	udelay(1800);
	set_led_interval(1);
	udelay(10);
	set_led_interval(0);
	udelay(1800);
}

static void led_train_start(void){
	// training start
	set_led_interval(1);
	udelay(10);
	set_led_interval(0);
	udelay(10);
	set_led_interval(1);
	udelay(10);
	set_led_interval(0);
	udelay(1800);
}

static void led_train_end(void){
	// training end
	set_led_interval(1);
	udelay(10);
	set_led_interval(0);
	udelay(10);
	set_led_interval(1);
	udelay(10);
	set_led_interval(0);
	udelay(10);
	set_led_interval(1);
	udelay(10);
	set_led_interval(0);
	udelay(1800);
}

int b3_led_on(int on_time_ms, int off_time_ms, int second_on_ms,
		int second_off_ms, int third_on_ms, int third_off_ms) {

	led_reset();

	led_train_start();

	if(on_time_ms || off_time_ms) {
		// pattern
		set_led_interval(1);
		udelay(on_time_ms*1000);
		set_led_interval(0);
		udelay(off_time_ms*1000);
		if(second_on_ms || second_off_ms) {
			// pattern
			set_led_interval(1);
			udelay(second_on_ms*1000);
			set_led_interval(0);
			udelay(second_off_ms*1000);
			if(third_on_ms || third_off_ms) {
				// pattern
				set_led_interval(1);
				udelay(third_on_ms*1000);
				set_led_interval(0);
				udelay(third_off_ms*1000);
			}
		}
	} else {
		// bypass mode
	}

	led_train_end();

	// run
	set_led_interval(1);

	return 0;

}

int b3_led_off(void){

	b3_set_led_color(LED_NONE);

	led_reset();

	return 0;
}

int b3_led_toggle(void) {
	if(led_color == LED_RED) {
		led_color = LED_BLUE;
	} else {
		led_color = led_color << 1;
	}
	b3_set_led_color(led_color);
	return 0;
}
