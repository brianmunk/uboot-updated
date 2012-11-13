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
#ifndef BUBBA3_LED_H
#define BUBBA3_LED_H

#define LED_NONE	0x00
#define LED_RED		0x04
#define LED_GREEN	0x02
#define LED_BLUE	0x01
#define LED_BOOT	(LED_RED | LED_BLUE)


void b3_set_led_color(int rgb);

int b3_led_on(int on_time_ms, int off_time_ms, int second_on_ms,
		int second_off_ms, int third_on_ms, int third_off_ms);

int b3_led_off(void);

int b3_led_toggle(void);

#endif
