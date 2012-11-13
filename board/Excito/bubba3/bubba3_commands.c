/*
 * (C) Copyright 2010
 * Excito elektronik i Skåne AB <www.excito.com>
 * Written-by: Tor Krill <tor@excito.com>
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
#include <config.h>
#include <command.h>
#include <asm/arch/kirkwood.h>

#include "bubba3.h"
#include "bubba3_led.h"

int do_bubbacmd (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[]){
	int ret = 0;


	switch (argc) {
	case 2:
		if (strncmp (argv[1], "button", 6) == 0) {
			ret = b3_probe_button();
		}else if (strncmp (argv[1], "led_on", 6) == 0) {
			ret = b3_led_on(0,0,0,0,0,0);
		}else if (strncmp (argv[1], "led_off", 6) == 0) {
			ret = b3_led_off();
		}else if (strncmp (argv[1], "toggle_color", 12) == 0) {
			ret = b3_led_toggle();
		}else if (strncmp (argv[1], "led_blink", 9) == 0) {
			ret = b3_led_on(250,250,0,0,0,0);
		}else if (strncmp (argv[1], "suspend", 7) == 0) {
			b3_suspend ();
		}else{
			printf ("Usage:\n%s\n", cmdtp->usage);
			return 1;
		}
		break;
	case 3:
		if (strncmp(argv[1],"color", 5) == 0) {

			if( strncmp(argv[2],"red",3) == 0) {
				b3_set_led_color(LED_RED);
			}else if( strncmp(argv[2],"blue",4) == 0) {
				b3_set_led_color(LED_BLUE);
			}else if( strncmp(argv[2],"green",5) == 0) {
				b3_set_led_color(LED_GREEN);
			}else if( strncmp(argv[2],"boot",4) == 0) {
				b3_set_led_color(LED_BOOT);
			}else if( strncmp(argv[2],"all",3) == 0) {
				b3_set_led_color(
					LED_GREEN | LED_BLUE | LED_RED);
			}else{
				printf ("Usage:\n%s\n", cmdtp->usage);
				return 1;
			}

		}else{
			printf ("Usage:\n%s\n", cmdtp->usage);
			return 1;
		}
		break;
	default:
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	return ret;
}

U_BOOT_CMD (bubbacmd, 3, 1, do_bubbacmd,
		"Board specific commands for Bubba",
		"bubbacmd button - read out button status\n"
		"bubbacmd toggle_color - change color for front led\n"
		"bubbacmd led_blink - Blink front led\n"
		"bubbacmd color [red|blue|green|boot|all] - set led color\n"
		"bubbacmd led_off - Turn led off\n"
		"bubbacmd led_on - Turn led on\n"
		"bubbacmd suspend - suspend bubba\n");
