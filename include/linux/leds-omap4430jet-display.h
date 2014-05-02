/*
 * OMAP4430 JET display LED Driver
 *
 * Copyright (C) 2010 Texas Instruments
 * Copyright (C) 2013 Recon Instruments
 *
 * Author: Dan Murphy <DMurphy@ti.com>
 *         Gil Zhaiek <gil@reconinstruments.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __OMAP4430_JET_DISPLAY_LED__
#define __OMAP4430_JET_DISPLAY_LED__

struct omap4430_jet_disp_led_platform_data {
	void (*display_led_init)(void);
	void (*primary_display_set)(u8 value);
};

#endif  /*__OMAP4430_JET_DISPLAY_LED__*/
