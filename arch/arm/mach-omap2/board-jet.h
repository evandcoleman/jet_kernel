/*
 * arch/arm/mach-omap2/board-jet.h
 *
 * Copyright (C) 2011 Reconinstruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MACH_OMAP_BOARD_44XX_SDP_H
#define _MACH_OMAP_BOARD_44XX_SDP_H

int jet_touch_init(void);
int jet_sensor_init(void);
int jet_panel_init(void);
int jet_pwbutton_init(void);
void jet_modem_init(bool force_mux);
void omap4_create_board_props(void);
void omap4_power_init(void);
void board_serial_init(void);


#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
extern struct usbhs_omap_board_data usbhs_bdata;
#endif
#endif
