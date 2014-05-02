/* OMAP Identity file for OMAP4 boards.
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * Based on
 * mach-omap2/board-44xx-tablet.c
 * mach-omap2/board-4430sdp.c
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <plat/omap-serial.h>
#include "mux.h"
/*
 * uncomment below if we want the external test machine to control wilink UART interface
*/
//#define CONIG_WILINK_UART_EXT
static struct omap_device_pad tablet_uart1_pads[] __initdata = {
#if defined(CONFIG_JET_V2)//we don't use uart1 for version 2
    {
        .name   = "mcspi1_cs2.mcspi1_cs2",
        .enable = OMAP_MUX_MODE7,
    },
    {
        .name   = "mcspi1_cs3.mcspi1_cs3",
        .enable = OMAP_MUX_MODE7,
    },
    {
        .name   = "uart3_cts_rctx.uart3_cts_rctx",
        .enable = OMAP_MUX_MODE7,
    },
    {
        .name   = "mcspi1_cs1.mcspi1_cs1",
        .enable = OMAP_MUX_MODE7,
    },
#else
    {
        .name   = "mcspi1_cs2.mcspi1_cs2",
        .enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
        .flags  = OMAP_DEVICE_PAD_REMUX,
        .idle   = OMAP_WAKEUP_EN | OMAP_PIN_OFF_INPUT_PULLUP |
              OMAP_MUX_MODE1,
    },
    {
        .name   = "mcspi1_cs3.mcspi1_cs3",
        .flags  = OMAP_DEVICE_PAD_REMUX,
        .enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
        .idle   = OMAP_PIN_OFF_INPUT_PULLUP | OMAP_MUX_MODE7,
    },
    {
        .name   = "uart3_cts_rctx.uart3_cts_rctx",
        .enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
    },
    {
        .name   = "mcspi1_cs1.mcspi1_cs1",
        .enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
    },
#endif
};
#ifdef CONIG_WILINK_UART_EXT
static struct omap_device_pad tablet_uart2_pads[] __initdata = {
	{
		.name	= "uart2_tx.uart2_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE7,
	},
	{
		.name	= "uart2_rx.uart2_rx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE7,
	},
	{
		.name	= "uart2_cts.uart2_cts",
		.enable	= OMAP_MUX_MODE7,
	},
	{
		.name	= "uart2_rts.uart2_rts",
		.enable	= OMAP_MUX_MODE7,
	},
};
#else

static struct omap_device_pad tablet_uart2_pads[] __initdata = {
#if defined(CONFIG_JET_V2)
	{
		.name	= "uart2_cts.uart2_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.flags  = OMAP_DEVICE_PAD_REMUX,
		.idle   = OMAP_WAKEUP_EN | OMAP_PIN_OFF_INPUT_PULLUP |
			  OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rts.uart2_rts",
		.flags  = OMAP_DEVICE_PAD_REMUX,
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
		.idle   = OMAP_PIN_OFF_INPUT_PULLUP | OMAP_MUX_MODE7,
	},
#endif
	{
		.name	= "uart2_tx.uart2_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rx.uart2_rx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};
#endif

static struct omap_device_pad tablet_uart3_pads[] __initdata = {
	{
		.name	= "uart3_tx_irtx.uart3_tx_irtx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_rx_irrx.uart3_rx_irrx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad tablet_uart4_pads[] __initdata = {
#if defined(CONFIG_JET_V2)//we don't use uart1 for version 2
	{
		.name	= "uart4_tx.uart4_tx",
		.enable	= OMAP_MUX_MODE7,
	},
	{
		.name	= "uart4_rx.uart4_rx",
		.enable	= OMAP_MUX_MODE7,
	},
#else
	{
		.name	= "uart4_tx.uart4_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart4_rx.uart4_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
#endif
};

static struct omap_uart_port_info tablet_uart_info_uncon __initdata = {
	.use_dma	= 0,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
	.wer = 0,
};

static struct omap_uart_port_info tablet_uart_info __initdata = {
	.use_dma	= 0,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
	.wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
};

static struct omap_uart_port_info tablet_wilink_uart_info __initdata = {
	.use_dma	= 0,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
	.wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
	.rts_mux_driver_control = 1,
};

void __init board_serial_init(void)
{
#if defined(CONFIG_JET_V2)
	omap_serial_init_port_pads(0, tablet_uart1_pads,
		ARRAY_SIZE(tablet_uart1_pads), &tablet_uart_info_uncon);
	omap_serial_init_port_pads(1, tablet_uart2_pads,
		ARRAY_SIZE(tablet_uart2_pads), &tablet_wilink_uart_info);
#else
	omap_serial_init_port_pads(0, tablet_uart1_pads,
		ARRAY_SIZE(tablet_uart1_pads), &tablet_wilink_uart_info);
	omap_serial_init_port_pads(1, tablet_uart2_pads,
		ARRAY_SIZE(tablet_uart2_pads), &tablet_uart_info);
#endif
	omap_serial_init_port_pads(2, tablet_uart3_pads,
		ARRAY_SIZE(tablet_uart3_pads), &tablet_uart_info);
	omap_serial_init_port_pads(3, tablet_uart4_pads,
		ARRAY_SIZE(tablet_uart4_pads), &tablet_uart_info_uncon);
}

