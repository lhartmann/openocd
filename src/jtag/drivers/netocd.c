/***************************************************************************
 *   Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/* 2014-12: Addition of the SWD protocol support is based on the initial work
 * on bcm2835gpio.c by Paul Fertser and modifications by Jean-Christian de Rivaz. */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include "bitbang.h"
#include <string.h>

/* TCP Networing */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netdb.h>

/* Communication protocol: */
/*   Bits [7:6] select command set: */
#define NOCD_CMD_SET_M  0xC0
#define NOCD_CMD_SET(i) (((i)&3)<<6)
/*   SET0 uses [5:0] as raw GPIO bits: */
#define NOCD0_TMS         0x01
#define NOCD0_TCK         0x02
#define NOCD0_TDI         0x04
#define NOCD0_TDO         0x08
#define NOCD0_SWCLK       0x10
#define NOCD0_SWDIO       0x20

/* SET1 uses [1:0] as raw GPIO bits: */
#define NOCD1_TRST        0x01
#define NOCD1_SRST        0x02

/* SET3 uses [5:0] as individual commands */
#define NOCD3_READ0       0xC0 // Read back set 0
#define NOCD3_READ1       0xC1 // Read back set 1
#define NOCD3_LED_OFF     0xC2
#define NOCD3_LED_ON      0xC3
#define NOCD3_SWDIO_OUT   0xC4
#define NOCD3_SWDIO_IN    0xC5
#define NOCD3_JTAG_XCH_W  0xC8 // Write-only
#define NOCD3_JTAG_XCH_R  0xC9 // Write and read
#define NOCD3_SWD_XCH_W   0xCA
#define NOCD3_SWD_XCH_R   0xCB

/* Static local config data */
/*   Remote host name or IP */
static char netocd_host[128] = "";
/*   Decimal integer or service name (e.g. "HTTP" or "80") */
static char netocd_port[32] = "";
static bool netocd_remote_exchange = false;

/* Remote socket connection */
static int  netocd_fd = -1;
/* Last output of set0 */
static uint32_t netocd_last_0 = NOCD0_TMS;
/* Last output of set1 */
static uint32_t netocd_last_1 = NOCD1_SRST | NOCD1_SRST;

#define BUFFER_SIZE 16
static uint8_t netocd_buffer[BUFFER_SIZE];
static int netocd_buffer_pos = 0;

/* Connects to host-service.
   Returns socket fd on success, or -1 on error. */
static int tcp_connect(const char *host, const char *service)
{
	struct addrinfo *book, *i;
	int r = getaddrinfo(host, service, 0, &book);
	if (r) {
		/* Failed to find host/service as selected */
		return -1;
	}

	for (i=book; i; i = i->ai_next) {
		r = socket(i->ai_family, i->ai_socktype, i->ai_protocol);
		if (r == -1) continue;

		if (connect(r, i->ai_addr, i->ai_addrlen) != -1)
			/* YAY! Found him! */
			break;

		close(r);
		r = -1;
	}

	freeaddrinfo(book);

	/* Failed. */
	if (r == -1) return r;

	/* Try disabling Nagle */
	int flag = 1;
	flag = setsockopt(r, /* socket affected */
		IPPROTO_TCP,     /* set option at TCP level */
		TCP_NODELAY,     /* name of option */
		(char *) &flag,  /* the cast is historical cruft */
		sizeof(int));    /* length of option value */
	if (flag < 0)
		LOG_WARNING("NETOCD: Failed to disable Nagle, will be slow!");

	return r;
}

static void netocd_buffer_flush(void) {
	if (write(netocd_fd, netocd_buffer, netocd_buffer_pos) != netocd_buffer_pos) {
		LOG_ERROR("NETOCD: Broken link on write.");
		exit(1);
	}
	LOG_DEBUG("NETOCD: Flush.");
	netocd_buffer_pos = 0;
}

static void netocd_put_byte(uint8_t byte)
{
	netocd_buffer[netocd_buffer_pos++] = byte;

	if (netocd_buffer_pos == BUFFER_SIZE)
		netocd_buffer_flush();
}

static void netocd_put_u16(uint16_t x)
{
	netocd_put_byte(x >>  0);
	netocd_put_byte(x >>  8);
}

/* static void netocd_put_u32(uint32_t x)
{
	netocd_put_byte(x >>  0);
	netocd_put_byte(x >>  8);
	netocd_put_byte(x >> 16);
	netocd_put_byte(x >> 24);
} */

static void netocd_put(uint8_t byte) {
	uint8_t mask = byte & ~NOCD_CMD_SET_M;

	switch (byte & NOCD_CMD_SET_M) {
		case NOCD_CMD_SET(0):
			if (netocd_last_0 != mask)
				netocd_put_byte(byte);
			netocd_last_0 = mask;
			break;

		case NOCD_CMD_SET(1):
			if (netocd_last_1 != mask)
				netocd_put_byte(byte);
			netocd_last_1 = mask;
			break;
			break;

		case NOCD_CMD_SET(2):
			break;

		case NOCD_CMD_SET(3):
			netocd_put_byte(byte);
			break;
	}
}

static uint8_t netocd_get_byte(void) {
	uint8_t byte;
	int r;
	netocd_buffer_flush();
	do {
		r = read(netocd_fd, &byte, 1);
		if (r < 0) {
			LOG_ERROR("NETOCD: broken link on read.");
			exit(1);
		}
	} while (r != 1);
	return byte;
}

COMMAND_HANDLER(netocd_handle_host)
{
	if (CMD_ARGC == 1)
	{
		memset(netocd_host, 0, sizeof(netocd_host));
		strncpy(netocd_host, CMD_ARGV[0], sizeof(netocd_host)-1);
	}

	command_print(CMD_CTX, "NETOCD: Host set to %s", netocd_host);
	return ERROR_OK;
}

COMMAND_HANDLER(netocd_handle_port)
{
	if (CMD_ARGC == 1)
	{
		memset(netocd_port, 0, sizeof(netocd_port));
		strncpy(netocd_port, CMD_ARGV[0], sizeof(netocd_port)-1);
	}

	command_print(CMD_CTX, "NETOCD: Port/service set to %s", netocd_port);
	return ERROR_OK;
}

COMMAND_HANDLER(netocd_handle_remote_exchange)
{
	if (CMD_ARGC == 1)
		netocd_remote_exchange = atoi(CMD_ARGV[0]) || !strcmp("true", CMD_ARGV[0]) || !strcmp("enable", CMD_ARGV[0]);

	command_print(CMD_CTX, "NETOCD: Remote exchange %s", netocd_remote_exchange ? "enabled" : "disabled");
	return ERROR_OK;
}

static const struct command_registration netocd_command_handlers[] = {
	{
		.name = "netocd_host",
		.handler = &netocd_handle_host,
		.mode = COMMAND_CONFIG,
		.help = "IP address or hostname for the NOCD8266.",
	},
	{
		.name = "netocd_port",
		.handler = &netocd_handle_port,
		.mode = COMMAND_CONFIG,
		.help = "TCP port number to connect to.",
	},
	{
		.name = "netocd_remote_exchange",
		.handler = &netocd_handle_remote_exchange,
		.mode = COMMAND_CONFIG,
		.help = "Use remote bitbang_exchange(...) implementation.",
		.usage = "true | false ",
	},
	COMMAND_REGISTRATION_DONE
};

static void netocd_swdio_drive(bool is_output)
{
	if (is_output)
		netocd_put_byte(NOCD3_SWDIO_OUT);
	else
		netocd_put_byte(NOCD3_SWDIO_IN);
}

static void netocd_blink(int led_on)
{
	if (led_on)
		netocd_put_byte(NOCD3_LED_ON);
	else
		netocd_put_byte(NOCD3_LED_OFF);
}

static int netocd_swdio_read(void)
{
	netocd_put_byte(NOCD3_READ0);
	return netocd_get_byte() & NOCD0_SWDIO ? 1 : 0;
}

static void netocd_swdio_write(int swclk, int swdio)
{
	uint8_t byte = netocd_last_0;

	if (swclk)
		byte |= NOCD0_SWCLK;
	else
		byte &= ~NOCD0_SWCLK;

	if (swdio)
		byte |= NOCD0_SWDIO;
	else
		byte &= ~NOCD0_SWDIO;

	netocd_put(byte);
}

/*
 * Bitbang interface read of TDO
 *
 */
static int netocd_read(void)
{
	netocd_put_byte(NOCD3_READ0);
	return netocd_get_byte() & NOCD0_TDO ? 1 : 0;
}

/*
 * Bitbang interface write of TCK, TMS, TDI
 *
 */
static void netocd_write(int tck, int tms, int tdi)
{
	if (swd_mode) {
		netocd_swdio_write(tck, tdi);
		return;
	}

	uint8_t byte = netocd_last_0;

	if (tck)
		byte |= NOCD0_TCK;
	else
		byte &= ~NOCD0_TCK;

	if (tms)
		byte |= NOCD0_TMS;
	else
		byte &= ~NOCD0_TMS;

	if (tdi)
		byte |= NOCD0_TDI;
	else
		byte &= ~NOCD0_TDI;

	netocd_put(byte);
}

/*
 * Bitbang interface to manipulate reset lines SRST and TRST
 *
 * (1) assert or (0) deassert reset lines
 */
static void netocd_reset(int trst, int srst)
{
	uint8_t byte = netocd_last_1 | NOCD_CMD_SET(1);
	byte &= ~(NOCD1_SRST | NOCD1_TRST);

	if (trst) byte |= NOCD1_TRST;
	if (srst) byte |= NOCD1_SRST;

	netocd_put(byte);
}

// Small exchange, remote, offset<=7, bit_cnt<=128*8
static void netocd_exchange_small(bool rnw, uint8_t buf[], uint8_t offset, uint16_t bit_cnt) {
	LOG_DEBUG("NETOCD: exchange_small(%d, %p, %d, %d).", rnw, buf, offset, bit_cnt);
	// REQUEST:
	// Select proper format
	if (swd_mode)
		netocd_put_byte(rnw ? NOCD3_SWD_XCH_R : NOCD3_SWD_XCH_W);
	else
		netocd_put_byte(rnw ? NOCD3_JTAG_XCH_R : NOCD3_JTAG_XCH_W);

	// Parameters
	netocd_put_byte(offset);
	netocd_put_u16(bit_cnt);

	// Data
	LOG_DEBUG("NETOCD: Write:");
	uint16_t bytes = (offset+bit_cnt+7) / 8;
	for (int i=0; i<bytes; ++i) {
		uint8_t byte = buf ? buf[i] : 0;
		LOG_DEBUG("NETOCD:   0x%02X", byte);
		netocd_put_byte(byte);
	}

	// RESPONSE, if reading:
	if (rnw)
	{
		LOG_DEBUG("NETOCD: reading...");
		for (int i=0; i<bytes; ++i)
		{
			uint8_t byte = netocd_get_byte();
			if (buf) buf[i] = byte;
			LOG_DEBUG("NETOCD:   0x%02X", byte);
		}
	}
}

static void netocd_exchange(bool rnw, uint8_t buf[], unsigned int offset, unsigned int bit_cnt) {
	LOG_DEBUG("NETOCD: exchange(%d, ..., %d, %d).", rnw, offset, bit_cnt);
	while (bit_cnt) {
		// Skip unnecessary or processed bytes
		if (buf)
			buf    += offset / 8;
		offset %= 8;

		// How many bits to exchange this loop
		int bits = bit_cnt+offset > 128*8 ? 128*8-offset : bit_cnt;

		// Do remote exchange
		netocd_exchange_small(rnw, buf, offset, bits);

		// Mark processed
		offset  += bits;
		bit_cnt -= bits;
	}
}

static int netocd_init(void);
static int netocd_quit(void);

static const char * const netocd_transports[] = { "jtag", "swd", NULL };

struct jtag_interface netocd_interface = {
	.name = "netocd",
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
	.transports = netocd_transports,
	.swd = &bitbang_swd,
	.commands = netocd_command_handlers,
	.init = netocd_init,
	.quit = netocd_quit,
};

static struct bitbang_interface netocd_bitbang = {
	.read = netocd_read,
	.write = netocd_write,
	.reset = netocd_reset,
	.swdio_read = netocd_swdio_read,
	.swdio_drive = netocd_swdio_drive,
	.blink = netocd_blink,
	.exchange = netocd_exchange,
};

static int netocd_init(void)
{
	LOG_INFO("NETOCD JTAG/SWD driver");

	bitbang_interface = &netocd_bitbang;
	if (!netocd_remote_exchange)
		netocd_bitbang.exchange = 0;

	netocd_fd = tcp_connect(netocd_host, netocd_port);
	if (netocd_fd < 0)
		return ERROR_JTAG_INIT_FAILED;

	if (swd_mode)
		bitbang_swd_switch_seq(JTAG_TO_SWD);
	else
		bitbang_swd_switch_seq(SWD_TO_JTAG);

	return ERROR_OK;
}

static int netocd_quit(void)
{
	LOG_INFO("NETOCD: Quitting on demmand.");
	if (netocd_fd > 0)
		close(netocd_fd);
	netocd_fd = -1;

	return ERROR_OK;
}

