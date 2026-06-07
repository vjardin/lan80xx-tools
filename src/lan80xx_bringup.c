// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright 2026 Free Mobile — Vincent Jardin
 *
 * lan80xx_bringup.c — replay a LAN80XX register sequence.
 *
 * Brings up a LAN80XX-family PHY by replaying a recorded register
 * sequence — no MEPA, MESA, or kernel PHY framework at runtime. The
 * sequence is recorded ONCE from a known-good MEPA bring-up via the
 * lan80xx-spid event log (lan80xx-spid -L + scripts/events2seq.py),
 * then this tool replays it on boards.
 * It works either standalone on a spidev node or through a
 * lan80xx-spid proxy socket.
 */

#define _DEFAULT_SOURCE		/* S_ISSOCK with -std=c17 */

#include <err.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <linux/spi/spidev.h>

#define MMD_DEVICE_INFO	0x1Eu
#define REG_DEVICE_ID	0x0000u
#define MAX_PADDING	15

#define V_POLL_MS	10
#define V_TIMEOUT_MS	1000

struct spi_ctx {
	int		fd;
	uint32_t	hz;
	unsigned int	padding;
	bool		proxy;
	uint32_t	seq;
};

/*
 * lan80xx-spid proxy wire protocol, L1 subset.
 * See Canonical definition:spiproxy.h (version 1); duplicated so
 * this file keeps the repo's no-dependency property.
 * TODO: include spiproxy.h
 */
#define PROXY_VER	1
#define PROXY_READ	1
#define PROXY_WRITE	2
#define PROXY_RESP	0x80
#define PROXY_ST_OK	0

struct proxy_hdr {
	uint8_t		ver;
	uint8_t		type;
	uint16_t	flags;
	uint32_t	seq;
	uint32_t	len;
};

struct proxy_op {
	uint8_t		slice;
	uint8_t		write;
	uint8_t		mmd;
	uint8_t		rsvd;
	uint16_t	reg;
	uint16_t	rsvd2;
	uint32_t	val;
};

static int proxy_rw(struct spi_ctx *s, bool wr, unsigned int slice,
		    unsigned int mmd, uint16_t reg, uint32_t *val)
{
	struct {
		struct proxy_hdr h;
		struct proxy_op op;
	} msg;
	uint8_t type = wr ? PROXY_WRITE : PROXY_READ;
	ssize_t n;

	memset(&msg, 0, sizeof(msg));
	msg.h.ver = PROXY_VER;
	msg.h.type = type;
	msg.h.seq = ++s->seq;
	msg.h.len = sizeof(msg.op);
	msg.op.slice = (uint8_t)slice;
	msg.op.write = wr;
	msg.op.mmd = (uint8_t)mmd;
	msg.op.reg = reg;
	msg.op.val = wr ? *val : 0;
	if (send(s->fd, &msg, sizeof(msg), 0) < 0) {
		warn("proxy send");
		return -1;
	}
	n = recv(s->fd, &msg, sizeof(msg), 0);
	if (n < (ssize_t)sizeof(msg.h) || msg.h.seq != s->seq ||
	    msg.h.type != (type | PROXY_RESP) || msg.h.flags != PROXY_ST_OK) {
		warnx("proxy %s failed", wr ? "write" : "read");
		return -1;
	}
	if (!wr)
		*val = msg.op.val;
	return 0;
}

static void spi_fill_addr(uint8_t *tx, bool wr, unsigned int slice,
			  unsigned int mmd, uint16_t reg)
{
	uint32_t a = ((slice & 0x3u) << 21) | ((mmd & 0x1fu) << 16) | reg;

	tx[0] = (uint8_t)((wr ? 0x80u : 0) | (a >> 16));
	tx[1] = (uint8_t)(a >> 8);
	tx[2] = (uint8_t)a;
}

static int raw_rw(struct spi_ctx *s, bool wr, unsigned int slice,
		  unsigned int mmd, uint16_t reg, uint32_t *val)
{
	uint8_t tx[3 + MAX_PADDING + 4], rx[sizeof(tx)];
	uint8_t tx2[sizeof(tx)], rx2[sizeof(tx)];
	struct spi_ioc_transfer trs[2];
	unsigned int d = 3;	/* response follows the address bytes */

	memset(tx, 0xff, sizeof(tx));
	memset(trs, 0, sizeof(trs));
	spi_fill_addr(tx, wr, slice, mmd, reg);
	trs[0].tx_buf = (unsigned long)tx;
	trs[0].rx_buf = (unsigned long)rx;
	trs[0].len = 3 + (wr ? 4 : s->padding + 4);
	trs[0].speed_hz = s->hz;
	trs[0].bits_per_word = 8;
	if (wr) {
		tx[3] = (uint8_t)(*val >> 24);
		tx[4] = (uint8_t)(*val >> 16);
		tx[5] = (uint8_t)(*val >> 8);
		tx[6] = (uint8_t)*val;
		if (ioctl(s->fd, SPI_IOC_MESSAGE(1), trs) < 1) {
			warn("SPI_IOC_MESSAGE");
			return -1;
		}
		return 0;
	}
	memset(tx2, 0xff, sizeof(tx2));
	spi_fill_addr(tx2, false, slice, MMD_DEVICE_INFO, REG_DEVICE_ID);
	trs[0].cs_change = 1;
	trs[0].delay_usecs = (uint16_t)(2000000 / s->hz + 1);
	trs[1].tx_buf = (unsigned long)tx2;
	trs[1].rx_buf = (unsigned long)rx2;
	trs[1].len = 3 + s->padding + 4;
	trs[1].speed_hz = s->hz;
	trs[1].bits_per_word = 8;
	if (ioctl(s->fd, SPI_IOC_MESSAGE(2), trs) < 1) {
		warn("SPI_IOC_MESSAGE");
		return -1;
	}
	*val = ((uint32_t)rx2[d] << 24) | ((uint32_t)rx2[d + 1] << 16) |
	       ((uint32_t)rx2[d + 2] << 8) | rx2[d + 3];
	return 0;
}

static int reg_rw(struct spi_ctx *s, bool wr, unsigned int slice,
		  unsigned int mmd, uint16_t reg, uint32_t *val)
{
	return s->proxy ? proxy_rw(s, wr, slice, mmd, reg, val)
			: raw_rw(s, wr, slice, mmd, reg, val);
}

static int dev_open(struct spi_ctx *s, const char *dev, uint32_t hz,
		    unsigned int padding)
{
	struct sockaddr_un sa = { .sun_family = AF_UNIX };
	uint8_t mode = SPI_MODE_0, bits = 8;
	struct stat st;

	s->hz = hz;
	s->padding = padding;
	s->proxy = false;
	s->seq = 0;
	if (stat(dev, &st) == 0 && S_ISSOCK(st.st_mode)) {
		s->fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
		snprintf(sa.sun_path, sizeof(sa.sun_path), "%s", dev);
		if (s->fd < 0 ||
		    connect(s->fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
			warn("connect %s", dev);
			return -1;
		}
		setsockopt(s->fd, SOL_SOCKET, SO_RCVTIMEO,
			   &(struct timeval){ .tv_sec = 2 },
			   sizeof(struct timeval));
		s->proxy = true;
		return 0;
	}
	s->fd = open(dev, O_RDWR);
	if (s->fd < 0) {
		warn("open %s", dev);
		return -1;
	}
	if (ioctl(s->fd, SPI_IOC_WR_MODE, &mode) < 0 ||
	    ioctl(s->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
	    ioctl(s->fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) {
		warn("spidev ioctl");
		return -1;
	}
	return 0;
}

static void msleep(unsigned int ms)
{
	struct timespec ts = { ms / 1000, (long)(ms % 1000) * 1000000L };

	nanosleep(&ts, NULL);
}

static void usage(const char *argv0)
{
	fprintf(stderr,
		"Usage: %s <spidev|proxy-socket> <sequence-file> [-s hz] [-p pad] [-v]\n"
		"Replays a recorded LAN80XX register sequence (see file header\n"
		"comment for the format; record with lan80xx-spid -L +\n"
		"scripts/events2seq.py). -s/-p apply to raw spidev only.\n",
		argv0);
}

int main(int argc, char **argv)
{
	struct spi_ctx s;
	FILE *fp;
	char line[256];
	uint32_t hz = 5000000, val, mask, expect;
	unsigned int slice, mmd, reg, ms, padding = 1, lineno = 0;
	unsigned int n_w = 0, n_v = 0, elapsed;
	bool verbose = false;
	char opc;
	int opt;

	if (argc < 3) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}
	optind = 3;
	while ((opt = getopt(argc, argv, "s:p:vh")) != -1) {
		switch (opt) {
		case 's':
			hz = (uint32_t)strtoul(optarg, NULL, 0);
			break;
		case 'p':
			padding = (unsigned int)strtoul(optarg, NULL, 0);
			break;
		case 'v':
			verbose = true;
			break;
		default:
			usage(argv[0]);
			return EXIT_FAILURE;
		}
	}
	if (padding > MAX_PADDING) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}
	fp = fopen(argv[2], "r");
	if (fp == NULL)
		err(EXIT_FAILURE, "%s", argv[2]);
	if (dev_open(&s, argv[1], hz, padding) < 0)
		return EXIT_FAILURE;
	printf("Replaying %s on %s%s\n", argv[2], argv[1],
	       s.proxy ? " (lan80xx-spid proxy)" : "");

	while (fgets(line, sizeof(line), fp) != NULL) {
		lineno++;
		if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
			continue;
		opc = line[0];
		if (opc == 'W' &&
		    sscanf(line + 1, " %u %x %x %x", &slice, &mmd, &reg,
			   &val) == 4) {
			if (verbose)
				printf("W s%u %02x %04x = %08x\n",
				       slice, mmd, reg, val);
			if (reg_rw(&s, true, slice, mmd, (uint16_t)reg, &val))
				goto fail;
			n_w++;
		} else if (opc == 'R' &&
			   sscanf(line + 1, " %u %x %x", &slice, &mmd,
				  &reg) == 3) {
			if (reg_rw(&s, false, slice, mmd, (uint16_t)reg, &val))
				goto fail;
			printf("R s%u %02x %04x = %08x\n", slice, mmd, reg,
			       val);
		} else if (opc == 'P' &&
			   sscanf(line + 1, " %u", &ms) == 1) {
			if (verbose)
				printf("P %u ms\n", ms);
			msleep(ms);
		} else if (opc == 'V' &&
			   sscanf(line + 1, " %u %x %x %x %x", &slice, &mmd,
				  &reg, &mask, &expect) == 5) {
			for (elapsed = 0;; elapsed += V_POLL_MS) {
				/* double-read: clear latched-low bits */
				if (reg_rw(&s, false, slice, mmd,
					   (uint16_t)reg, &val) ||
				    reg_rw(&s, false, slice, mmd,
					   (uint16_t)reg, &val))
					goto fail;
				if ((val & mask) == expect)
					break;
				if (elapsed >= V_TIMEOUT_MS) {
					fprintf(stderr,
						"FAIL line %u: V s%u %02x %04x = %08x, want (&%08x) == %08x\n",
						lineno, slice, mmd, reg, val,
						mask, expect);
					goto fail;
				}
				msleep(V_POLL_MS);
			}
			printf("V s%u %02x %04x = %08x ok\n", slice, mmd, reg,
			       val);
			n_v++;
		} else {
			fprintf(stderr, "FAIL line %u: cannot parse: %s",
				lineno, line);
			goto fail;
		}
	}
	fclose(fp);
	printf("done: %u writes, %u verifies ok\n", n_w, n_v);
	return EXIT_SUCCESS;

fail:
	fclose(fp);
	fprintf(stderr, "aborted at line %u (%u writes done)\n", lineno, n_w);
	return EXIT_FAILURE;
}
