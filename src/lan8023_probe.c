// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright 2026 Free Mobile — Vincent Jardin
 *
 * lan8023_probe.c — LAN8023 / LAN80XX SPI presence + identity probe.
 *
 * Check a LAN80XX-family PHY (LAN8023, LAN8044, etc.) over a Linux
 * spidev node and prints decoded contents of the chip's identity, fuse,
 * strap, MCU boot, and POST1 BIST registers ; it works without dependencies with
 * MEPA, MESA, or the kernel PHY framework. Useful as a first sanity
 * check when bringing up a board.
 */

#include <err.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define MMD_DEVICE_INFO			0x1Eu	/* MMD_ID_MCU_IO_MNGT_MISC */

#define REG_DEVICE_ID			0x0000u
#define REG_DEVICE_OTP_REV		0x0001u
#define REG_DEVICE_SILICON_REV		0x0002u
#define REG_DEVICE_FEATURE_DIS		0x0003u
#define REG_POST1_POST_STATUS		0x0080u
#define REG_MCU_BOOT_STATUS		0x01E0u
#define REG_STRAP_READ			0x0200u
#define REG_STRAP_OVERRIDE		0x0201u

/* DEVICE_FEATURE_DISABLE_REG bit layout — see MEPA src/lan80xx_private.c:55-60. */
#define FEAT_MACSEC_DIS			(1u << 0)
#define FEAT_1588_DIS			(1u << 1)
#define FEAT_25G_DIS			(1u << 2)
#define FEAT_QUAD_DIS			(1u << 3)	/* 1 = Dual, 0 = Quad */
#define FEAT_CLEARTAGS_DIS		(1u << 4)
#define FEAT_MPLS_DIS			(1u << 5)

/*
 * STRAP_READ_REG bits — see MEPA include/microchip/lan80xx_mcu.h:65-70
 * (DFU/SERDES_INIT) and src/lan80xx_private.h:138 (BIST_BYPASS). MEPA
 * does not name the remaining bits; surface them as raw if set.
 */
#define STRAP_DFU			(1u << 0)
#define STRAP_SERDES_INIT		(1u << 1)
#define STRAP_BIST_BYPASS		(1u << 3)
#define STRAP_NAMED_MASK		(STRAP_DFU | STRAP_SERDES_INIT | \
					 STRAP_BIST_BYPASS)

/*
 * POST1_POST_STATUS bits — see MEPA
 * src/regs/regs_lan80xx_mcu_io_mngt_misc.h:843 and following.
 */
#define POST1_DONE			(1u << 0)
#define POST1_SLICE0_PASS		(1u << 1)
#define POST1_SLICE1_PASS		(1u << 2)
#define POST1_SLICE2_PASS		(1u << 3)
#define POST1_SLICE3_PASS		(1u << 4)
#define POST1_BIST_BYPASS		(1u << 15)

/*
 * Documented MCU boot-status code — see MEPA
 * include/microchip/lan80xx_mcu.h:58. Other status codes are not in
 * the open MEPA tree.
 */
#define BOOT_STS_DFU_MODE		0x12u

#define MAX_PADDING			15

/* 2 variants */
#define EXPECTED_DEVICE_ID		0x8024u
#define EXPECTED_DEVICE_ID_ALT		0x8023u

/*
 * Some silicon ships with DEVICE_ID OTP blown to 0x8005. The value is
 * NOT in MEPA's published chip table and we do NOT have a confirmed
 * meaning from Microchip. XXX TBC. Observed empirically on a
 * module whose SILICON_REV reads 0x8024 (LAN8024 family).
 */
#define DEVICE_ID_OTP_TBC_8005		0x8005u  /* XXX TBC with Microchip */

#define ARRAY_SIZE(a)			(sizeof(a) / sizeof((a)[0]))

struct chip_info {
	uint16_t	id;
	const char	*name;
	const char	*desc;
	bool		quad;
	bool		has_25g;
	bool		has_1588;
	bool		has_macsec;
};

/* Lifted verbatim from mepa/microchip/lan80xx/src/lan80xx_types.h:44-55. */
static const struct chip_info chip_table[] = {
	{ 0x8044, "LAN8044", "Quad 1G/10G/25G PHY with 1588 and MACSEC", true,  true,  true,  true  },
	{ 0x8043, "LAN8043", "Quad 1G/10G/25G PHY with 1588",            true,  true,  true,  false },
	{ 0x8042, "LAN8042", "Quad 1G/10G/25G PHY with MACSEC",          true,  true,  false, true  },
	{ 0x8024, "LAN8024", "Dual 1G/10G/25G PHY with 1588 and MACSEC", false, true,  true,  true  },
	{ 0x8023, "LAN8023", "Dual 1G/10G/25G PHY with 1588",            false, true,  true,  false },
	{ 0x8022, "LAN8022", "Dual 1G/10G/25G PHY with MACSEC",          false, true,  false, true  },
	{ 0x8268, "LAN8268", "Quad 1G/10G PHY with 1588 and MACSEC",     true,  false, true,  true  },
	{ 0x8267, "LAN8267", "Quad 1G/10G PHY with 1588",                true,  false, true,  false },
	{ 0x8264, "LAN8264", "Dual 1G/10G PHY with 1588 and MACSEC",     false, false, true,  true  },
	{ 0x8263, "LAN8263", "Dual 1G/10G PHY with 1588",                false, false, true,  false },
	{ 0x8262, "LAN8262", "Dual 1G/10G PHY with MACSEC",              false, false, false, true  },
};

static const struct chip_info *chip_lookup(uint16_t id)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(chip_table); i++) {
		if (chip_table[i].id == id)
			return &chip_table[i];
	}
	return NULL;
}

static const char *silicon_rev_name(uint8_t rev)
{
	/*
	 * MEPA src/lan80xx_types.h:61-62 only documents A0 (0xA0) and
	 * A1 (0xA1). Everything else is undocumented in the open tree.
	 */
	switch (rev) {
	case 0xA0:
		return "A0";
	case 0xA1:
		return "A1";
	default:
		return "not documented in MEPA";
	}
}

struct spi_ctx {
	int		fd;
	uint32_t	hz;
	unsigned int	padding;
	unsigned int	channel;
};

static int spi_open(struct spi_ctx *s, const char *dev, uint32_t hz,
		    unsigned int padding, unsigned int channel)
{
	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;

	s->fd = open(dev, O_RDWR);
	if (s->fd < 0) {
		warn("open %s", dev);
		return -1;
	}
	if (ioctl(s->fd, SPI_IOC_WR_MODE, &mode) < 0 ||
	    ioctl(s->fd, SPI_IOC_RD_MODE, &mode) < 0 ||
	    ioctl(s->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
	    ioctl(s->fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) {
		warn("spidev ioctl");
		close(s->fd);
		return -1;
	}
	s->hz = hz;
	s->padding = padding;
	s->channel = channel;
	return 0;
}

static int spi_reg_read(const struct spi_ctx *s, unsigned int mmd,
			uint16_t addr, uint32_t *out)
{
	uint8_t tx[3 + MAX_PADDING + 4];
	uint8_t rx[sizeof(tx)];
	struct spi_ioc_transfer tr;
	uint32_t a;
	unsigned int d;

	memset(tx, 0xff, sizeof(tx));
	memset(rx, 0x00, sizeof(rx));

	/* RW=0 (read) → top bit of byte 0 stays clear. */
	a = ((s->channel & 0x3u)  << 21) |
	    ((mmd        & 0x1fu) << 16) |
	     (addr       & 0xffffu);
	tx[0] = (uint8_t)(a >> 16);
	tx[1] = (uint8_t)(a >> 8);
	tx[2] = (uint8_t)(a);

	memset(&tr, 0, sizeof(tr));
	tr.tx_buf = (unsigned long)tx;
	tr.rx_buf = (unsigned long)rx;
	tr.len = 3 + s->padding + 4;
	tr.speed_hz = s->hz;
	tr.bits_per_word = 8;

	if (ioctl(s->fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
		warn("SPI_IOC_MESSAGE");
		return -1;
	}

	d = 3 + s->padding;
	*out = ((uint32_t)rx[d]     << 24) |
	       ((uint32_t)rx[d + 1] << 16) |
	       ((uint32_t)rx[d + 2] <<  8) |
	       ((uint32_t)rx[d + 3]);
	return 0;
}

static void print_header(const char *label, unsigned int mmd, uint16_t off,
			 uint32_t raw)
{
	printf("  %-22s MMD 0x%02x off 0x%03x  raw=0x%08x\n",
	       label, mmd, off, raw);
}

static void decode_device_id(uint32_t raw, const struct chip_info **chip)
{
	uint16_t id = (uint16_t)(raw & 0xFFFFu);

	*chip = chip_lookup(id);
	if (id == 0x0000) {
		printf("    -> 0x0000 — possible A0 silicon (reads 0; check SILICON_REV).\n");
	} else if (id == 0xFFFF) {
		printf("    -> 0xFFFF — bus floats high. No chip / wrong CS / chip in reset.\n");
	} else if (id == DEVICE_ID_OTP_TBC_8005) {
		printf("    -> 0x%04x  XXX TBC with Microchip (not in MEPA's table); family from SILICON_REV\n",
		       id);
	} else if (*chip) {
		printf("    -> 0x%04x  %s  \"%s\"\n",
		       id, (*chip)->name, (*chip)->desc);
	} else {
		printf("    -> 0x%04x  unknown LAN80XX device id\n", id);
	}
}

static void decode_silicon_rev(uint32_t raw)
{
	uint8_t rev = (uint8_t)(raw & 0xFFu);

	printf("    -> 0x%02x  silicon rev %s\n", rev, silicon_rev_name(rev));
}

static void decode_otp_rev(uint32_t raw)
{
	printf("    -> OTP revision 0x%02x\n", (unsigned int)(raw & 0xFFu));
}

static void decode_feature_disable(uint32_t raw, const struct chip_info *chip)
{
	bool dis_macsec    = !!(raw & FEAT_MACSEC_DIS);
	bool dis_1588      = !!(raw & FEAT_1588_DIS);
	bool dis_25g       = !!(raw & FEAT_25G_DIS);
	bool dis_quad      = !!(raw & FEAT_QUAD_DIS);
	bool dis_cleartags = !!(raw & FEAT_CLEARTAGS_DIS);
	bool dis_mpls      = !!(raw & FEAT_MPLS_DIS);

	/*
	 * Bit semantics: 1 = "feature disabled by fuse". For QUAD: 1 = the
	 * chip is Dual (only 2 ports usable), 0 = Quad.
	 */
	printf("    -> port count : %s   (FEAT_QUAD_DIS=%u)\n",
	       dis_quad ? "Dual (2)" : "Quad (4)", dis_quad);
	printf("       1588 PTP   : %s   (FEAT_1588_DIS=%u)\n",
	       dis_1588 ? "disabled" : "enabled", dis_1588);
	printf("       25G speed  : %s   (FEAT_25G_DIS=%u)\n",
	       dis_25g ? "disabled" : "enabled", dis_25g);
	printf("       MACsec     : %s   (FEAT_MACSEC_DIS=%u)\n",
	       dis_macsec ? "disabled" : "enabled", dis_macsec);
	printf("       MPLS       : %s   (FEAT_MPLS_DIS=%u)\n",
	       dis_mpls ? "disabled" : "enabled", dis_mpls);
	printf("       CLEARTAGS  : %s   (FEAT_CLEARTAGS_DIS=%u)\n",
	       dis_cleartags ? "disabled" : "enabled", dis_cleartags);

	if (chip) {
		bool ok = ((!dis_quad)   == chip->quad) &&
			  ((!dis_25g)    == chip->has_25g) &&
			  ((!dis_1588)   == chip->has_1588) &&
			  ((!dis_macsec) == chip->has_macsec);

		printf("    -> consistency vs DEVICE_ID 0x%04x: %s\n",
		       chip->id,
		       ok ? "OK (matches the SKU description)"
			  : "MISMATCH (fuses contradict DEVICE_ID — odd)");
	}
}

static void decode_strap(uint32_t raw)
{
	uint8_t s       = (uint8_t)(raw & 0xFFu);
	bool dfu        = !!(s & STRAP_DFU);
	bool serdes     = !!(s & STRAP_SERDES_INIT);
	bool bist_bp    = !!(s & STRAP_BIST_BYPASS);
	uint8_t unnamed = s & (uint8_t)~STRAP_NAMED_MASK;

	printf("    -> raw 0x%02x  DFU=%u  SERDES_INIT=%u  BIST_BYPASS=%u%s\n",
	       s, dfu, serdes, bist_bp,
	       !serdes ? "  (SERDES_INIT low → bootrom won't progress on A0/A1!)"
		       : "");
	if (unnamed)
		printf("       other bits set: 0x%02x  (no name in MEPA lan80xx_mcu.h)\n",
		       unnamed);
}

static void decode_mcu_boot_status(uint32_t raw)
{
	uint16_t v = (uint16_t)(raw & 0xFFFFu);
	const char *meaning = "running (or pre-DFU) — only 0x12=DFU is documented";

	if (v == BOOT_STS_DFU_MODE)
		meaning = "DFU mode (firmware update active)";
	else if (v == 0x0000)
		meaning = "0 — MCU not yet booted / chip held in reset";
	else if (v == 0xFFFF)
		meaning = "0xFFFF — bus floats / no response";
	printf("    -> 0x%04x  %s\n", v, meaning);
}

static void decode_post1_status(uint32_t raw, const struct chip_info *chip)
{
	bool done   = !!(raw & POST1_DONE);
	bool s0     = !!(raw & POST1_SLICE0_PASS);
	bool s1     = !!(raw & POST1_SLICE1_PASS);
	bool s2     = !!(raw & POST1_SLICE2_PASS);
	bool s3     = !!(raw & POST1_SLICE3_PASS);
	bool bypass = !!(raw & POST1_BIST_BYPASS);

	printf("    -> POST1_DONE=%u  BIST_BYPASS=%u\n", done, bypass);
	printf("       slice0=%s  slice1=%s  slice2=%s  slice3=%s\n",
	       s0 ? "pass" : "fail/init",
	       s1 ? "pass" : "fail/init",
	       s2 ? "pass" : "fail/init",
	       s3 ? "pass" : "fail/init");
	if (chip && !chip->quad)
		printf("       (slice2/slice3 not populated on this Dual-port SKU)\n");
}

static void usage(const char *argv0)
{
	fprintf(stderr,
		"Usage: %s <spidev> [-c channel] [-s speed_hz] [-p padding]\n"
		"  spidev    e.g. /dev/spidev0.0\n"
		"  -c 0..3   channel within the chip (default 0)\n"
		"  -s hz     SPI clock in Hz (default 1000000)\n"
		"  -p 0..%d   dummy bytes between addr and data (default 0)\n",
		argv0, MAX_PADDING);
}

int main(int argc, char **argv)
{
	const struct chip_info *chip = NULL;
	struct spi_ctx s;
	const char *dev;
	uint32_t hz = 1000000;
	uint32_t v;
	uint16_t did;
	unsigned int channel = 0;
	unsigned int padding = 0;
	uint8_t rev;
	bool a0_alive;
	bool otp_unprog = false;
	bool otp_id_tbc = false;	/* DEVICE_ID is observed but TBC */
	int opt;
	int rc = EXIT_SUCCESS;

	if (argc < 2) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}
	dev = argv[1];

	optind = 2;
	while ((opt = getopt(argc, argv, "c:s:p:h")) != -1) {
		switch (opt) {
		case 'c':
			channel = (unsigned int)strtoul(optarg, NULL, 0);
			break;
		case 's':
			hz = (uint32_t)strtoul(optarg, NULL, 0);
			break;
		case 'p':
			padding = (unsigned int)strtoul(optarg, NULL, 0);
			break;
		default:
			usage(argv[0]);
			return EXIT_FAILURE;
		}
	}
	if (channel > 3 || padding > MAX_PADDING) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}

	if (spi_open(&s, dev, hz, padding, channel) < 0)
		return EXIT_FAILURE;

	printf("Probing %s  ch=%u  @ %u Hz  padding=%u\n\n",
	       dev, channel, hz, padding);

	/* DEVICE_ID first — if the bus is dead we bail before reading more. */
	printf("DEVICE_ID:\n");
	if (spi_reg_read(&s, MMD_DEVICE_INFO, REG_DEVICE_ID, &v) < 0) {
		rc = EXIT_FAILURE;
		goto out;
	}
	print_header("DEVICE_ID_REG", MMD_DEVICE_INFO, REG_DEVICE_ID, v);
	decode_device_id(v, &chip);
	did = (uint16_t)(v & 0xFFFFu);

	/* SILICON_REV — read even on stuck-zero DEVICE_ID, that's the A0 path. */
	printf("\nSilicon revision:\n");
	if (spi_reg_read(&s, MMD_DEVICE_INFO, REG_DEVICE_SILICON_REV, &v) < 0) {
		rc = EXIT_FAILURE;
		goto out;
	}
	print_header("DEVICE_SILICON_REV", MMD_DEVICE_INFO,
		     REG_DEVICE_SILICON_REV, v);
	decode_silicon_rev(v);
	rev = (uint8_t)(v & 0xFFu);
	a0_alive = (did == 0 && rev == 0xA0);
	/*
	 * DEVICE_ID does not authoritatively identify the family in two
	 * observed cases. Both fall through to SILICON_REV-based
	 * identification rather than asserting on DEVICE_ID alone:
	 *
	 *  - 0x0000: OTP region unprogrammed on engineering samples;
	 *            documented in MEPA's A0 path.
	 *  - 0x8005: XXX TBC with Microchip -- observed empirically but
	 *            no published interpretation. We do NOT infer a
	 *            meaning here; the operator should ask Microchip.
	 *
	 * The SILICON_REV register's low 16 bits still carry the
	 * device-family signature in both cases (e.g. raw=0x00008024 on
	 * LAN8024 silicon), so we use that. Add new
	 * unknown-but-observed DEVICE_IDs to this list as bring-up
	 * encounters them, alongside an XXX TBC note for each.
	 */
	if ((did == 0 || did == DEVICE_ID_OTP_TBC_8005) && !a0_alive) {
		uint16_t did_fb = (uint16_t)(v & 0xFFFFu);
		const struct chip_info *chip_fb = chip_lookup(did_fb);

		if (chip_fb) {
			if (did == 0) {
				otp_unprog = true;
			} else {
				otp_id_tbc = true;
			}
			did = did_fb;
			chip = chip_fb;
		}
	}

	if (did != 0xFFFF && (did != 0 || a0_alive)) {
		printf("\nOTP revision:\n");
		if (spi_reg_read(&s, MMD_DEVICE_INFO,
				 REG_DEVICE_OTP_REV, &v) == 0) {
			print_header("DEVICE_OTP_REV", MMD_DEVICE_INFO,
				     REG_DEVICE_OTP_REV, v);
			decode_otp_rev(v);
		}

		printf("\nFeature fuses:\n");
		if (spi_reg_read(&s, MMD_DEVICE_INFO,
				 REG_DEVICE_FEATURE_DIS, &v) == 0) {
			print_header("DEVICE_FEATURE_DIS", MMD_DEVICE_INFO,
				     REG_DEVICE_FEATURE_DIS, v);
			decode_feature_disable(v, chip);
		}

		printf("\nStraps:\n");
		if (spi_reg_read(&s, MMD_DEVICE_INFO,
				 REG_STRAP_READ, &v) == 0) {
			print_header("STRAP_READ_REG", MMD_DEVICE_INFO,
				     REG_STRAP_READ, v);
			decode_strap(v);
		}

		printf("\nMCU boot:\n");
		if (spi_reg_read(&s, MMD_DEVICE_INFO,
				 REG_MCU_BOOT_STATUS, &v) == 0) {
			print_header("MCU_BOOT_STATUS_REG", MMD_DEVICE_INFO,
				     REG_MCU_BOOT_STATUS, v);
			decode_mcu_boot_status(v);
		}

		printf("\nPOST1 BIST:\n");
		if (spi_reg_read(&s, MMD_DEVICE_INFO,
				 REG_POST1_POST_STATUS, &v) == 0) {
			print_header("POST1_POST_STATUS", MMD_DEVICE_INFO,
				     REG_POST1_POST_STATUS, v);
			decode_post1_status(v, chip);
		}
	}

	/* Final verdict. */
	printf("\nResult: ");
	if (did == EXPECTED_DEVICE_ID || did == EXPECTED_DEVICE_ID_ALT) {
		const char *note = "";

		if (otp_unprog)
			note = " (DEVICE_ID OTP unprogrammed; family from SILICON_REV)";
		else if (otp_id_tbc)
			note = " (DEVICE_ID=0x8005, XXX TBC with Microchip; family from SILICON_REV)";

		printf("OK — %s detected on ch %u%s.\n",
		       chip ? chip->name : "LAN80XX", channel, note);
		rc = EXIT_SUCCESS;
	} else if (a0_alive) {
		printf("OK — LAN80XX A0 silicon on ch %u (DEVICE_ID reads 0;\n"
		       "        MEPA's common probe path treats this as LAN8044).\n",
		       channel);
		rc = EXIT_SUCCESS;
	} else if (chip) {
		printf("Mixed — bus is alive and a LAN80XX answers (%s, \"%s\"),\n"
		       "        but it is not the expected SKU (0x%04x or 0x%04x).\n"
		       "        Wrong chip, wrong channel, or wrong SPI CS routing.\n",
		       chip->name, chip->desc,
		       EXPECTED_DEVICE_ID, EXPECTED_DEVICE_ID_ALT);
		rc = EXIT_FAILURE;
	} else if (did == 0xFFFF) {
		printf("FAIL — bus floats high (0xFFFF). Likely no slave responding:\n"
		       "        check power, RESET#, and that CS is wired to this slave.\n");
		rc = EXIT_FAILURE;
	} else if (did == 0x0000) {
		printf("FAIL — DEVICE_ID and SILICON_REV both read 0. Chip is held\n"
		       "        in reset, unpowered, or SCLK/MOSI is stuck.\n");
		rc = EXIT_FAILURE;
	} else {
		printf("FAIL — DEVICE_ID 0x%04x is not in our chip table and is\n"
		       "        TBC with MEPA lan80xx_types.h. The bus is\n"
		       "        responding (other reads above succeeded), so this\n"
		       "        looks like an undocumented/new LAN80XX SKU rather\n"
		       "        than a wiring or SCLK issue. Awaiting authoritative\n"
		       "        identification from Microchip before classifying.\n",
		       did);
		rc = EXIT_FAILURE;
	}

out:
	close(s.fd);
	return rc;
}
