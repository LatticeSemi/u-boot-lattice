// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2000-2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 */

/*
 * Misc boot support
 */
#include <common.h>
#include <command.h>
#include <net.h>

#ifdef CONFIG_CMD_GO

/* Allow ports to override the default behavior */
__attribute__((weak))
unsigned long do_go_exec(ulong (*entry)(int, char * const []), int argc,
				 char *const argv[])
{
	return entry (argc, argv);
}

static int do_go(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	ulong	rc, length;
	int     rcode = 0;
	u16     calculated_crc, read_crc;
	u8      *addr = NULL;

	if (argc < 3)
		return CMD_RET_USAGE;

	addr = hextoul(argv[1], NULL);
	length = hextoul(argv[2], NULL);

#if IS_ENABLED(CONFIG_CRC_CHECKING)
	printf ("## Checking CRC\n");
	read_crc = (addr[length - 2] << 8)  | addr[length - 1];
	calculated_crc = crc16_ccitt(0x1D0F, addr, length - 2);
	if (read_crc != calculated_crc) {
		printf("Bad firmware CRC: file 0x%04x calculated 0x%04x\n",
			read_crc, calculated_crc);
	} else {
		printf("CRC pass: 0x%04x\n", calculated_crc);
	}
#endif

	printf ("## Starting application at 0x%08lX ...\n", addr);
	flush();

	/*
	 * pass address parameter as argv[0] (aka command name),
	 * and all remaining args
	 */
	rc = do_go_exec ((void *)addr, argc - 1, argv + 1);
	if (rc != 0) rcode = 1;

	printf ("## Application terminated, rc = 0x%lX\n", rc);
	return rcode;
}

/* -------------------------------------------------------------------- */

U_BOOT_CMD(
	go, CONFIG_SYS_MAXARGS, 1,	do_go,
	"start application at address 'addr' with 'length'",
	"addr length [arg ...]\n    - start application at address 'addr' with\n"
	"      'length' as number of bytes to read\n"
	"      passing 'arg' as arguments"
);

#endif

U_BOOT_CMD(
	reset, 2, 0,	do_reset,
	"Perform RESET of the CPU",
	"- cold boot without level specifier\n"
	"reset -w - warm reset if implemented"
);

#ifdef CONFIG_CMD_POWEROFF
U_BOOT_CMD(
	poweroff, 1, 0,	do_poweroff,
	"Perform POWEROFF of the device",
	""
);
#endif
