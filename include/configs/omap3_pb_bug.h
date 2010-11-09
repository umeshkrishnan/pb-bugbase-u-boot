/*
 * (C) Copyright 2010
 * Bug Labs, http://buglabs.net
 *
 * Based on omap3_evm.h:
 * (C) Copyright 2006 Texas Instruments.
 * Richard Woodruff <r-woodruff2@ti.com>
 * Syed Mohammed Khasim <x0khasim@ti.com>
 *
 * Configuration settings for the 3530 TI EVM3530 board.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H
#include <asm/sizes.h>

/*  ============================================================================
 *  High Level Configuration Options
 *  ============================================================================
 */
#define CONFIG_ARMV7		1	/* This is an ARM V7 CPU core	*/
#define CONFIG_OMAP		1	/* in a TI OMAP core		*/
#define CONFIG_OMAP34XX		1	/* which is a 34XX */
#define CONFIG_OMAP3430		1	/* which is in a 3430 */
#define CONFIG_OMAP3PB		1	/* working with PB BUG BASE 1.0 */

#define CONFIG_SDRC	/* The chip has SDRC controller */
#define ES_2_0			1

#include <asm/arch/cpu.h>	/* Get chip and board defs  */
#include <asm/arch/omap3.h>

/*
 * Display CPU and Board information
 */
#define CONFIG_DISPLAY_CPUINFO		1
#define CONFIG_DISPLAY_BOARDINFO	1

/* Clock Defines */
#define V_OSCK			26000000	/* Clock output from T2 */
#define V_SCLK			(V_OSCK >> 1)

#undef CONFIG_USE_IRQ			/* No support for IRQs */
#define CONFIG_MISC_INIT_R

#define CONFIG_CMDLINE_TAG		1	/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1
#define CONFIG_REVISION_TAG		1

/*  ----------------------------------------------------------------------------
 *  OFF Mode pad configuration
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_OFF_PADCONF	1

/*  ============================================================================
 *  Commands
 *  ============================================================================
 */

#define CONFIG_BOOTP_MASK	CONFIG_BOOTP_DEFAULT

/* this must be included AFTER the definition of CONFIG_COMMANDS (if any) */
#include <config_cmd_default.h>

#define CONFIG_CMD_EXT2		/* EXT2 Support			*/
#define CONFIG_CMD_FAT		/* FAT support			*/
#define CONFIG_CMD_JFFS2	/* JFFS2 Support		*/

#define CONFIG_CMD_I2C		/* I2C serial bus support	*/
#define CONFIG_CMD_MMC		/* MMC support			*/
#define CONFIG_CMD_NAND
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_PING

#undef CONFIG_CMD_FLASH		/* flinfo, erase, protect	*/
#undef CONFIG_CMD_FPGA		/* FPGA configuration Support	*/
#undef CONFIG_CMD_IMI		/* iminfo			*/
#undef CONFIG_CMD_IMLS		/* List all found images	*/

/*  ============================================================================
 *  Hardware drivers
 *  ============================================================================
 */

/*
 * TWL4030
 */
#define CONFIG_TWL4030_POWER		1

/*  ----------------------------------------------------------------------------
 *  NS16550
 *  ----------------------------------------------------------------------------
 */
#define V_NS16550_CLK		(48000000)      /* 48MHz (APLL96/2) */

#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_SYS_NS16550_CLK		V_NS16550_CLK

#define CONFIG_SYS_NS16550_COM1	OMAP34XX_UART1
#define CONFIG_SYS_NS16550_COM3	OMAP34XX_UART3

/*  ----------------------------------------------------------------------------
 *  Serial Console
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_SERIAL1		3	/* UART1 */
#define CONFIG_CONS_INDEX	3
#define CONFIG_BAUDRATE		115200
#define CONFIG_SYS_BAUDRATE_TABLE	{4800, 9600, 19200, 38400, 57600, 115200}

#define CONFIG_MMC              1
#define CONFIG_OMAP3_MMC	1
#define CONFIG_DOS_PARTITION    1
/*  ----------------------------------------------------------------------------
 *  SMSC9115 Ethernet from SMSC9118 family
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_NET_MULTI
#define CONFIG_SMC911X
#define CONFIG_SMC911X_32_BIT
#define CONFIG_SMC911X_BASE	0x2C000000

/*  ----------------------------------------------------------------------------
 *  I2C
 *  ----------------------------------------------------------------------------
 */
#ifdef CONFIG_CMD_I2C

#define CONFIG_DRIVER_OMAP34XX_I2C	1

#define CONFIG_HARD_I2C			1
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_SYS_I2C_SLAVE		1
#define CONFIG_SYS_I2C_BUS		0
#define CONFIG_SYS_I2C_BUS_SELECT	1

#endif  /* (CONFIG_COMMANDS & CONFIG_CMD_I2C) */

/*  ----------------------------------------------------------------------------
 *  OneNAND & NAND
 *  ----------------------------------------------------------------------------
 */
#define NAND_MAX_CHIPS		1
#define CONFIG_NAND_OMAP_GPMC
#define GPMC_NAND_ECC_LP_x16_LAYOUT
#define CONFIG_SYS_ONENAND_BASE	ONENAND_MAP
#define ONENAND_DEBUG

#define CONFIG_SYS_NAND_ADDR NAND_BASE  /* physical address to access nand*/
#define CONFIG_SYS_NAND_BASE NAND_BASE  /* physical address to access nand at CS0*/

#define CONFIG_SYS_MAX_NAND_DEVICE      1 /* Max number of NAND devices */
#define SECTORSIZE               512

#define NAND_ALLOW_ERASE_ALL
#define ADDR_COLUMN              1
#define ADDR_PAGE                2
#define ADDR_COLUMN_PAGE         3

#define NAND_ChipID_UNKNOWN      0x00
#define NAND_MAX_FLOORS          1
#define NAND_MAX_CHIPS           1
#define NAND_NO_RB               1
#define CONFIG_SYS_NAND_WP

/*  ----------------------------------------------------------------------------
 *  PISMO
 *  ----------------------------------------------------------------------------
 */
#define PISMO1_NAND_SIZE		GPMC_SIZE_128M
#define PISMO1_ONEN_SIZE		GPMC_SIZE_128M
#define DBG_MPDB_SIZE		GPMC_SIZE_16M

#define CONFIG_SYS_MAX_FLASH_BANKS	2
#define CONFIG_SYS_MAX_FLASH_SECT	(520)

#define CONFIG_SYS_FLASH_BASE		boot_flash_base
#define PHYS_FLASH_SECT_SIZE	boot_flash_sec

#define ONENAND_ENV_OFFSET	0x240000   /* environment starts here  */
#define SMNAND_ENV_OFFSET        0x240000    /* environment starts here  */

/*  ----------------------------------------------------------------------------
 *  Flash
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_SYS_NO_FLASH		1	/* Disable Flash support */

/*  ============================================================================
 *  Miscellaneous configurable options
 *  ============================================================================
 */
/*  ----------------------------------------------------------------------------
 *  SDRAM Bank Allocation method
 *  ----------------------------------------------------------------------------
 */
#define SDRC_R_B_C		1

/*  ----------------------------------------------------------------------------
 *  Physical Memory Map
 *  (CS1 may or may not be populated)
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_NR_DRAM_BANKS	2
#define PHYS_SDRAM_1		OMAP34XX_SDRC_CS0
#define PHYS_SDRAM_1_SIZE	SZ_128M			/* At least 128 megs */
#define PHYS_SDRAM_2		OMAP34XX_SDRC_CS1
#define CONFIG_OMAP3_MICRON_DDR		1

/*  ----------------------------------------------------------------------------
 *  Range for memory test
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_SYS_MEMTEST_START	(OMAP34XX_SDRC_CS0)
#define CONFIG_SYS_MEMTEST_END		(OMAP34XX_SDRC_CS0+0x01F00000)

#undef  CFG_CLKS_IN_HZ		/* everything, incl board info, in Hz */


/* 2430 has 12 GP timers, they can be driven by the SysClk (12/13/19.2) or by
 * 32KHz clk, or from external sig. This rate is divided by a local divisor.
 */

#define CONFIG_SYS_TIMERBASE		OMAP34XX_GPT2
#define CONFIG_SYS_PTV			2  /* 2^(pvt+1) */
#define CONFIG_SYS_HZ			1000

/*  ----------------------------------------------------------------------------
 *  JFFS2
 *  (OMAP3 EVM supports JFFS2 by default)
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_SYS_JFFS2_MEM_NAND
#define JFFS2_BOOT_DEFAULTS	TRUE


/*  ----------------------------------------------------------------------------
 *  Monitor
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_SYS_MONITOR_BASE	ONENAND_MAP	/* At start of flash */

#define CONFIG_SYS_MONITOR_LEN		SZ_256K		/* Reserve 2 sectors */


/*  ============================================================================
 *  Environment
 *  ============================================================================
 */
#define ENV_IS_VARIABLE		1
#define CONFIG_ENV_IS_IN_NAND	1
#define CONFIG_SYS_ENV_SECT_SIZE	boot_flash_sec
#define CONFIG_ENV_OFFSET		boot_flash_off
#define CONFIG_ENV_ADDR		boot_flash_env_addr

/*  ----------------------------------------------------------------------------
 *  Allow environment overwrite
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_ENV_OVERWRITE

/*  ----------------------------------------------------------------------------
 *  Size of environment
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_ENV_SIZE		SZ_128K

/*  ----------------------------------------------------------------------------
 *  Size of malloc() pool
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + SZ_128K)

/*  ----------------------------------------------------------------------------
 *  Size of global data arra
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_SYS_GBL_DATA_SIZE	128

/*  ----------------------------------------------------------------------------
 *  Prompt
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_SYS_HUSH_PARSER		/* use "hush" command parser */
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_PROMPT		"BUGBASE2 # "
#define CONFIG_CMDLINE_EDITING

/*  ----------------------------------------------------------------------------
 *  Help
 *  (Undef to save memory)
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_SYS_LONGHELP

/*  ----------------------------------------------------------------------------
 *  Default load address
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_SYS_LOAD_ADDR	(OMAP34XX_SDRC_CS0)

/*  ----------------------------------------------------------------------------
 *  Buffers
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_SYS_CBSIZE	512				    /* Console I/O   */
#define CONFIG_SYS_PBSIZE	(CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16)  /* Print	     */
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE			    /* Boot Argument */

/*  ----------------------------------------------------------------------------
 *  Stack sizes
 *  Used to set stack sizes in start.S
 *  ----------------------------------------------------------------------------
 */
#define CONFIG_STACKSIZE	  SZ_128K	/* Regular stack */

#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ      SZ_4K		/* IRQ stack     */
#define CONFIG_STACKSIZE_FIQ      SZ_4K		/* FIQ stack     */
#endif

/*  ----------------------------------------------------------------------------
 *  Boot options
 *  ----------------------------------------------------------------------------
 */

#define CONFIG_PREBOOT                  /* enable preboot variable */
#define CONFIG_BOOTDELAY	3
#define CONFIG_BOOTCOMMAND	"run mmcboot"
#define CONFIG_AUTO_COMPLETE	1	/* TBD */
#define CONFIG_SYS_MAXARGS	16	/* max number of command args */ /* TBD */

#define CONFIG_BOOTARGS	"console=ttyS2,115200n8 root=/dev/mmcblk0p1 rw rootwait"

#define CONFIG_EXTRA_ENV_SETTINGS \
  "console=ttyS2,115200n8\0" \
  "mmcboot=mmc init;ext2load mmc 0:1 0x80000000 /boot/uImage;run mmcargs;bootm 0x80000000\0"\
  "mmcargs=setenv bootargs console=${console} root=/dev/mmcblk0p1 rw rootwait rootdelay=1 kgdboc=${console} usbcore.autosuspend=-1 vram=24M omapfb.vram=0:0M,1:24M,2:0M omapdss.def_disp=dvi omapfb.mode=dvi:1280x1024MR-32@57\0" \



#ifndef __ASSEMBLY__
extern struct gpmc *gpmc_cfg;
extern volatile unsigned int boot_flash_env_addr;
extern unsigned int nand_cs_base;
extern unsigned int boot_flash_base;
extern unsigned int boot_flash_off;
extern unsigned int boot_flash_sec;
extern unsigned int boot_flash_type;
#endif  /* __ASSEMBLY__ */

#endif  /* __CONFIG_BUGBASE2_H */
