/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_atomic.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_ecall.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_ipi.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_system.h>
#include <sbi/sbi_timer.h>
#include <sbi/sbi_version.h>

#define BANNER                                              \
	"   ____                    _____ ____ _____\n"     \
	"  / __ \\                  / ____|  _ \\_   _|\n"  \
	" | |  | |_ __   ___ _ __ | (___ | |_) || |\n"      \
	" | |  | | '_ \\ / _ \\ '_ \\ \\___ \\|  _ < | |\n" \
	" | |__| | |_) |  __/ | | |____) | |_) || |_\n"     \
	"  \\____/| .__/ \\___|_| |_|_____/|____/_____|\n"  \
	"        | |\n"                                     \
	"        |_|\n\n"

static void sbi_boot_prints(struct sbi_scratch *scratch, u32 hartid)
{
	char str[64];
	const struct sbi_platform *plat = sbi_platform_ptr(scratch);

	misa_string(str, sizeof(str));
#ifdef OPENSBI_VERSION_GIT
	sbi_printf("\nOpenSBI %s (%s %s)\n", OPENSBI_VERSION_GIT,
		   __DATE__, __TIME__);
#else
	sbi_printf("\nOpenSBI v%d.%d (%s %s)\n", OPENSBI_VERSION_MAJOR,
		   OPENSBI_VERSION_MINOR, __DATE__, __TIME__);
#endif

	sbi_printf(BANNER);

	/* Platform details */
	sbi_printf("Platform Name          : %s\n", sbi_platform_name(plat));
	sbi_printf("Platform HART Features : RV%d%s\n", misa_xlen(), str);
	sbi_printf("Platform Max HARTs     : %d\n",
		   sbi_platform_hart_count(plat));
	sbi_printf("Current Hart           : %u\n", hartid);
	/* Firmware details */
	sbi_printf("Firmware Base          : 0x%lx\n", scratch->fw_start);
	sbi_printf("Firmware Size          : %d KB\n",
		   (u32)(scratch->fw_size / 1024));
	/* Generic details */
	sbi_printf("Runtime SBI Version    : %d.%d\n",
		   sbi_ecall_version_major(), sbi_ecall_version_minor());
	sbi_printf("\n");

	sbi_hart_pmp_dump(scratch);
}

static void __noreturn init_coldboot(struct sbi_scratch *scratch, u32 hartid)
{
	int rc;
	const struct sbi_platform *plat = sbi_platform_ptr(scratch);

	rc = sbi_system_early_init(scratch, TRUE);
	if (rc)
		sbi_hart_hang();

	rc = sbi_hart_init(scratch, hartid, TRUE);
	if (rc)
		sbi_hart_hang();

	rc = sbi_console_init(scratch);
	if (rc)
		sbi_hart_hang();

	rc = sbi_platform_irqchip_init(plat, TRUE);
	if (rc)
		sbi_hart_hang();

	rc = sbi_ipi_init(scratch, TRUE);
	if (rc)
		sbi_hart_hang();

	rc = sbi_timer_init(scratch, TRUE);
	if (rc)
		sbi_hart_hang();

	rc = sbi_system_final_init(scratch, TRUE);
	if (rc)
		sbi_hart_hang();

	if (!(scratch->options & SBI_SCRATCH_NO_BOOT_PRINTS))
		sbi_boot_prints(scratch, hartid);

	if (!sbi_platform_has_hart_hotplug(plat))
		sbi_hart_wake_coldboot_harts(scratch, hartid);
	sbi_hart_mark_available(hartid);
	sbi_hart_switch_mode(hartid, scratch->next_arg1, scratch->next_addr,
			     scratch->next_mode, FALSE);
}

static void __noreturn init_warmboot(struct sbi_scratch *scratch, u32 hartid)
{
	int rc;
	const struct sbi_platform *plat = sbi_platform_ptr(scratch);

	if (!sbi_platform_has_hart_hotplug(plat))
		sbi_hart_wait_for_coldboot(scratch, hartid);

	if (sbi_platform_hart_disabled(plat, hartid))
		sbi_hart_hang();

	rc = sbi_system_early_init(scratch, FALSE);
	if (rc)
		sbi_hart_hang();

	rc = sbi_hart_init(scratch, hartid, FALSE);
	if (rc)
		sbi_hart_hang();

	rc = sbi_platform_irqchip_init(plat, FALSE);
	if (rc)
		sbi_hart_hang();

	rc = sbi_ipi_init(scratch, FALSE);
	if (rc)
		sbi_hart_hang();

	rc = sbi_timer_init(scratch, FALSE);
	if (rc)
		sbi_hart_hang();

	rc = sbi_system_final_init(scratch, FALSE);
	if (rc)
		sbi_hart_hang();

	sbi_hart_mark_available(hartid);

	if (sbi_platform_has_hart_hotplug(plat))
		/* TODO: To be implemented in-future. */
		sbi_hart_hang();
	else
		sbi_hart_switch_mode(hartid, scratch->next_arg1,
				     scratch->next_addr,
				     scratch->next_mode, FALSE);
}

/**
 * Hawksbill has a hardware issue that it doesn't hold hart 1 to 12 when
 * system reset and power on.
 *
 * We make hart 1 to 12 wait for interrupt at fw_base.S, it prevents these
 * harts wander around when reset or power on.
 * After hart 0 loading image, hart 0 would go to _start, and be responsible
 * for waking up other harts. Here is a workaround that initialize platform ipi
 * earlier for sending ipi.
 *
 * We cannot make hart 1 to 12 be stalled by original way. (i.e at
 * sbi_hart_wait_for_coldboot that called by init_warmboot). In hawksbill case
 * hart 1 to 12 would go warm_boot, and no one can wake up them. Even if hart 0
 * wake them at the moment, all registers status had been corrupted by hardware
 * initialization. So we stall hart 1 to 12 as earlier as possible.
 *
 * Notice: Power on case is still unresolved. At that moment, memory is empty,
 * hart 1 to 12 are going to anywhere unexpectedly
 */
static void __noreturn hawksbill_hart0_boot(struct sbi_scratch *scratch)
{
	const struct sbi_platform *plat = sbi_platform_ptr(scratch);
	int max_hart = sbi_platform_hart_count(plat);
	int rc;

	rc = sbi_platform_ipi_init(plat, TRUE);
	if (rc)
		sbi_hart_hang();

	for (int i = 1; i < max_hart; i++) {
		if (!sbi_platform_hart_disabled(plat, i)) {
			sbi_platform_ipi_send(plat, i);
		}
	}

	sbi_hart_hang();
}

static atomic_t coldboot_lottery = ATOMIC_INITIALIZER(0);

/**
 * Initialize OpenSBI library for current HART and jump to next
 * booting stage.
 *
 * The function expects following:
 * 1. The 'mscratch' CSR is pointing to sbi_scratch of current HART
 * 2. Stack pointer (SP) is setup for current HART
 * 3. Interrupts are disabled in MSTATUS CSR
 * 4. All interrupts are disabled in MIE CSR
 *
 * @param scratch pointer to sbi_scratch of current HART
 */
void __noreturn sbi_init(struct sbi_scratch *scratch)
{
	bool coldboot			= FALSE;
	u32 hartid			= sbi_current_hartid();
	const struct sbi_platform *plat = sbi_platform_ptr(scratch);

	if (hartid == 0) {
		hawksbill_hart0_boot(scratch);
	}

	if (sbi_platform_hart_disabled(plat, hartid))
		sbi_hart_hang();

	if (atomic_add_return(&coldboot_lottery, 1) == 1)
		coldboot = TRUE;

	/* On hawksbill issue fixing (see line 154), hart 1 to 12 would be waked
	 * and enter here. They need to clear MSIP, otherwise, the wfi in
	 * warm_boot (at sbi_hart_wait_for_coldboot) has no effect.
	 **/
	sbi_platform_ipi_clear(plat, hartid);

	if (coldboot)
		init_coldboot(scratch, hartid);
	else
		init_warmboot(scratch, hartid);
}
