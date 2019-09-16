/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Atish Patra <atish.patra@wdc.com>
 */

#include <libfdt.h>
#include <fdt.h>
#include <sbi/riscv_encoding.h>
#include <sbi/sbi_const.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_platform.h>
#include <sbi/riscv_io.h>
#include <sbi_utils/serial/sifive-uart.h>
#include "plic.h"
#include "clint.h"

/* clang-format off */

#define VCU118_TLCLK				50000000

#define VCU118_CLINT_ADDR			0x2000000

#define VCU118_PLIC_ADDR			0xc000000
#define VCU118_PLIC_NUM_SOURCES			0x15
#define VCU118_PLIC_NUM_PRIORITIES		7

#define VCU118_UART0_ADDR			0x10010000
#define VCU118_UART_BAUDRATE			115200

/**
 * Hawksbill project consist ofs two VCU118 boards.
 * Each VCU118 has 5 HARTs but HART ID 0 doesn't have S mode.
 */
#define VCU118_ENABLED_HART_MASK		\
	(1 << 1 | 1 << 2 | 1 << 3 | 1 << 4 | 	\
	 1 << 9 | 1 << 10 | 1 << 11 | 1 << 12)
#define VCU118_HARITD_DISABLED			~(VCU118_ENABLED_HART_MASK)
#define VCU118_HART_COUNT			16
#define VCU118_HART_STACK_SIZE			8192

/* clang-format on */

static void vcu118_modify_dt(void *fdt)
{
	u32 i, size;
	int chosen_offset, err;
	int cpu_offset;
	char cpu_node[32] = "";
	const char *mmu_type;

	size = fdt_totalsize(fdt);
	err  = fdt_open_into(fdt, fdt, size + 256);
	if (err < 0)
		sbi_printf(
			"Device Tree can't be expanded to accmodate new node");

	for (i = 0; i < VCU118_HART_COUNT; i++) {
		sbi_sprintf(cpu_node, "/cpus/cpu@%d", i);
		cpu_offset = fdt_path_offset(fdt, cpu_node);
		mmu_type   = fdt_getprop(fdt, cpu_offset, "mmu-type", NULL);
		if (mmu_type && (!strcmp(mmu_type, "riscv,sv39") ||
				 !strcmp(mmu_type, "riscv,sv48")))
			continue;
		else
			fdt_setprop_string(fdt, cpu_offset, "status",
					   "disabled");
		memset(cpu_node, 0, sizeof(cpu_node));
	}

	chosen_offset = fdt_path_offset(fdt, "/chosen");
	fdt_setprop_string(fdt, chosen_offset, "serial0",
			   "/soc/serial@10010000:115200");

	vcu118_plic_fdt_fixup(fdt, "riscv,plic0");
}

static int vcu118_final_init(bool cold_boot)
{
	void *fdt;

	if (!cold_boot)
		return 0;

	fdt = sbi_scratch_thishart_arg1_ptr();
	vcu118_modify_dt(fdt);

	return 0;
}

static u32 vcu118_pmp_region_count(u32 hartid)
{
	return 1;
}

static int vcu118_pmp_region_info(u32 hartid, u32 index, ulong *prot,
				  ulong *addr, ulong *log2size)
{
	int ret = 0;

	switch (index) {
	case 0:
		*prot     = PMP_R | PMP_W | PMP_X;
		*addr     = 0;
		*log2size = __riscv_xlen;
		break;
	default:
		ret = -1;
		break;
	};

	return ret;
}

static int vcu118_console_init(void)
{
	return sifive_uart_init(VCU118_UART0_ADDR, VCU118_TLCLK,
				VCU118_UART_BAUDRATE);
}

static int vcu118_irqchip_init(bool cold_boot)
{
	int rc;
	u32 hartid = sbi_current_hartid();

	if (cold_boot) {
		rc = vcu118_plic_cold_irqchip_init(VCU118_PLIC_ADDR,
						   VCU118_PLIC_NUM_SOURCES,
						   VCU118_HART_COUNT);
		if (rc)
			return rc;
	}

	return vcu118_plic_warm_irqchip_init(hartid,
					     (hartid) ? (2 * hartid - 1) : 0,
					     (hartid) ? (2 * hartid) : -1);
}

static int vcu118_ipi_init(bool cold_boot)
{
	int rc;

	if (cold_boot) {
		rc = vcu118_clint_cold_ipi_init(VCU118_CLINT_ADDR,
						VCU118_HART_COUNT);
		if (rc)
			return rc;
	}

	return vcu118_clint_warm_ipi_init();
}

static int vcu118_timer_init(bool cold_boot)
{
	int rc;

	if (cold_boot) {
		rc = vcu118_clint_cold_timer_init(VCU118_CLINT_ADDR,
						  VCU118_HART_COUNT);
		if (rc)
			return rc;
	}

	return vcu118_clint_warm_timer_init();
}

static int vcu118_system_down(u32 type)
{
	/* For now nothing to do. */
	return 0;
}

const struct sbi_platform_operations platform_ops = {
	.pmp_region_count  = vcu118_pmp_region_count,
	.pmp_region_info   = vcu118_pmp_region_info,
	.final_init	   = vcu118_final_init,
	.console_putc      = sifive_uart_putc,
	.console_getc      = sifive_uart_getc,
	.console_init      = vcu118_console_init,
	.irqchip_init      = vcu118_irqchip_init,
	.ipi_send	   = vcu118_clint_ipi_send,
	.ipi_clear	   = vcu118_clint_ipi_clear,
	.ipi_init	   = vcu118_ipi_init,
	.timer_value       = vcu118_clint_timer_value,
	.timer_event_stop  = vcu118_clint_timer_event_stop,
	.timer_event_start = vcu118_clint_timer_event_start,
	.timer_init	   = vcu118_timer_init,
	.system_reboot     = vcu118_system_down,
	.system_shutdown   = vcu118_system_down
};

const struct sbi_platform platform = {
	.opensbi_version    = OPENSBI_VERSION,
	.platform_version   = SBI_PLATFORM_VERSION(0x0, 0x01),
	.name		    = "SiFive VCU118",
	.features	    = SBI_PLATFORM_DEFAULT_FEATURES,
	.hart_count	    = VCU118_HART_COUNT,
	.hart_stack_size    = VCU118_HART_STACK_SIZE,
	.disabled_hart_mask = VCU118_HARITD_DISABLED,
	.platform_ops_addr  = (unsigned long)&platform_ops
};
