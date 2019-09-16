/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 SiFive Corporation.
 *
 * Authors:
 *   Zong Li <zong.li@sifive.com>
 */

#include <sbi/riscv_io.h>
#include <sbi/riscv_encoding.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_string.h>
#include <libfdt.h>
#include <fdt.h>
#include <sbi/sbi_hart.h>

#include "platform.h"
#include "plic.h"

#define PLIC_PRIORITY_BASE 0x0
#define PLIC_PENDING_BASE 0x1000
#define PLIC_ENABLE_BASE 0x2000
#define PLIC_ENABLE_STRIDE 0x80
#define PLIC_CONTEXT_BASE 0x200000
#define PLIC_CONTEXT_STRIDE 0x1000

static u32 plic_hart_count;
static u32 plic_num_sources;
static volatile void *plic_base[HAWKSBILL_NODE_NUM];

static void vcu118_plic_set_priority(u32 source, u32 val, u32 idx)
{
	volatile void *plic_priority =
		plic_base[idx] + PLIC_PRIORITY_BASE + 4 * source;
	writel(val, plic_priority);
}

static void vcu118_plic_set_thresh(u32 cntxid, u32 val, u32 idx)
{
	volatile void *plic_thresh = plic_base[idx] + PLIC_CONTEXT_BASE +
				     PLIC_CONTEXT_STRIDE * cntxid;
	writel(val, plic_thresh);
}

static void vcu118_plic_set_ie(u32 cntxid, u32 word_index, u32 val, u32 idx)
{
	volatile void *plic_ie =
		plic_base[idx] + PLIC_ENABLE_BASE + PLIC_ENABLE_STRIDE * cntxid;
	writel(val, plic_ie + word_index * 4);
}

void vcu118_plic_fdt_fixup(void *fdt, const char *compat)
{
	u32 *cells;
	int i, cells_count;
	int plic_off;

	plic_off = fdt_node_offset_by_compatible(fdt, 0, compat);
	if (plic_off < 0)
		return;

	cells = (u32 *)fdt_getprop(fdt, plic_off, "interrupts-extended",
				   &cells_count);
	if (!cells)
		return;

	cells_count = cells_count / sizeof(u32);
	if (!cells_count)
		return;

	for (i = 0; i < (cells_count / 2); i++) {
		if (fdt32_to_cpu(cells[2 * i + 1]) == IRQ_M_EXT)
			cells[2 * i + 1] = cpu_to_fdt32(0xffffffff);
	}
}

int vcu118_plic_warm_irqchip_init(u32 target_hart, int m_cntx_id, int s_cntx_id)
{
	u32 idx = hawksbill_get_node_idx();
	size_t i, ie_words = plic_num_sources / 32 + 1;

	if (plic_hart_count <= target_hart)
		return -1;

	/* By default, disable all IRQs for M-mode of target HART */
	if (m_cntx_id > -1) {
		for (i = 0; i < ie_words; i++)
			vcu118_plic_set_ie(m_cntx_id, i, 0, idx);
	}

	/* By default, disable all IRQs for S-mode of target HART */
	if (s_cntx_id > -1) {
		for (i = 0; i < ie_words; i++)
			vcu118_plic_set_ie(s_cntx_id, i, 0, idx);
	}

	/* By default, enable M-mode threshold */
	if (m_cntx_id > -1)
		vcu118_plic_set_thresh(m_cntx_id, 1, idx);

	/* By default, disable S-mode threshold */
	if (s_cntx_id > -1)
		vcu118_plic_set_thresh(s_cntx_id, 0, idx);

	return 0;
}

int vcu118_plic_cold_irqchip_init(unsigned long base, u32 num_sources,
				  u32 hart_count)
{
	int i;

	plic_hart_count  = hart_count;
	plic_num_sources = num_sources;
	plic_base[0]     = (void *)base;
	plic_base[1]     = (void *)(base + HAWKSBILL_NODE2_BASE);

	/* Configure default priorities of all IRQs */
	for (i = 1; i <= plic_num_sources; i++) {
		vcu118_plic_set_priority(i, 1, 0);
		vcu118_plic_set_priority(i, 1, 1);
	}

	return 0;
}
