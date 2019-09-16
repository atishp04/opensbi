/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 SiFive Corporation.
 *
 * Authors:
 *   Zong Li <zong.li@sifive.com>
 */

#ifndef __VCU118_PLIC_H__
#define __VCU118_PLIC_H__

#include <sbi/sbi_types.h>

void vcu118_plic_fdt_fixup(void *fdt, const char *compat);

int vcu118_plic_warm_irqchip_init(u32 target_hart, int m_cntx_id,
				  int s_cntx_id);

int vcu118_plic_cold_irqchip_init(unsigned long base, u32 num_sources,
				  u32 hart_count);

#endif
