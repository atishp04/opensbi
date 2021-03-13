/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Atish Patra <atish.patra@wdc.com>
 */

#include <sbi/sbi_ecall.h>
#include <sbi/sbi_ecall_interface.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_trap.h>
#include <sbi/sbi_version.h>
#include <sbi/sbi_pmu.h>
#include <sbi/sbi_scratch.h>
#include <sbi/riscv_asm.h>

static int sbi_ecall_pmu_handler(unsigned long extid, unsigned long funcid,
				 const struct sbi_trap_regs *regs,
				 unsigned long *out_val,
				 struct sbi_trap_info *out_trap)
{
	int ret = 0;
	uint64_t ival;

	switch (funcid) {
	case SBI_EXT_PMU_NUM_COUNTERS:
		ret = sbi_pmu_num_ctr();
		if (ret >= 0) {
			*out_val = ret;
			ret = 0;
		}
		break;
	case SBI_EXT_PMU_COUNTER_GET_INFO:
		ret = sbi_pmu_get_ctr_info(regs->a0, out_val);
		break;
	case SBI_EXT_PMU_COUNTER_CFG_MATCH:
		ret = sbi_pmu_get_ctr_match(regs->a0, regs->a1, regs->a2,
					    regs->a3, regs->a4);
		if (ret >= 0) {
			*out_val = ret;
			ret = 0;
		}

		break;
	case SBI_EXT_PMU_COUNTER_FW_READ:
		ret = sbi_pmu_read_ctr(regs->a0, out_val);
		break;
	case SBI_EXT_PMU_COUNTER_START:

#if __riscv_xlen == 32
		ival = ((uint64_t)regs->a1 << 32) | regs->a2;
#else
		ival = regs->a1;
#endif
		ret = sbi_pmu_start_ctr(regs->a0, ival);
		break;
	case SBI_EXT_PMU_COUNTER_STOP:
		ret = sbi_pmu_stop_ctr(regs->a0, regs->a1);
		break;
	default:
		ret = SBI_ENOTSUPP;
	};

	return ret;
}

struct sbi_ecall_extension ecall_pmu = {
	.extid_start = SBI_EXT_PMU,
	.extid_end = SBI_EXT_PMU,
	.handle = sbi_ecall_pmu_handler,
};
