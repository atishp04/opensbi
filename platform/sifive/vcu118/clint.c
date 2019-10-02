/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 SiFive Corporation.
 *
 * Authors:
 *   Zong Li <zong.li@sifive.com>
 */

#include <sbi/riscv_io.h>
#include <sbi/riscv_atomic.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_console.h>

#include "platform.h"
#include "clint.h"

static u32 clint_ipi_hart_count;
static volatile void *clint_ipi_base[HAWKSBILL_NODE_NUM];
static volatile u32 *clint_ipi[HAWKSBILL_NODE_NUM];

void vcu118_clint_ipi_send(u32 target_hart)
{
	u32 node_idx   = hawksbill_get_node_idx_by_target(target_hart);
	u32 target_idx = hawksbill_get_node_target_idx(target_hart);

	if (clint_ipi_hart_count <= target_idx)
		return;

	/* Set CLINT IPI */
	writel(1, &clint_ipi[node_idx][target_idx]);
}

void vcu118_clint_ipi_clear(u32 target_hart)
{
	u32 node_idx   = hawksbill_get_node_idx_by_target(target_hart);
	u32 target_idx = hawksbill_get_node_target_idx(target_hart);

	if (clint_ipi_hart_count <= target_idx)
		return;

	/* Clear CLINT IPI */
	writel(0, &clint_ipi[node_idx][target_idx]);
}

int vcu118_clint_warm_ipi_init(void)
{
	u32 hartid = sbi_current_hartid();

	if (!clint_ipi_base[hawksbill_get_node_idx_by_target(hartid)])
		return -1;

	/* Clear CLINT IPI */
	vcu118_clint_ipi_clear(hartid);

	return 0;
}

int vcu118_clint_cold_ipi_init(unsigned long base, u32 hart_count)
{
	/* Figure-out CLINT IPI register address */
	clint_ipi_hart_count = hart_count;
	clint_ipi_base[0]    = (void *)base;
	clint_ipi[0]	 = (u32 *)clint_ipi_base[0];
	clint_ipi_base[1]    = (void *)(base + HAWKSBILL_NODE2_BASE);
	clint_ipi[1]	 = (u32 *)clint_ipi_base[1];

	return 0;
}

static u32 clint_time_hart_count;
static volatile void *clint_time_base[HAWKSBILL_NODE_NUM];
static volatile u64 *clint_time_val[HAWKSBILL_NODE_NUM];
static volatile u64 *clint_time_cmp[HAWKSBILL_NODE_NUM];

static inline u32 vcu118_clint_time_read_hi(u32 idx)
{
	return readl_relaxed((u32 *)clint_time_val[idx] + 1);
}

static u64 vcu118_clint_timer_value_by_idx(u32 idx)
{
#if __riscv_xlen == 64
	return readq_relaxed(clint_time_val[idx]);
#else
	u32 lo, hi;

	do {
		hi = vcu118_clint_time_read_hi(idx);
		lo = readl_relaxed(clint_time_val[idx]);
	} while (hi != vcu118_clint_time_read_hi(idx));

	return ((u64)hi << 32) | (u64)lo;
#endif
}

u64 vcu118_clint_timer_value(void)
{
	return vcu118_clint_timer_value_by_idx(0);
}

void vcu118_clint_timer_event_stop(void)
{
	u32 target_hart = sbi_current_hartid();
	u32 node_idx   = hawksbill_get_node_idx_by_target(sbi_current_hartid());
	u32 target_idx = hawksbill_get_node_target_idx(target_hart);

	if (clint_time_hart_count <= target_idx)
		return;

		/* Clear CLINT Time Compare */
#if __riscv_xlen == 64
	writeq_relaxed(-1ULL, &clint_time_cmp[node_idx][target_idx]);
#else
	writel_relaxed(-1UL, &clint_time_cmp[node_idx][target_idx]);
	writel_relaxed(-1UL,
		       (void *)(&clint_time_cmp[node_idx][target_idx]) + 0x04);
#endif
}

void vcu118_clint_timer_event_start(u64 next_event)
{
	u32 target_hart = sbi_current_hartid();
	u32 node_idx   = hawksbill_get_node_idx_by_target(sbi_current_hartid());
	u32 target_idx = hawksbill_get_node_target_idx(target_hart);
	u64 mtime_master, mtime_own, mtime_diff;

	if (clint_time_hart_count <= target_idx)
		return;

	/*
	 * Correct the next_event if it isn't in master cluster.
	 *
	 * All clusters use mtime of clint0 as their timer, becasue we cannot
	 * synchronize mtime of each cluster by hardware. But timer interrupt
	 * are still triggered by comparing mtimecmp and mtime of clint which
	 * on the own cluster. It causes the timer interrupt become imprecise.
	 *
	 * The next_event is always a positive number.
	 */
	if (node_idx != 0) {
		mtime_master = vcu118_clint_timer_value_by_idx(0);
		mtime_own = vcu118_clint_timer_value_by_idx(node_idx);
		if (mtime_master > mtime_own) {
			mtime_diff = mtime_master - mtime_own;
			next_event -= mtime_diff;
		} else {
			mtime_diff = mtime_own - mtime_master;
			next_event += mtime_diff;
		}
	}

	/* Program CLINT Time Compare */
#if __riscv_xlen == 64
	writeq_relaxed(next_event, &clint_time_cmp[node_idx][target_idx]);
#else
	u32 mask = -1UL;
	writel_relaxed(next_event & mask,
		       &clint_time_cmp[node_idx][target_idx]);
	writel_relaxed(next_event >> 32,
		       (void *)(&clint_time_cmp[node_idx][target_idx]) + 0x04);
#endif
}

int vcu118_clint_warm_timer_init(void)
{
	u32 target_hart = sbi_current_hartid();
	u32 node_idx   = hawksbill_get_node_idx_by_target(sbi_current_hartid());
	u32 target_idx = hawksbill_get_node_target_idx(target_hart);

	if (clint_time_hart_count <= target_idx || !clint_time_base[node_idx])
		return -1;

		/* Clear CLINT Time Compare */
#if __riscv_xlen == 64
	writeq_relaxed(-1ULL, &clint_time_cmp[node_idx][target_idx]);
#else
	writel_relaxed(-1UL, &clint_time_cmp[node_idx][target_idx]);
	writel_relaxed(-1UL,
		       (void *)(&clint_time_cmp[node_idx][target_idx]) + 0x04);
#endif

	return 0;
}

int vcu118_clint_cold_timer_init(unsigned long base, u32 hart_count)
{
	/* Figure-out CLINT Time register address */
	clint_time_hart_count = hart_count;
	clint_time_base[0]    = (void *)base;
	clint_time_val[0]     = (u64 *)(clint_time_base[0] + 0xbff8);
	clint_time_cmp[0]     = (u64 *)(clint_time_base[0] + 0x4000);
	clint_time_base[1]    = (void *)(base + HAWKSBILL_NODE2_BASE);
	clint_time_val[1]     = (u64 *)(clint_time_base[1] + 0xbff8);
	clint_time_cmp[1]     = (u64 *)(clint_time_base[1] + 0x4000);

	return 0;
}
