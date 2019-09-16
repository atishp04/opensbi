/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 SiFive Corporation.
 *
 * Authors:
 *   Zong Li <zong.li@sifive.com>
 */

#ifndef __SYS_CLINT_H__
#define __SYS_CLINT_H__

#include <sbi/sbi_types.h>

void vcu118_clint_ipi_send(u32 target_hart);

void vcu118_clint_ipi_clear(u32 target_hart);

int vcu118_clint_warm_ipi_init(void);

int vcu118_clint_cold_ipi_init(unsigned long base, u32 hart_count);

u64 vcu118_clint_timer_value(void);

void vcu118_clint_timer_event_stop(void);

void vcu118_clint_timer_event_start(u64 next_event);

int vcu118_clint_warm_timer_init(void);

int vcu118_clint_cold_timer_init(unsigned long base, u32 hart_count);

#endif
