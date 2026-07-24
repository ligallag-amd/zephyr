/*
 * Copyright (c) 2025 AMD Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Driver for RISC-V Advanced Interrupt Architecture (IMSIC/APLIC)
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RISCV_AIA_H_
#define ZEPHYR_INCLUDE_DRIVERS_RISCV_AIA_H_

#include <zephyr/device.h>

struct riscv_aia_msi_vector {
	unsigned int irq;
	mem_addr_t address;
	uint16_t eventid;
};

int riscv_imsic_alloc_msi(struct riscv_aia_msi_vector *vectors, size_t nvectors);
int riscv_imsic_alloc_msi_cpu(struct riscv_aia_msi_vector *vectors, size_t nvectors, uint32_t proc_id);
int riscv_aplic_set_affinity(unsigned int irq, uint32_t proc_id);

#endif /* ZEPHYR_INCLUDE_DRIVERS_RISCV_AIA_H_ */
