/*
 * Copyright (c) 2025 AMD Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Driver for RISC-V Advanced Interrupt Architecture (IMSIC/APLIC)
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_INTC_RISCV_AIA_PRIV_H_
#define ZEPHYR_INCLUDE_DRIVERS_INTC_RISCV_AIA_PRIV_H_

#include <zephyr/device.h>

unsigned int imsic_attach_aplic(const struct device *dev, const struct device *aplic, size_t nvectors);
unsigned int imsic_intid(const struct device *dev, unsigned int irq);
void aplic_retrigger_level(const struct device *dev, unsigned int intid);
void aplic_set_intid_state(const struct device *dev, unsigned int intid, bool enable);
void aplic_set_intid_priority(const struct device *dev, unsigned int intid, unsigned int prio, uint32_t flags);

#endif /* ZEPHYR_INCLUDE_DRIVERS_INTC_RISCV_AIA_PRIV_H_ */
