/*
 * Copyright (c) 2025 AMD, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Advanced Interrupt Architecture (IMSIC/APLIC) driver
 *        for RISC-V processors
 */

#include <stdlib.h>

#include "sw_isr_common.h"

#include <zephyr/debug/symtab.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree/interrupt_controller.h>

#include <zephyr/sw_isr_table.h>
#include <zephyr/drivers/interrupt_controller/riscv_aia.h>
#include <zephyr/irq.h>
#include <zephyr/irq_nextlevel.h>
#include "riscv_aia_priv.h"

#define DT_DRV_COMPAT riscv_imsics

/* TODO Supervisor mode support */

/* Additional AIA CSRs */
#define miselect 0x350
#define mireg    0x351
#define mtopei   0x35c

/* Additional AIA indirect CSRs */
#define eidelivery  0x70
#define eithreshold 0x72
#define eip0        0x80
#define eie0        0xC0

#ifdef CONFIG_64BIT
#define EI_PER_REG    64
#define EI_REG_STRIDE 2
#else
#define EI_PER_REG    32
#define EI_REG_STRIDE 1
#endif

#define EI_REG(irq) (((irq) / EI_PER_REG) * EI_REG_STRIDE)
#define EI_BIT(irq) BIT((irq) % EI_PER_REG)

/* IMSIC MMIO registers */
#define SETEIPNUM_LE 0x000
#define SETEIPNUM_BE 0x004

struct imsic_hart_context {
	mem_addr_t intr_file;
};

struct imsic_config {
	mem_addr_t reg;
	unsigned int irq;
	void (*irq_config_func)(void);
	unsigned int nr_ids;
	const struct imsic_hart_context *hart_context;
	unsigned int irq_offset;
};

struct imsic_data {
	struct k_spinlock lock;
	unsigned int nr_enabled;
	unsigned int nr_alloc;
#ifdef CONFIG_APLIC
	const struct device *aplic;
	unsigned aplic_offset;
	unsigned aplic_nr_ids;
#endif
};

#define IMSIC_NR_INTIDS(config) MIN((config)->nr_ids, CONFIG_MAX_IRQ_PER_AGGREGATOR)

unsigned int imsic_intid(const struct device *dev, unsigned int irq)
{
	const struct imsic_config *config = dev->config;
	__ASSERT_NO_MSG(irq > config->irq_offset);
	return irq - config->irq_offset;
}

static inline unsigned int aplic_intid(struct imsic_data *data, unsigned int intid)
{
	if (intid >= data->aplic_offset &&
	    intid < data->aplic_offset + data->aplic_nr_ids)
		return intid - data->aplic_offset + 1;

	/* Not APLIC interrupt */
	return 0;
}

static void imsic_irq_handler(const struct device *dev)
{
	const struct imsic_config *config = dev->config;
	unsigned long intid;
	struct _isr_table_entry *ite;

	/* Atomic read/clear mtopei */
	__asm__ volatile ("csrrw %0, " STRINGIFY(mtopei) ", x0" \
				: "=r" (intid));

	intid >>= 16;
	if (intid == 0 || intid >= IMSIC_NR_INTIDS(config)) {
		z_irq_spurious(NULL);
		return;
	}

	ite = &_sw_isr_table[config->irq_offset + intid];
	ite->isr(ite->arg);

#ifdef CONFIG_APLIC
	struct imsic_data *data = dev->data;
	unsigned int aintid = aplic_intid(data, intid);

	/* If this is an APLIC level-triggered interrupt and
	 * is still active, we need to have APLIC re-trigger
	 * it before we exit.
	 */
	if (aintid)
		aplic_retrigger_level(data->aplic, aintid);
#endif
}

static void imsic_intid_set_state(const struct device *dev, unsigned int intid, bool enable)
{
	const struct imsic_config *config = dev->config;
	struct imsic_data *data = dev->data;
	unsigned long val;

	__ASSERT_NO_MSG(intid > 0 && intid <= IMSIC_NR_INTIDS(config));

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	csr_write(miselect, eie0 + EI_REG(intid));

	if (enable) {
		val = csr_read_set(mireg, EI_BIT(intid));
		if ((val & EI_BIT(intid)) == 0) {
			++data->nr_enabled;
			__ASSERT_NO_MSG(data->nr_enabled <= IMSIC_NR_INTIDS(config));
		}
	} else {
		val = csr_read_clear(mireg, EI_BIT(intid));
		if ((val & EI_BIT(intid)) != 0) {
			__ASSERT_NO_MSG(data->nr_enabled);
			--data->nr_enabled;
		}
	}

#ifdef CONFIG_APLIC
	unsigned int aintid = aplic_intid(data, intid);
	if (aintid)
		aplic_set_intid_state(data->aplic, aintid, enable);
#endif

	k_spin_unlock(&data->lock, key);
}

static void imsic_irq_enable(const struct device *dev, unsigned int irq)
{
	imsic_intid_set_state(dev, imsic_intid(dev, irq), true);
}

static void imsic_irq_disable(const struct device *dev, unsigned int irq)
{
	imsic_intid_set_state(dev, imsic_intid(dev, irq), false);
}

static unsigned int imsic_get_state(const struct device *dev)
{
	struct imsic_data *data = dev->data;
	unsigned long val;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	val = data->nr_enabled;

	k_spin_unlock(&data->lock, key);

	return data->nr_enabled;
}

static int imsic_get_line_state(const struct device *dev, unsigned int irq)
{
	const struct imsic_config *config = dev->config;
	struct imsic_data *data = dev->data;
	unsigned int intid = imsic_intid(dev, irq);
	unsigned long val;

	__ASSERT_NO_MSG(intid > 0 && intid <= IMSIC_NR_INTIDS(config));

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	csr_write(miselect, eie0 + EI_REG(intid));
	val = csr_read(mireg);

	k_spin_unlock(&data->lock, key);

	return val & EI_BIT(intid);
}

static void imsic_set_priority(const struct device *dev,
			       unsigned int irq, unsigned int prio,
			       uint32_t flags)
{
#ifdef CONFIG_APLIC
	struct imsic_data *data = dev->data;
	unsigned int intid;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	intid = aplic_intid(data, imsic_intid(dev, irq));
	if (intid)
		aplic_set_intid_priority(data->aplic, intid, prio, flags);

	k_spin_unlock(&data->lock, key);
#endif
}

static int imsic_init(const struct device *dev)
{
	const struct imsic_config *config = dev->config;
	unsigned int eip = eip0;
	unsigned int eie = eie0;

	/* Disable and clear all interrupts */
	for (unsigned int id = 0; id < config->nr_ids; id += EI_PER_REG) {
		csr_write(miselect, eie);
		csr_write(mireg, 0);
		eie += EI_REG_STRIDE;
		csr_write(miselect, eip);
		csr_write(mireg, 0);
		eip += EI_REG_STRIDE;
	}

	/* Clear interrupt threshold, all interrupts enabled */
	csr_write(miselect, eithreshold);
	csr_write(mireg, 0);

	/* Enable interrupt delivery using MSI */
	csr_write(miselect, eidelivery);
	csr_write(mireg, 1);

	/* Configure IRQ handler */
	config->irq_config_func();

	return 0;
}

static int do_alloc_msi_range(const struct device *dev, size_t nvectors, unsigned int *offset)
{
	const struct imsic_config *config = dev->config;
	struct imsic_data *data = dev->data;

	if (data->nr_alloc + nvectors > IMSIC_NR_INTIDS(config))
		return -ENOSPC;

	*offset = data->nr_alloc + 1;
	data->nr_alloc += nvectors;

	return 0;
}

static int do_alloc_msi(const struct device *dev,
			struct riscv_aia_msi_vector *vectors, size_t nvectors,
			uint32_t proc_id)
{
	const struct imsic_config *config = dev->config;
	mem_addr_t msiaddr;
	unsigned int offset;
	int ret;

	ret = do_alloc_msi_range(dev, nvectors, &offset);
	if (ret < 0)
		return ret;

	__ASSERT_NO_MSG(proc_id < DT_CHILD_NUM(DT_PATH(cpus)));
	__ASSERT_NO_MSG(config->hart_context[proc_id].intr_file);

	msiaddr = config->hart_context[proc_id].intr_file + SETEIPNUM_LE;

	for (size_t i = 0; i < nvectors; ++i) {
		vectors->eventid = offset;
		vectors->address = msiaddr;
		vectors->irq = config->irq_offset + offset;
		offset++;
		vectors++;
	}

	return 0;
}

int riscv_imsic_alloc_msi(struct riscv_aia_msi_vector *vectors, size_t nvectors)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	struct imsic_data *data = dev->data;
	int ret;

	k_spinlock_key_t key = k_spin_lock(&data->lock);
	ret = do_alloc_msi(dev, vectors, nvectors, arch_proc_id());
	k_spin_unlock(&data->lock, key);

	return ret;
}

int riscv_imsic_alloc_msi_cpu(struct riscv_aia_msi_vector *vectors, size_t nvectors,
			      uint32_t proc_id)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	struct imsic_data *data = dev->data;
	int ret;

	k_spinlock_key_t key = k_spin_lock(&data->lock);
	ret = do_alloc_msi(dev, vectors, nvectors, proc_id);
	k_spin_unlock(&data->lock, key);

	return ret;
}

#ifdef CONFIG_APLIC
unsigned int imsic_attach_aplic(const struct device *dev,
				const struct device *aplic,
				size_t nvectors)
{
	struct imsic_data *data = dev->data;
	int ret;

	ret = do_alloc_msi_range(dev, nvectors, &data->aplic_offset);
	__ASSERT_NO_MSG(ret == 0);
	ARG_UNUSED(ret);

	data->aplic_nr_ids = nvectors;
	data->aplic = aplic;

	return data->aplic_offset;
}
#endif

static const struct irq_next_level_api imsic_api = {
	.intr_enable = imsic_irq_enable,
	.intr_disable = imsic_irq_disable,
	.intr_get_state = imsic_get_state,
	.intr_get_line_state = imsic_get_line_state,
	.intr_set_priority = imsic_set_priority
};

#define IMSIC_IRQ_FUNC_DECLARE(n) static void imsic_irq_config_func_##n(void)

#define IMSIC_IRQ_FUNC_DEFINE(n)                                                                   \
	static void imsic_irq_config_func_##n(void)                                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), 0, imsic_irq_handler, DEVICE_DT_INST_GET(n), 0);      \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

#define IMSIC_HART_CONTEXT(n) [DT_PROP(n, cpu_id)] = { .intr_file = DT_REG_ADDR(n) },

#define IMSIC_HART_CONTEXT_DECLARE(n)                                                              \
	static const struct imsic_hart_context imsic_hart_ctx_##n[DT_CHILD_NUM(DT_PATH(cpus))] = { \
		DT_INST_FOREACH_CHILD(n, IMSIC_HART_CONTEXT)                                       \
	}

#define IMSIC_CONFIG_INIT(n)                                                                       \
	IRQ_PARENT_ENTRY_DEFINE(                                                                   \
		imsic##n, DEVICE_DT_INST_GET(n), DT_INST_IRQN(n),                                  \
		INTC_INST_ISR_TBL_OFFSET(n),                                                       \
		DT_INST_INTC_GET_AGGREGATOR_LEVEL(n));                                             \
	IMSIC_IRQ_FUNC_DECLARE(n);                                                                 \
	IMSIC_HART_CONTEXT_DECLARE(n);                                                             \
	static const struct imsic_config imsic_config_##n = {                                      \
		.reg = DT_INST_REG_ADDR(n),                                                        \
		.irq = DT_INST_IRQN(n),                                                            \
		.irq_config_func = imsic_irq_config_func_##n,                                      \
		.nr_ids = DT_INST_PROP(n, riscv_num_ids),                                          \
		.irq_offset = INTC_INST_ISR_TBL_OFFSET(n),                                         \
		.hart_context = imsic_hart_ctx_##n,                                                \
	};                                                                                         \
	IMSIC_IRQ_FUNC_DEFINE(n)

#define IMSIC_DATA_INIT(n)                                                                         \
	static struct imsic_data imsic_data_##n;                                                   \

#define IMSIC_DEVICE_INIT(n)                                                                       \
	IMSIC_CONFIG_INIT(n)                                                                       \
	IMSIC_DATA_INIT(n)                                                                         \
	DEVICE_DT_INST_DEFINE(n, &imsic_init, NULL,                                                \
			      &imsic_data_##n, &imsic_config_##n,                                  \
			      PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY,                             \
			      &imsic_api);

DT_INST_FOREACH_STATUS_OKAY(IMSIC_DEVICE_INIT)
