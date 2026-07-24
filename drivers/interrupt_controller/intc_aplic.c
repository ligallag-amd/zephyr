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
#include <zephyr/dt-bindings/interrupt-controller/riscv-aplic.h>
#include <zephyr/irq.h>
#include <zephyr/irq_nextlevel.h>
#include "riscv_aia_priv.h"

#define DT_DRV_COMPAT riscv_aplic

/* TODO Supervisor mode support and delegation */

/* APLIC MMIO registers */
#define DOMAINCFG            0x0000
#define DOMAINCFG_IE         BIT(8)
#define DOMAINCFG_DM         BIT(2)
#define DOMAINCFG_BE         BIT(0)
#define SOURCECFG(i)         (0x0000 + 4 * (i))
#define SOURCECFG_D          BIT(10)
#define SOURCECFG_CI         GENMASK(9, 0)
#define SOURCECFG_SM         GENMASK(2, 0)
#define MMSIADDRCFG          0x1BC0
#define MMSIADDRCFGH         0x1BC4
#define MMSIADDRCFGH_L       BIT(31)
#define MMSIADDRCFGH_HHXS    GENMASK(28, 24)
#define MMSIADDRCFGH_LHXS    GENMASK(22, 20)
#define MMSIADDRCFGH_HHXW    GENMASK(18, 16)
#define MMSIADDRCFGH_LHXW    GENMASK(15, 12)
#define MMSIADDRCFGH_HBPPN   GENMASK(11, 0)
#define SMSIADDRCFG          0x1BC8
#define SMSIADDRCFGH         0x1BCC
#define SETIP(i)             (0x1C00 + 4 * (i))
#define SETIPNUM             0x1CDC
#define IN_CLRIP(i)          (0x1D00 + 4 * (i))
#define CLRIPNUM             0x1DDC
#define SETIE(i)             (0x1E00 + 4 * (i))
#define SETIENUM             0x1EDC
#define CLRIE(i)             (0x1F00 + 4 * (i))
#define CLRIENUM             0x1FDC
#define SETIPNUM_LE          0x2000
#define SETIPNUM_BE          0x2004
#define GENMSI               0x3000
#define GENMSI_HART_IDX      GENMASK(31, 18)
#define GENMSI_BUSY          BIT(12)
#define GENMSI_EEID          GENMASK(10, 0)
#define TARGET(i)            (0x3000 + 4 * (i))
#define TARGET_HART_IDX      GENMASK(31, 18)
#define TARGET_IPRIO         GENMASK(7, 0)
#define TARGET_GUEST_IDX     GENMASK(17, 12)
#define TARGET_EEID          GENMASK(10, 0)
#define IDELIVERY(i)         (0x4000 + 32 * (i))
#define IFORCE(i)            (0x4004 + 32 * (i))
#define ITHRESHOLD(i)        (0x4008 + 32 * (i))
#define TOPI(i)              (0x4018 + 32 * (i))
#define TOPI_IID             GENMASK(25, 16)
#define TOPI_PRIO            GENMASK(7, 0)
#define CLAIMI(i)            (0x401C + 32 * (i))
#define CLAIMI_IID           GENMASK(25, 16)
#define CLAIMI_PRIO          GENMASK(7, 0)

struct aplic_config {
	mem_addr_t reg;
#ifdef CONFIG_IMSIC
	const struct device *imsic;
#else
	unsigned int irq;
	void (*irq_config_func)(void);
	unsigned int irq_offset;
#endif
	unsigned int nr_ids;
};

struct aplic_data {
	struct k_spinlock lock;
	unsigned int nr_enabled;
#ifdef CONFIG_IMSIC
	unsigned int msi_offset;
#endif
};

#define APLIC_NR_INTIDS(config) MIN((config)->nr_ids, CONFIG_MAX_IRQ_PER_AGGREGATOR)
#define APLIC_NR_REGS(config) DIV_ROUND_UP((config)->nr_ids + 1, 32)

#ifndef CONFIG_IMSIC

static inline unsigned int aplic_intid(const struct device *dev, unsigned int irq)
{
	const struct aplic_config *config = dev->config;
	__ASSERT_NO_MSG(irq > config->irq_offset);
	return irq - config->irq_offset;
}

static void aplic_irq_handler(const struct device *dev)
{
	const struct aplic_config *config = dev->config;
	unsigned long intid;
	struct _isr_table_entry *ite;
	uint32_t cpu = arch_proc_id();

	intid = sys_read32(config->reg + CLAIMI(cpu));
	intid >>= 16;
	if (intid == 0 || intid >= APLIC_NR_INTIDS(config)) {
		z_irq_spurious(NULL);
		return;
	}

	ite = &_sw_isr_table[config->irq_offset + intid];
	ite->isr(ite->arg);
}

static void aplic_irq_enable(const struct device *dev, unsigned int irq)
{
	aplic_set_intid_state(dev, aplic_intid(dev, irq), true);
}

static void aplic_irq_disable(const struct device *dev, unsigned int irq)
{
	aplic_set_intid_state(dev, aplic_intid(dev, irq), false);
}

static unsigned int aplic_get_state(const struct device *dev)
{
	struct aplic_data *data = dev->data;
	unsigned long val;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	val = data->nr_enabled;

	k_spin_unlock(&data->lock, key);

	return data->nr_enabled;
}

static int aplic_get_line_state(const struct device *dev, unsigned int irq)
{
	const struct aplic_config *config = dev->config;
	struct aplic_data *data = dev->data;
	unsigned int intid = aplic_intid(dev, irq);
	unsigned long val;

	__ASSERT_NO_MSG(intid > 0 && intid <= APLIC_NR_INTIDS(config));

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	val = sys_read32(config->reg + SETIE(intid / 32));
	val = (val >> (intid % 32)) & 1;

	k_spin_unlock(&data->lock, key);

	return val;
}

static void aplic_set_priority(const struct device *dev,
			       unsigned int irq, unsigned int prio,
			       uint32_t flags)
{
	aplic_set_intid_priority(dev, aplic_intid(dev, irq), prio, flags);
}

#endif /* !CONFIG_IMSIC */

void aplic_set_intid_state(const struct device *dev, unsigned int intid, bool enable)
{
	const struct aplic_config *config = dev->config;
	struct aplic_data *data = dev->data;
	unsigned long val;

	__ASSERT_NO_MSG(intid > 0 && intid <= APLIC_NR_INTIDS(config));

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	val = sys_read32(config->reg + SETIE(intid / 32));
	val = (val >> (intid % 32)) & 1;

	if (enable & !val) {
		++data->nr_enabled;
		__ASSERT_NO_MSG(data->nr_enabled <= APLIC_NR_INTIDS(config));
		sys_write32(intid, config->reg + SETIENUM);
	} else if (!enable && val) {
		__ASSERT_NO_MSG(data->nr_enabled);
		--data->nr_enabled;
		sys_write32(intid, config->reg + CLRIENUM);
	}

	k_spin_unlock(&data->lock, key);
}

void aplic_set_intid_priority(const struct device *dev,
			      unsigned int intid, unsigned int prio,
			      uint32_t flags)
{
	const struct aplic_config *config = dev->config;
	struct aplic_data *data = dev->data;
	unsigned long val;
	uint32_t cpu = arch_proc_id();

	__ASSERT_NO_MSG(intid > 0 && intid <= APLIC_NR_INTIDS(config));

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	val = sys_read32(config->reg + SOURCECFG(intid)) & ~SOURCECFG_SM;
	sys_write32(val | FIELD_PREP(SOURCECFG_SM, flags),
		    config->reg + SOURCECFG(intid));
	sys_write32(FIELD_PREP(TARGET_HART_IDX, cpu) |
#ifdef CONFIG_IMSIC
		    FIELD_PREP(TARGET_EEID, data->msi_offset + intid - 1),
#else
		    FIELD_PREP(TARGET_IPRIO, prio),
#endif
		    config->reg + TARGET(intid));

	k_spin_unlock(&data->lock, key);
}

static inline const struct device *get_aplic_dev_from_irq(uint32_t irq)
{
#ifdef CONFIG_DYNAMIC_INTERRUPTS
	return z_get_sw_isr_device_from_irq(irq);
#else
	return DEVICE_DT_INST_GET(0);
#endif
}

int riscv_aplic_set_affinity(unsigned int irq, uint32_t proc_id)
{
	const struct device *dev = get_aplic_dev_from_irq(irq);
	const struct aplic_config *config = dev->config;
	struct aplic_data *data = dev->data;
	unsigned int intid;
	unsigned long val;

#ifdef CONFIG_IMSIC
	/* irq is in IMSIC domain, convert to APLIC intid */
	intid = imsic_intid(config->imsic, irq);
	__ASSERT_NO_MSG(intid >= data->msi_offset);
	intid = intid - data->msi_offset + 1;
#else
	intid = aplic_intid(dev, irq);
#endif

	__ASSERT_NO_MSG(intid > 0 && intid <= APLIC_NR_INTIDS(config));

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	val = sys_read32(config->reg + TARGET(intid)) & ~TARGET_HART_IDX;
	sys_write32(val | FIELD_PREP(TARGET_HART_IDX, proc_id),
		    config->reg + TARGET(intid));

	k_spin_unlock(&data->lock, key);

	return 0;
}

#ifdef CONFIG_IMSIC
void aplic_retrigger_level(const struct device *dev, unsigned int intid)
{
	const struct aplic_config *config = dev->config;
	unsigned long val, sense;

	__ASSERT_NO_MSG(intid > 0 && intid <= APLIC_NR_INTIDS(config));

	val = sys_read32(config->reg + SOURCECFG(intid));
	sense = FIELD_GET(SOURCECFG_SM, val);
	if (sense == IRQ_TYPE_LEVEL_HIGH || sense == IRQ_TYPE_LEVEL_LOW)
		sys_write32(intid, config->reg + SETIPNUM);
}
#endif

static int aplic_init(const struct device *dev)
{
	const struct aplic_config *config = dev->config;
	struct aplic_data *data = dev->data;
	uint32_t domaincfg = DOMAINCFG_IE;

	/* Set LE mode with interrupts disabled */
	sys_write32(0, config->reg + DOMAINCFG);

	/* Disable and clear all interrupts */
	for (int i = 0; i < APLIC_NR_REGS(config); ++i) {
		sys_write32(0xffffffff, config->reg + CLRIE(i));
		sys_write32(0xffffffff, config->reg + IN_CLRIP(i));
	}

#ifdef CONFIG_IMSIC
	__ASSERT_NO_MSG(device_is_ready(config->imsic));
	data->msi_offset = imsic_attach_aplic(config->imsic, dev,
					      APLIC_NR_INTIDS(config));
#endif

	for (unsigned int id = 1; id <= config->nr_ids; ++id) {
		/* Inactive, no delegation */
		sys_write32(0, config->reg + SOURCECFG(id));
	}

#ifdef CONFIG_IMSIC
	uint32_t msiaddrh = sys_read32(config->reg + MMSIADDRCFGH);
	if ((msiaddrh & MMSIADDRCFGH_L) == 0) {
		/* TODO Configure MSI target address if needed.
		 * For now, just assert we have hard-coded config.
		 */
		__ASSERT_NO_MSG(false);
	}
	domaincfg |= DOMAINCFG_DM;
#else
	/* Setup IDC */
	for (uint32_t cpu = 0; cpu < arch_num_cpus(); ++cpu) {
		sys_write32(0, config->reg + ITHRESHOLD(cpu));
		sys_write32(1, config->reg + IDELIVERY(cpu));
	}
#endif
	/* Enable interrupt delivery from this domain */
	sys_write32(domaincfg, config->reg + DOMAINCFG);

	return 0;
}

#define APLIC_DATA_INIT(n)                                                                         \
	static struct aplic_data aplic_data_##n;                                                   \

#ifdef CONFIG_IMSIC

#define APLIC_CONFIG_INIT(n)                                                                       \
	static const struct aplic_config aplic_config_##n = {                                      \
		.reg = DT_INST_REG_ADDR(n),                                                        \
		.imsic =  DEVICE_DT_GET(DT_INST_PROP(n, msi_parent)),                              \
		.nr_ids = DT_INST_PROP(n, riscv_num_sources),                                      \
	};                                                                                         \

#define APLIC_DEVICE_INIT(n)                                                                       \
	APLIC_CONFIG_INIT(n)                                                                       \
	APLIC_DATA_INIT(n)                                                                         \
	DEVICE_DT_INST_DEFINE(n, &aplic_init, NULL,                                                \
			      &aplic_data_##n, &aplic_config_##n,                                  \
			      PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY,                             \
			      NULL);

#else /* !CONFIG_IMSIC */

static const struct irq_next_level_api aplic_api = {
	.intr_enable = aplic_irq_enable,
	.intr_disable = aplic_irq_disable,
	.intr_get_state = aplic_get_state,
	.intr_get_line_state = aplic_get_line_state,
	.intr_set_priority = aplic_set_priority
};

#define APLIC_IRQ_FUNC_DECLARE(n) static void aplic_irq_config_func_##n(void)

#define APLIC_IRQ_FUNC_DEFINE(n)                                                                   \
	static void aplic_irq_config_func_##n(void)                                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), 0, aplic_irq_handler, DEVICE_DT_INST_GET(n), 0);      \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

#define APLIC_CONFIG_INIT(n)                                                                       \
	IRQ_PARENT_ENTRY_DEFINE(                                                                   \
		aplic##n, DEVICE_DT_INST_GET(n), DT_INST_IRQN(n),                                  \
		INTC_INST_ISR_TBL_OFFSET(n),                                                       \
		DT_INST_INTC_GET_AGGREGATOR_LEVEL(n));                                             \
	APLIC_IRQ_FUNC_DECLARE(n);                                                                 \
	static const struct aplic_config aplic_config_##n = {                                      \
		.reg = DT_INST_REG_ADDR(n),                                                        \
		.irq = DT_INST_IRQN(n),                                                            \
		.irq_config_func = aplic_irq_config_func_##n,                                      \
		.nr_ids = DT_INST_PROP(n, riscv_num_sources),                                      \
		.irq_offset = INTC_INST_ISR_TBL_OFFSET(n),                                         \
	};                                                                                         \
	APLIC_IRQ_FUNC_DEFINE(n)

#define APLIC_DEVICE_INIT(n)                                                                       \
	APLIC_CONFIG_INIT(n)                                                                       \
	APLIC_DATA_INIT(n)                                                                         \
	DEVICE_DT_INST_DEFINE(n, &aplic_init, NULL,                                                \
			      &aplic_data_##n, &aplic_config_##n,                                  \
			      PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY,                             \
			      &aplic_api);

#endif /* CONFIG_IMSIC */

DT_INST_FOREACH_STATUS_OKAY(APLIC_DEVICE_INIT)
