# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_library_include_directories(${ZEPHYR_BASE}/arch/common/include)
zephyr_sources(
  soc_irq.S
  soc_common_irq.c
  vector.S
  )
