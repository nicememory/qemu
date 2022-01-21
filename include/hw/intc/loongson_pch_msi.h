/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Loongson 7A1000 MSI I/O interrupt support
 *
 * Copyright (C) 2021 Loongson Technology Corporation Limited
 * Copyright (c) 2021 Jintao Yin <jintao.yin@i-soft.com.cn>
 */

#ifndef LOONGSON_PCH_MSI_H
#define LOONGSON_PCH_MSI_H

#include "hw/irq.h"
#include "qemu/units.h"
#include "hw/sysbus.h"
#include "qom/object.h"

/* Msi irq start start from 64 to 255 */
#define PCH_MSI_IRQ_START   64
#define PCH_MSI_IRQ_END     255
#define PCH_MSI_IRQ_NUM     192

typedef struct loongson_pch_msi {
    SysBusDevice parent_obj;
    qemu_irq pch_msi_irq[PCH_MSI_IRQ_NUM];
    MemoryRegion msi_mmio;
} loongson_pch_msi;

#define TYPE_LOONGSON_PCH_MSI "loongson_pch_msi"
OBJECT_DECLARE_SIMPLE_TYPE(loongson_pch_msi, LOONGSON_PCH_MSI)

#endif /* LOONGSON_PCH_MSI_H */
