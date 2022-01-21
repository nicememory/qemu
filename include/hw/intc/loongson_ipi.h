/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Loongson IPI interrupt support
 *
 * Copyright (C) 2021 Loongson Technology Corporation Limited
 * Copyright (c) 2021 Jintao Yin <jintao.yin@i-soft.com.cn>
 */

#ifndef LOONGSON_IPI_H
#define LOONGSON_IPI_H

#include "hw/irq.h"
#include "qemu/units.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define SMP_IPI_MAILBOX       0x1000ULL
#define CORE_IPI_STATUS       0x0
#define CORE_IPI_EN           0x4
#define CORE_IPI_SET          0x8
#define CORE_IPI_CLEAR        0xc
#define CORE_MAILBOX_0        0x20
#define CORE_MAILBOX_1        0x28
#define CORE_MAILBOX_2        0x30
#define CORE_MAILBOX_3        0x38

#define CORE_IPI_SEND         0x40
#define CORE_MAIL_SEND        0x48

#define CORE_FREQ_SEND        0x58

#define CORE_IPI_LEN          0x100
#define MAX_IPI_CORE_NUM      16
#define MAX_IPI_MBX_NUM       4

typedef struct loongson_ipi_core {
    uint32_t status;
    uint32_t en;
    uint32_t set;
    uint32_t clear;
    uint64_t buf[MAX_IPI_MBX_NUM];
    uint32_t id;
    qemu_irq irq;
} loongson_ipi_core;

typedef struct loongson_ipi {
    SysBusDevice parent_obj;
    loongson_ipi_core core[MAX_IPI_CORE_NUM];
    MemoryRegion ipi_mmio[MAX_IPI_CORE_NUM];
} loongson_ipi;

#define TYPE_LOONGSON_IPI "loongson_ipi"
OBJECT_DECLARE_SIMPLE_TYPE(loongson_ipi, LOONGSON_IPI)

#endif /* LOONGSON_IPI_H */
