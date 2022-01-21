/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Loongson extended interrupt controller support
 *
 * Copyright (C) 2021 Loongson Technology Corporation Limited
 * Copyright (c) 2021 Jintao Yin <jintao.yin@i-soft.com.cn>
 */

#ifndef LOONGSON_EXTIOI_H
#define LOONGSON_EXTIOI_H

#include "hw/irq.h"
#include "qemu/units.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define LOONGSON3_MAX_VCPUS        16
#define LOONGSON3_MIN_VCPUS        4
#define LOONGSON3_CORES_PER_NODE   4

#define LS3A_INTC_IP               4
#define MAX_CORES                  LOONGSON3_MAX_VCPUS
#define EXTIOI_IRQS                (256)
#define EXTIOI_IRQS_BITMAP_SIZE    (EXTIOI_IRQS / 8)
/* map to ipnum per 32 irqs */
#define EXTIOI_IRQS_PER_IPNUM      32
#define EXTIOI_IRQS_IPMAP_SIZE     (EXTIOI_IRQS / EXTIOI_IRQS_PER_IPNUM)
#define EXTIOI_IRQS_COREMAP_SIZE   EXTIOI_IRQS
#define EXTIOI_IRQS_NODETYPE_NUM  16
#define EXTIOI_IRQS_NODETYPE_SIZE  (EXTIOI_IRQS_NODETYPE_NUM * 2)

#define APIC_OFFSET                  0x400
#define APIC_BASE                    (0x1000ULL + APIC_OFFSET)

#define EXTIOI_NODETYPE_START        (0x4a0 - APIC_OFFSET)
#define EXTIOI_NODETYPE_END          (0x4c0 - APIC_OFFSET)
#define EXTIOI_IPMAP_START           (0x4c0 - APIC_OFFSET)
#define EXTIOI_IPMAP_END             (0x4c8 - APIC_OFFSET)
#define EXTIOI_ENABLE_START          (0x600 - APIC_OFFSET)
#define EXTIOI_ENABLE_END            (0x620 - APIC_OFFSET)
#define EXTIOI_BOUNCE_START          (0x680 - APIC_OFFSET)
#define EXTIOI_BOUNCE_END            (0x6a0 - APIC_OFFSET)
#define EXTIOI_ISR_START             (0x700 - APIC_OFFSET)
#define EXTIOI_ISR_END               (0x720 - APIC_OFFSET)
#define EXTIOI_COREISR_START         (0x800 - APIC_OFFSET)
#define EXTIOI_COREISR_END           (0xB20 - APIC_OFFSET)
#define EXTIOI_COREMAP_START         (0xC00 - APIC_OFFSET)
#define EXTIOI_COREMAP_END           (0xD00 - APIC_OFFSET)

typedef struct ext_sw_ipisr {
    uint8_t irq[EXTIOI_IRQS_BITMAP_SIZE];
} ext_sw_ipisr;

typedef struct loongson_extioi {
    SysBusDevice parent_obj;
    /* hardware state */
    uint8_t enable[EXTIOI_IRQS_BITMAP_SIZE];
    uint8_t bounce[EXTIOI_IRQS_BITMAP_SIZE];
    uint8_t coreisr[LOONGSON3_CORES_PER_NODE][EXTIOI_IRQS_BITMAP_SIZE];
    uint8_t ipmap[EXTIOI_IRQS_IPMAP_SIZE];
    uint8_t coremap[EXTIOI_IRQS_COREMAP_SIZE];
    uint8_t nodetype[EXTIOI_IRQS_NODETYPE_SIZE];

    /*software state */
    uint8_t sw_ipmap[EXTIOI_IRQS];
    uint8_t sw_coremap[EXTIOI_IRQS];
    ext_sw_ipisr sw_ipisr[LOONGSON3_CORES_PER_NODE][LS3A_INTC_IP];

    qemu_irq parent_irq[MAX_CORES][LS3A_INTC_IP];
    qemu_irq irq[EXTIOI_IRQS];
    MemoryRegion mmio;
} loongson_extioi;

#define TYPE_LOONGSON_EXTIOI "loongson_extioi"
OBJECT_DECLARE_SIMPLE_TYPE(loongson_extioi, LOONGSON_EXTIOI)

#endif /* LOONGSON_EXTIOI_H */
