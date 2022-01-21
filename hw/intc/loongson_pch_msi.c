/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Loongson 7A1000 MSI I/O interrupt support
 *
 * Copyright (C) 2021 Loongson Technology Corporation Limited
 * Copyright (c) 2021 Jintao Yin <jintao.yin@i-soft.com.cn>
 */

#include "qemu/osdep.h"
#include "hw/pci/msi.h"
#include "hw/intc/loongson_pch_pic.h"
#include "hw/intc/loongson_pch_msi.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "trace.h"

static uint64_t loongson_msi_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void loongson_msi_mem_write(void *opaque, hwaddr addr,
                                    uint64_t val, unsigned size)
{
    loongson_pch_msi *s = LOONGSON_PCH_MSI(opaque);
    int irq_num = val & 0xff;

    trace_loongson_msi_set_irq(irq_num);
    qemu_set_irq(s->pch_msi_irq[irq_num - PCH_PIC_IRQ_NUM], 1);
}

static const MemoryRegionOps loongson_pch_msi_ops = {
    .read  = loongson_msi_mem_read,
    .write = loongson_msi_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void loongson_pch_msi_init(Object *obj)
{
    loongson_pch_msi *s = LOONGSON_PCH_MSI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->msi_mmio, obj, &loongson_pch_msi_ops,
                          s, TYPE_LOONGSON_PCH_MSI, 0x8);
    sysbus_init_mmio(sbd, &s->msi_mmio);
    msi_nonbroken = true;

    for (i = 0; i < PCH_MSI_IRQ_NUM; i++) {
        sysbus_init_irq(sbd, &s->pch_msi_irq[i]);
    }
}

static const TypeInfo loongson_pch_msi_info = {
    .name          = TYPE_LOONGSON_PCH_MSI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(loongson_pch_msi),
    .instance_init = loongson_pch_msi_init,
};

static void loongson_pch_msi_register_types(void)
{
    type_register_static(&loongson_pch_msi_info);
}

type_init(loongson_pch_msi_register_types)
