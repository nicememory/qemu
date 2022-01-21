/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Loongson extended interrupt controller support
 *
 * Copyright (C) 2021 Loongson Technology Corporation Limited
 * Copyright (c) 2021 Jintao Yin <jintao.yin@i-soft.com.cn>
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/bswap.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "exec/address-spaces.h"
#include "hw/boards.h"
#include "hw/intc/loongson_extioi.h"
#include "migration/vmstate.h"
#include "trace.h"

static void extioi_update_irq(void *opaque, int irq_num, int level)
{
    loongson_extioi *s = LOONGSON_EXTIOI(opaque);
    uint8_t ipnum, corenum;
    unsigned long found1, found2;

    ipnum = s->sw_ipmap[irq_num];
    corenum = s->sw_coremap[irq_num];
    if (level == 1) {
        if (!test_bit(irq_num, (void *)s->enable)) {
            return;
        }
        bitmap_set((void *)s->coreisr[corenum], irq_num, 1);
        found1 = find_next_bit((void *)&(s->sw_ipisr[corenum][ipnum]),
                               EXTIOI_IRQS, 0);
        bitmap_set((void *)&(s->sw_ipisr[corenum][ipnum]), irq_num, 1);

        if (found1 >= EXTIOI_IRQS) {
            qemu_set_irq(s->parent_irq[corenum][ipnum], level);
        }
    } else {
        bitmap_clear((void *)s->coreisr[corenum], irq_num, 1);
        found1 = find_next_bit((void *)&(s->sw_ipisr[corenum][ipnum]),
                               EXTIOI_IRQS, 0);
        bitmap_clear((void *)&(s->sw_ipisr[corenum][ipnum]), irq_num, 1);
        found2 = find_next_bit((void *)&(s->sw_ipisr[corenum][ipnum]),
                               EXTIOI_IRQS, 0);

        if ((found1 < EXTIOI_IRQS) && (found2 >= EXTIOI_IRQS)) {
            qemu_set_irq(s->parent_irq[corenum][ipnum], level);
        }
    }
}

static void extioi_setirq(void *opaque, int irq, int level)
{
    loongson_extioi *s = LOONGSON_EXTIOI(opaque);
    trace_extioi_setirq(irq, level);
    extioi_update_irq(s, irq, level);
}

static uint64_t extioi_readfn(void *opaque, hwaddr addr, unsigned size)
{
    loongson_extioi *s = LOONGSON_EXTIOI(opaque);
    uint64_t ret = 0, offset, reg_count, corenum;
    if (size != 1) {
        g_assert_not_reached();
    }
    offset = addr;
    if ((offset >= EXTIOI_ENABLE_START) && (offset < EXTIOI_ENABLE_END)) {
        ret = s->enable[offset - EXTIOI_ENABLE_START];
    } else if ((offset >= EXTIOI_BOUNCE_START) && (offset < EXTIOI_BOUNCE_END)) {
        ret = s->bounce[offset - EXTIOI_BOUNCE_START];
    } else if ((offset >= EXTIOI_COREISR_START) && (offset < EXTIOI_COREISR_END)) {
        corenum = offset - EXTIOI_COREISR_START;
        reg_count = corenum & 0x1f;
        corenum >>= 8;
        corenum &= 0x3;
        ret = s->coreisr[corenum][reg_count];
    } else if ((offset >= EXTIOI_IPMAP_START) && (offset < EXTIOI_IPMAP_END)) {
        ret = s->ipmap[offset - EXTIOI_IPMAP_START];
    } else if ((offset >= EXTIOI_COREMAP_START) && (offset < EXTIOI_COREMAP_END)) {
        ret = s->coremap[offset - EXTIOI_COREMAP_START];
    } else if ((offset >= EXTIOI_NODETYPE_START) && (offset < EXTIOI_NODETYPE_END)) {
        ret = s->nodetype[offset - EXTIOI_NODETYPE_START];
    }    
    trace_loongson_extioi_read(size, addr, ret);
    return ret;
}

static void extioi_writefn(void *opaque, hwaddr addr,
                           uint64_t value, unsigned size)
{
    loongson_extioi *s = LOONGSON_EXTIOI(opaque);
    uint64_t offset = addr, reg_count, old_val, val,
                      bits, i, level, corenum, ipnum, irqnum;
    trace_loongson_extioi_write(size, addr, value);
    if (size != 1) {
        g_assert_not_reached();
    }
    val = value & 0xFF;
    if ((offset >= EXTIOI_ENABLE_START) && (offset < EXTIOI_ENABLE_END)) {
        reg_count = (offset - EXTIOI_ENABLE_START);
        old_val = s->enable[reg_count];
        if (old_val != val) {
            s->enable[reg_count] = val;
            old_val ^= val;
            bits = 8;
            while ((i = find_first_bit((void *)&old_val, bits)) != bits) {
                level = test_bit(i, (unsigned long *)&val);
                extioi_update_irq(s, i + reg_count * 8, level);
                clear_bit(i, (void *)&old_val);
            }
        }
    } else if ((offset >= EXTIOI_BOUNCE_START) && (offset < EXTIOI_BOUNCE_END)) {
        s->bounce[offset - EXTIOI_BOUNCE_START] = val;
    } else if ((offset >= EXTIOI_COREISR_START) && (offset < EXTIOI_COREISR_END)) {
        corenum = offset - EXTIOI_COREISR_START;
        reg_count = corenum & 0x1f;
        corenum >>= 8;
        corenum &= 0x3;
        old_val = s->coreisr[corenum][reg_count];
        s->coreisr[corenum][reg_count] = (old_val & ~val);

        if (old_val != (old_val & ~val)) {
            bits = 8;
            while ((i = find_first_bit((void *)&val, bits)) != bits) {
                level = test_bit(i, (unsigned long *)&old_val);
                if (level) {
                    extioi_update_irq(s, i + reg_count * 8, 0);
                }
                clear_bit(i, (void *)&val);
            }
        }
    } else if ((offset >= EXTIOI_IPMAP_START) && (offset < EXTIOI_IPMAP_END)) {
        reg_count = offset - EXTIOI_IPMAP_START;
        s->ipmap[reg_count] = val;
        /* Routing in groups of 32 interrupt */
        ipnum = find_first_bit((void *)&val, LS3A_INTC_IP);
        if (ipnum == LS3A_INTC_IP) {
            ipnum = 0;
        }
        irqnum = reg_count * EXTIOI_IRQS_PER_IPNUM;
        i = irqnum + EXTIOI_IRQS_PER_IPNUM;
        for (; irqnum < i; irqnum++) {
            s->sw_ipmap[irqnum] = ipnum;
        }
    } else if ((offset >= EXTIOI_COREMAP_START) && (offset < EXTIOI_COREMAP_END)) {
        reg_count = offset - EXTIOI_COREMAP_START;
        s->coremap[reg_count] = val;
        s->sw_coremap[reg_count] = val & 0xf;
    } else if ((offset >= EXTIOI_NODETYPE_START) && (offset < EXTIOI_NODETYPE_END)) {
        s->nodetype[offset - EXTIOI_NODETYPE_START] = val;
    }
}

static const MemoryRegionOps extioi_ops = {
    .read = extioi_readfn,
    .write = extioi_writefn,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void loongson_extioi_realize(DeviceState *dev, Error **errp)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    loongson_extioi *p = LOONGSON_EXTIOI(dev);
    int i, pin;

    for (i = 0; i < EXTIOI_IRQS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &p->irq[i]);
    }

    qdev_init_gpio_in(dev, extioi_setirq, EXTIOI_IRQS);
    memory_region_init_io(&p->mmio, OBJECT(p), &extioi_ops, p,
                          TYPE_LOONGSON_EXTIOI, 0x900);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &p->mmio);

    for (i = 0; i < ms->smp.cpus; i++) {
        for (pin = 0; pin < LS3A_INTC_IP; pin++) {
            qdev_init_gpio_out(dev, &p->parent_irq[i][pin], 1);
        }
    }
}

static const VMStateDescription vmstate_ext_sw_ipisr = {
    .name = "ext_sw_ipisr",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8_ARRAY(irq, ext_sw_ipisr, EXTIOI_IRQS_BITMAP_SIZE),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_loongson_extioi = {
    .name = TYPE_LOONGSON_EXTIOI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8_ARRAY(enable, loongson_extioi,
                             EXTIOI_IRQS_BITMAP_SIZE),
        VMSTATE_UINT8_ARRAY(bounce, loongson_extioi,
                             EXTIOI_IRQS_BITMAP_SIZE),
        VMSTATE_UINT8_2DARRAY(coreisr, loongson_extioi,
                               LOONGSON3_CORES_PER_NODE, EXTIOI_IRQS_BITMAP_SIZE),
        VMSTATE_UINT8_ARRAY(ipmap, loongson_extioi, 
                            EXTIOI_IRQS_IPMAP_SIZE),
        VMSTATE_UINT8_ARRAY(coremap, loongson_extioi,
                            EXTIOI_IRQS_COREMAP_SIZE),
        VMSTATE_UINT8_ARRAY(nodetype, loongson_extioi,
                            EXTIOI_IRQS_NODETYPE_SIZE),
        VMSTATE_UINT8_ARRAY(sw_ipmap, loongson_extioi, EXTIOI_IRQS),
        VMSTATE_UINT8_ARRAY(sw_coremap, loongson_extioi, EXTIOI_IRQS),
        VMSTATE_STRUCT_2DARRAY(sw_ipisr, loongson_extioi, LOONGSON3_CORES_PER_NODE,
                               LS3A_INTC_IP, 1, vmstate_ext_sw_ipisr,
                               ext_sw_ipisr),
        VMSTATE_END_OF_LIST()
    }
};

static void loongson_extioi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_loongson_extioi;
    dc->realize = loongson_extioi_realize;
}

static const TypeInfo loongson_extioi_info = {
    .name          = TYPE_LOONGSON_EXTIOI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(loongson_extioi),
    .class_init    = loongson_extioi_class_init,
};

static void loongson_extioi_register_types(void)
{
    type_register_static(&loongson_extioi_info);
}

type_init(loongson_extioi_register_types)
