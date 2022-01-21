/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Loongson IPI interrupt support
 *
 * Copyright (C) 2021 Loongson Technology Corporation Limited
 * Copyright (c) 2021 Jintao Yin <jintao.yin@i-soft.com.cn>
 */

#include "qemu/osdep.h"
#include "hw/intc/loongson_ipi.h"
#include "hw/boards.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "trace.h"
#include "hw/hw.h"

static uint64_t loongson_ipi_read(void *opaque, hwaddr addr, unsigned size)
{
    loongson_ipi_core *s = opaque;
    void *ptr;
    uint64_t ret = 0;
    switch (addr) {
    case CORE_IPI_STATUS:
        ret = s->status;
        break;
    case CORE_IPI_EN:
        ret = s->en;
        break;
    case CORE_IPI_SET:
        ret = 0;
        break;
    case CORE_IPI_CLEAR:
        ret = 0;
        break;
    case CORE_MAILBOX_0 ... CORE_MAILBOX_3:
        ptr = (void *)(addr - CORE_MAILBOX_0 + (uint8_t *)s->buf);
        if (size == 4) {
            ret = ldl_le_p(ptr);
        } else if (size == 8) {
            ret = ldq_le_p(ptr);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "invalid read: %x", (uint32_t)addr);
        exit(1);
        break;
    }

    trace_loongson_ipi_read(size, (uint64_t)addr, ret);
    return ret;
}

static void loongson_ipi_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    loongson_ipi_core *s = opaque;
    void *ptr;
    uint32_t old_status, new_status;
    trace_loongson_ipi_write(size, (uint64_t)addr, val);
    switch (addr) {
    case CORE_IPI_STATUS:
        qemu_log_mask(LOG_GUEST_ERROR, "can not be written");
        exit(1);
        break;
    case CORE_IPI_EN:
        old_status = s->status & s->en;
        s->en = val;
        new_status = s->status & s->en;
        if (new_status && !old_status) {
            qemu_irq_raise(s->irq);
        } else if (!new_status && old_status) {
            qemu_irq_lower(s->irq);
        }
        break;
    case CORE_IPI_SET:
        old_status = s->status & s->en;
        s->status |= val;
        new_status = s->status & s->en;
        if (new_status && !old_status) {
            qemu_irq_raise(s->irq);
        }
        break;
    case CORE_IPI_CLEAR:
        old_status = s->status & s->en;
        s->status ^= val;
        new_status = s->status & s->en;
        if (!new_status && old_status) {
            qemu_irq_lower(s->irq);
        }
        break;
    case CORE_MAILBOX_0 ... CORE_MAILBOX_3:
        ptr = (void *)(addr - CORE_MAILBOX_0 + (uint8_t *)s->buf);
        if (size == 4) {
            stl_le_p(ptr, val);
        } else if (size == 8) {
            stq_le_p(ptr, val);
        }
        break;
    case CORE_IPI_SEND:
        exit(1);
        break;
    case CORE_MAIL_SEND:
        exit(1);
        break;
    case CORE_FREQ_SEND:
        exit(1);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "invalid write: %x", (uint32_t)addr);
        exit(1);
        break;
    }
}

static const MemoryRegionOps loongson_ipi_ops = {
    .read = loongson_ipi_read,
    .write = loongson_ipi_write,
    .impl.min_access_size = 4,
    .impl.max_access_size = 8,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void loongson_ipi_init(Object *obj)
{
    loongson_ipi *s = LOONGSON_IPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    const unsigned int cpu_nums = MACHINE(qdev_get_machine())->smp.cpus;
    for (unsigned int i = 0; i < cpu_nums; i++) {
        loongson_ipi_core *c =  &s->core[i];
        MemoryRegion *mmio = &s->ipi_mmio[i];
        c->status = 0;
        c->en = 0;
        c->set = 0;
        c->clear = 0;
        c->buf[0] = 0;
        c->buf[1] = 0;
        c->buf[2] = 0;
        c->buf[3] = 0;
        c->id = i;
        memory_region_init_io(mmio, obj, &loongson_ipi_ops,
                              c, "loongson_ipi", CORE_IPI_LEN);
        sysbus_init_mmio(sbd, mmio);
        qdev_init_gpio_out(DEVICE(obj), &(c->irq), 1);
   }
}

static const VMStateDescription vmstate_loongson_ipi_core = {
    .name = "loongson_ipi_single",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(status, loongson_ipi_core),
        VMSTATE_UINT32(en, loongson_ipi_core),
        VMSTATE_UINT32(set, loongson_ipi_core),
        VMSTATE_UINT32(clear, loongson_ipi_core),
        VMSTATE_UINT64_ARRAY(buf, loongson_ipi_core, MAX_IPI_MBX_NUM),
        VMSTATE_UINT32(id, loongson_ipi_core),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_loongson_ipi = {
    .name = TYPE_LOONGSON_IPI,
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(core, loongson_ipi, MAX_IPI_CORE_NUM, 0,
                             vmstate_loongson_ipi_core, loongson_ipi_core),
        VMSTATE_END_OF_LIST()
    }
};

static void loongson_ipi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_loongson_ipi;
}

static const TypeInfo loongson_ipi_info = {
    .name          = TYPE_LOONGSON_IPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(loongson_ipi),
    .instance_init = loongson_ipi_init,
    .class_init    = loongson_ipi_class_init,
};

static void loongson_ipi_register_types(void)
{
    type_register_static(&loongson_ipi_info);
}

type_init(loongson_ipi_register_types)
