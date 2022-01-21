/*
 *  GPIO Controller for a lot of Freescale SoCs
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Alexander Graf, <agraf@suse.de>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "hw/gpio/loongson_gpio.h"
#include "qemu/error-report.h"

static uint64_t loongson_gpio_read(void *opaque, hwaddr offset,
                                   unsigned int size)
{
    LoongsonGPIOState *s = opaque;
    uint64_t ret = 0;
    switch (offset) {
    case LOONGSON_GPIO_REG_START ... LOONGSON_GPIO_REG_END:
        memcpy((void*)&ret, (void *)((uint8_t *)(&s->output_en) +
               offset - LOONGSON_GPIO_REG_START), size);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
    }
    return ret;
}

static void loongson_gpio_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned int size)
{
    LoongsonGPIOState *s = opaque;
    if (!((offset == 8 && (value == 2 || value == 0 || value == 0x1000)) || (offset == 0 && value == 0xffffeffdU) || (offset == 0 && value == 0xfffffffdU) || (offset == 4 && value == 0xffff0000U)  || (offset == 4 && value == 0xffff2000U))) {
        error_report("loongson_gpio_write opaue = '%p' addr = '0x%llx' val = '0x%llx' size = '%u'", opaque, offset, value, size);
        exit(1);
    }
    switch (offset) {
    case LOONGSON_GPIO_REG_START ... LOONGSON_GPIO_REG_END:
        memcpy((void *)((uint8_t *)(&s->output_en) +
               offset - LOONGSON_GPIO_REG_START),(void*)&value, size);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
    }
}

static const MemoryRegionOps loongson_gpio_ops = {
    .read =  loongson_gpio_read,
    .write = loongson_gpio_write,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void loongson_gpio_reset(DeviceState *dev)
{
    LoongsonGPIOState *s = LOONGSON_GPIO(dev);
    s->output_en = 0xffffffffU;
    s->func_en = 0xffff0000U;
    s->output = 0;
    s->input = 0;
    s->int_pol = 0;
    s->int_en = 0;
}

static const VMStateDescription vmstate_loongson_gpio = {
    .name = TYPE_LOONGSON_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(output_en, LoongsonGPIOState),
        VMSTATE_UINT32(func_en, LoongsonGPIOState),
        VMSTATE_UINT32(output, LoongsonGPIOState),
        VMSTATE_UINT32(input, LoongsonGPIOState),
        VMSTATE_UINT32(int_pol, LoongsonGPIOState),
        VMSTATE_UINT32(int_en, LoongsonGPIOState),
        VMSTATE_END_OF_LIST()
    }
};

static void loongson_gpio_init(Object *obj)
{
    LoongsonGPIOState *s = LOONGSON_GPIO(obj);
    memory_region_init_io(&s->mmio, obj, &loongson_gpio_ops, s,
            TYPE_LOONGSON_GPIO, LOONGSON_GPIO_LEN);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void loongson_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_loongson_gpio;
    dc->reset = loongson_gpio_reset;
    dc->desc = "Loongson GPIO";
}

static const TypeInfo loongson_gpio_info = {
    .name          = TYPE_LOONGSON_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LoongsonGPIOState),
    .instance_init = loongson_gpio_init,
    .class_init    = loongson_gpio_class_init,
};

static void loongson_gpio_register_types(void)
{
    type_register_static(&loongson_gpio_info);
}

type_init(loongson_gpio_register_types)
