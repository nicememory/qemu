/*
 * Loongson SPI Controller emulation
 * 
 * Copyright (c) 2013 qiaochong@loongson.cn
 * Copyright (c) 2021 Jintao Yin <jintao.yin@i-soft.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "hw/pci/pci.h"
#include "hw/ssi/loongson_spi.h"
#include "qemu/error-report.h"

#define next_ptr(p) ((p + 1) & 7)

#define W_FULL  (1<<3)
#define W_EMPTY (1<<2)
#define R_FULL  (1<<1)
#define R_EMPTY (1<<0)

static uint64_t loongson_spi_read(void *opaque, hwaddr offset,
                                  unsigned size)
{
    LoongsonSPIState *s = opaque;
    uint32_t val;
    switch (offset) {
    case LOONGSON_SPI_REG_SFC_PARAM:
        return s->sfc_param;
    default:
        val = 0;
        break;
    }
    error_report("loongson_spi_read opaue = '%p' addr = '0x%llx' size = '%u'", opaque, offset, size);
    exit(1);
    return val;
}

static void loongson_spi_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    LoongsonSPIState *s = opaque;
    switch (offset) {
    case LOONGSON_SPI_REG_SFC_PARAM:
        s->sfc_param = value;
        return;
    default:
        break;
    }
    error_report("loongson_spi_write opaue = '%p' addr = '0x%llx' size = '%u'", opaque, offset, size);
    exit(1);
}

static void loongson_spi_reset(DeviceState *dev)
{
    LoongsonSPIState *s = LOONGSON_SPI(dev);
    s->spcr = 0x10;
    s->spsr = 5;
    s->txfifo = 0;
    s->sper = 0;
    s->sfc_param = 0x21;
    s->sfc_softcs = 0;
    s->sfc_timing = 3;
    s->ctrl = 0;
    s->cmd = 0;
    s->buf0 = 0;
    s->buf1 = 0;
    s->timer0 = 0;
    s->timer1 = 0;
    s->timer2 = 0;
}

static const MemoryRegionOps loongson_spi_ops = {
    .read = loongson_spi_read,
    .write = loongson_spi_write,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .valid.min_access_size = 1,
    .valid.max_access_size = 1,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void loongson_spi_realize(DeviceState *dev, Error **errp)
{
    LoongsonSPIState *d = LOONGSON_SPI(dev);
    memory_region_init_io(&d->mmio, NULL, &loongson_spi_ops, (void *)d,
                          TYPE_LOONGSON_SPI, LOONGSON_SPI_REG_LEN);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &d->mmio);
    d->ssi = ssi_create_bus(dev, "ssi");
    loongson_spi_reset(dev);
}

static void loongson_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = loongson_spi_realize;
    dc->reset = loongson_spi_reset;
    dc->desc = "Loongson SPI Controller";
}

static const TypeInfo loongson_spi_info = {
    .name          = TYPE_LOONGSON_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LoongsonSPIState),
    .class_init    = loongson_spi_class_init,
};


static void loongson_spi_register_types(void)
{
    type_register_static(&loongson_spi_info);
}

type_init(loongson_spi_register_types)
