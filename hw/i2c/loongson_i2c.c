/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Loongson i2c emulation
 *
 * Copyright (c) 2013 qiaochong@loongson.cn
 * Copyright (C) 2021 Loongson Technology Corporation Limited
 * Copyright (c) 2021 Jintao Yin <jintao.yin@i-soft.com.cn>
 */

/*
 * QEMU loongson 1a i2c emulation
 *
 * Copyright (c) 2013 qiaochong@loongson.cn
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"

/* I2C Interface */
typedef struct LoongsonI2CState {
    SysBusDevice busdev;
    MemoryRegion iomem;
    I2CBus *bus;
    uint8_t prer_lo;
    uint8_t prer_hi;
    uint8_t control;
    uint8_t status;
    uint8_t slv_control;
    uint8_t data;
    qemu_irq irq;
} LoongsonI2CState;

#define REG_I2C_PRER_LO 0x0
#define REG_I2C_PRER_HI 0x1
#define REG_I2C_CTR     0x2
#define REG_I2C_TXR     0x3
#define REG_I2C_RXR     0x3
#define REG_I2C_CR      0x4
#define REG_I2C_SR      0x4
#define REG_I2C_SLV_CTR 0x7

#define I2C_CTR_EN  0x80
#define I2C_CTR_IEN 0x40
#define I2C_CTR_MST_EN 0x20

#define I2C_C_START 0x80
#define I2C_C_STOP 0x40
#define I2C_C_READ 0x20
#define I2C_C_WRITE 0x10
#define I2C_C_WACK 0x8
#define I2C_C_IACK 0x1

#define I2C_S_IF        0x1
#define I2C_S_TIP       0x2
#define I2C_S_AL        0x20
#define I2C_S_BUSY      0x40
#define I2C_S_RNOACK    0x80


#define I2C_LEN  8

static uint64_t loongson_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    LoongsonI2CState *s = opaque;
    switch (addr) {
    case REG_I2C_PRER_LO:
        return s->prer_lo;
    case REG_I2C_PRER_HI:
        return s->prer_hi;
    case REG_I2C_CTR:
        return s->control;
    case REG_I2C_SR:
        return s->status | (i2c_bus_busy(s->bus)? I2C_S_BUSY : 0);
    case REG_I2C_RXR:
        return s->data;
    case REG_I2C_SLV_CTR:
        return s->slv_control;
    default:
        hw_error("ls1a_i2c_read bad reg 0x%x\n", (unsigned)addr);
        break;
    }
    return 0;
}

static void loongson_i2c_check_irq(LoongsonI2CState *s)
{
    if (!(s->control & I2C_CTR_EN)) {
        return;
    }
    if ((s->status & I2C_S_IF) && (s->control & I2C_CTR_IEN)) {
        qemu_irq_raise(s->irq);
    } else {
        qemu_irq_lower(s->irq);
    }
}

static void loongson_i2c_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    LoongsonI2CState *s = opaque;
    int ack = 0;
    switch (addr) {
    case REG_I2C_CR:
        if (value & I2C_C_IACK) {
            s->status &= ~I2C_S_IF;
        }
        if (s->control & I2C_CTR_MST_EN) {
            if (value & (I2C_C_WRITE | I2C_C_READ | I2C_C_STOP)) {
                if ((value & (I2C_C_START | I2C_C_WRITE)) == (I2C_C_START | I2C_C_WRITE)) { /* START condition */
                    ack = !i2c_start_transfer(s->bus, (s->data & ~1) >> 1 , s->data & 1);
                } else {
                    if (value & I2C_C_READ) { /* RWM */
                        s->data = i2c_recv(s->bus);
                        if (value & I2C_C_WACK) {/* ACKNAK */
                            i2c_nack(s->bus);
                        }
                        ack = 1;
                    } else if(value & I2C_C_WRITE) {
                        ack = !i2c_send(s->bus, s->data);
                    }
                }
                if (value & I2C_C_STOP) {/* STOP condition */
                    i2c_end_transfer(s->bus);
                }
                s->status = I2C_S_IF;
                if (!ack) { 
                    s->status |= I2C_S_RNOACK;
                }
            }
            loongson_i2c_check_irq(s);
        } else {
            hw_error("can not be a slave!\n");
            exit(1);
        }
        break;

    case REG_I2C_PRER_LO:
        s->prer_lo = value;
        break;
    case REG_I2C_PRER_HI:
        s->prer_hi = value;
        break;
    case REG_I2C_CTR:
        s->control = value;
        loongson_i2c_check_irq(s);
        break;
    case REG_I2C_TXR:
        s->data = value;
        break;
    case REG_I2C_SLV_CTR:
        s->slv_control = value;
        break;
    default:
        hw_error("ls1a_i2c_write bad reg 0x%x\n", (unsigned)addr);
        break;
    }
}

static const MemoryRegionOps loongson_i2c_ops = {
    .read = loongson_i2c_read,
    .write = loongson_i2c_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

#define TYPE_LOONGSON_I2C "loongson_i2c"
OBJECT_DECLARE_SIMPLE_TYPE(LoongsonI2CState, LOONGSON_I2C)

static void loongson_i2c_reset(DeviceState *dev)
{
    LoongsonI2CState *s = LOONGSON_I2C(dev);
    s->prer_lo = 0xff;
    s->prer_hi = 0xff;
    s->control = 0x20;
    s->status = 0;
    s->slv_control = 0;
    s->data = 0;
}

static void loongson_i2c_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    LoongsonI2CState *d = LOONGSON_I2C(dev);

    memory_region_init_io(&d->iomem, NULL, &loongson_i2c_ops, (void *)d, TYPE_LOONGSON_I2C, I2C_LEN);
    sysbus_init_irq(sbd, &d->irq);
    sysbus_init_mmio(sbd, &d->iomem);
    d->bus = i2c_init_bus(DEVICE(dev), "i2c");
}

static const VMStateDescription loongson_i2c_vmstate = {
    .name = TYPE_LOONGSON_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(prer_lo, LoongsonI2CState),
        VMSTATE_UINT8(prer_hi, LoongsonI2CState),
        VMSTATE_UINT8(control, LoongsonI2CState),
        VMSTATE_UINT8(status, LoongsonI2CState),
        VMSTATE_UINT8(slv_control, LoongsonI2CState),
        VMSTATE_UINT8(data, LoongsonI2CState),
        VMSTATE_END_OF_LIST()
    },
};

static void loongson_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &loongson_i2c_vmstate;
    dc->reset = loongson_i2c_reset;
    dc->realize = loongson_i2c_realize;
    dc->desc = "Loongson I2C controller";
}

static const TypeInfo loongson_i2c_info = {
    .name          = TYPE_LOONGSON_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LoongsonI2CState),
    .class_init    = loongson_i2c_class_init,
};


static void loongson_i2c_register_types(void)
{
    type_register_static(&loongson_i2c_info);
}

type_init(loongson_i2c_register_types)
