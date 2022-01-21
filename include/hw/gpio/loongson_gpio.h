/*
 * Loongson CPU general purpose input/output emulation
 *
 * Copyright (c) 2021 Jintao Yin <jintao.yin@i-soft.com.cn>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#ifndef HW_LOONGSON_GPIO_H
#define HW_LOONGSON_GPIO_H

#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qom/object.h"

#define LOONGSON_GPIO_PINS          32
#define LOONGSON_GPIO_LEN           0x18

#define LOONGSON_GPIO_REG_START     0x0
#define LOONGSON_GPIO_REG_END       0x18

typedef struct LoongsonGPIOState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    uint32_t output_en;
    uint32_t func_en;
    uint32_t output;
    uint32_t input;
    uint32_t int_pol;
    uint32_t int_en;

} LoongsonGPIOState;

#define TYPE_LOONGSON_GPIO "loongson_gpio"
OBJECT_DECLARE_SIMPLE_TYPE(LoongsonGPIOState, LOONGSON_GPIO)

#endif
