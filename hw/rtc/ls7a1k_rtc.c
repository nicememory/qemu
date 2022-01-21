/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Loongson 7A1000 RTC emulation
 *
 * Copyright (c) 2010 qiaochong@loongson.cn
 * Copyright (c) 2021 Jintao Yin <jintao.yin@i-soft.com.cn>
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
#include "qemu-common.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "sysemu/sysemu.h"
#include "migration/vmstate.h"

enum Ls7a1kRTCRegOffset {
    SYS_TOYTRIM = 0x20,
    SYS_TOYWRITE0 = 0x24,
    SYS_TOYWRITE1 = 0x28,
    SYS_TOYREAD0 = 0x2c,
    SYS_TOYREAD1 = 0x30,
    SYS_TOYMATCH0 = 0x34,
    SYS_TOYMATCH1 = 0x38,
    SYS_TOYMATCH2 = 0x3c,
    SYS_CNTRCTL = 0x40,
    
    SYS_RTCTRIM = 0x60,
    SYS_RTCWRITE0 = 0x64,
    SYS_RTCREAD0 = 0x68,
    SYS_RTCMATCH0 = 0x6c,
    SYS_RTCMATCH1 = 0x70,
    SYS_RTCMATCH2 = 0x74,

	SYS_RTC_IOMEM_SIZE = 0x100
};

enum Ls7a1kRTCCNTRCTLMask {
    TOYEN = 1 << 11,
    RTCEN = 1 << 13,
};

typedef struct Ls7a1kRTCState{
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    int64_t offset;
    int64_t save_alarm_offset;
    int64_t data;
    int tidx;
    uint32_t toymatch[3];
    uint32_t toytrim;
    uint32_t cntrctl;
    uint32_t rtctrim;
    uint32_t rtccount;
    uint32_t rtcmatch[3];
    qemu_irq toy_irq[3];
	qemu_irq rtc_irq[3];
    QEMUTimer *toy_timer;
} Ls7a1kRTCState;

static void update_toy_timer(Ls7a1kRTCState *s)
{
	int i;
	int imin,imax,inext;
	struct tm tm;
	uint64_t val, now, tmin, tmax, tnext;
	imin = imax = inext = 0;
	qemu_get_timedate(&tm, s->offset);
	tnext = tmin = tmax = now = \
	(tm.tm_sec << 0) | \
	(tm.tm_min << 6) | \
	(tm.tm_hour << 12) | \
	((tm.tm_mday-1) << 17) | \
	(tm.tm_mon << 22) |
	((tm.tm_year) << 26);

	for(i=0;i<3;i++)
	{
	val = s->toymatch[i];
	if(tmin > val) {tmin = val; imin = i;}
	if(tmax < val) {tmax = val; imax = i;}
	if(val > now  && ((tnext > val) | (tnext == now))) { tnext = val;  inext = i; }
	}

	if(tmax <= now)
	{
		s->tidx = imin;
		timer_mod(s->toy_timer, (tmin-now)*1000);
	}
	else
	{
		s->tidx = inext;
		timer_mod(s->toy_timer, (tnext-now)*1000);
	}

}

static uint64_t ls7a1k_rtc_read(void *opaque, hwaddr offset, unsigned size)
{
    Ls7a1kRTCState *s = opaque;
    uint64_t val = 0;
    struct tm tm;

    switch (offset) {
    case SYS_TOYTRIM:
        val = s->toytrim;
        break;
    case SYS_TOYREAD0:
	    qemu_get_timedate(&tm, s->offset);
	    val = ((qemu_clock_get_ms(QEMU_CLOCK_HOST)/100)%10)  | \
	    ((tm.tm_sec) << 4) | \
	    (((tm.tm_min)%60) << 10) | \
	    (((tm.tm_hour)%24) << 16) | \
	    (((tm.tm_mday) & 0x1f) << 21) | \
	    (((tm.tm_mon + 1) & 0x3f) << 26);
	    break;
    case SYS_TOYREAD1:
	    qemu_get_timedate(&tm, s->offset);
	    val = tm.tm_year;
	    break;
    case SYS_TOYMATCH0:
	    val = s->toymatch[0];
	    break;
    case SYS_TOYMATCH1:
	    val = s->toymatch[1];
	    break;
    case SYS_TOYMATCH2:
	    val = s->toymatch[2];
	    break;
    case SYS_CNTRCTL:
	    val = s->cntrctl;
	    break;
    case SYS_RTCREAD0:
	    val = s->rtccount;
	    break;
    case SYS_RTCMATCH0:
	    val = s->rtcmatch[0];
	    break;
    case SYS_RTCMATCH1:
	    val = s->rtcmatch[1];
	    break;
    case SYS_RTCMATCH2:
	    val = s->rtcmatch[2];
	    break;
    default:
	    break;
    }
    return val;
}

static void ls7a1k_rtc_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    Ls7a1kRTCState *s = opaque;
    struct tm tm;

    switch (offset) {
    case SYS_TOYWRITE0:
	    qemu_get_timedate(&tm, s->offset);
	    tm.tm_sec = (val>>4)&0x3f;
	    tm.tm_min = (val>>10)&0x3f;
	    tm.tm_hour = (val >> 16)&0x1f;
	    tm.tm_mday = ((val>>21) & 0x1f);
	    tm.tm_mon  = ((val>>26) & 0x3f) - 1;
        s->offset = qemu_timedate_diff(&tm);
        break;
    case SYS_TOYWRITE1:
	    qemu_get_timedate(&tm, s->offset);
	    tm.tm_year = val;
        s->offset = qemu_timedate_diff(&tm);
        break;
    case SYS_TOYMATCH0:
	    s->toymatch[0] = val;
	    update_toy_timer(s);
	    break;
    case SYS_TOYMATCH1:
	    s->toymatch[1] = val;
	    update_toy_timer(s);
	    break;
    case SYS_TOYMATCH2:
	    s->toymatch[2] = val;
	    update_toy_timer(s);
	    break;
    case SYS_CNTRCTL:
	    s->cntrctl = val;
	    break;
    case SYS_RTCWRITE0:
	    s->rtccount = val;
	    break;
    case SYS_RTCMATCH0:
	    s->rtcmatch[0] = val;
	    break;
    case SYS_RTCMATCH1:
	    s->rtcmatch[1] = val;
	    break;
    case SYS_RTCMATCH2:
	    s->rtcmatch[2] = val;
	    break;
    default:
        break;
    }
}

static void toy_timer(void *opaque)
{
	Ls7a1kRTCState *s = opaque;
	uint64_t data;
	int year, month, day;
	struct tm tm;
	data = s->toymatch[s->tidx];
	year = (data >> 26) & 0x3f;
	month = (data >> 22) & 0x1f;
	day = (data >> 17) & 0x1f;

	qemu_get_timedate(&tm, s->offset);

	if(tm.tm_mday-1 == day && tm.tm_mon == month && tm.tm_year == year && (s->cntrctl & TOYEN))
	 qemu_irq_raise(s->toy_irq[s->tidx]);
	update_toy_timer(s);
}

static const MemoryRegionOps ls7a1k_rtc_ops = {
    .read = ls7a1k_rtc_read,
    .write = ls7a1k_rtc_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

#define TYPE_LS7A1K_RTC "1s7a1k_rtc"
OBJECT_DECLARE_SIMPLE_TYPE(Ls7a1kRTCState, LS7A1K_RTC)

static void ls7a1k_rtc_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    Ls7a1kRTCState *s = LS7A1K_RTC(sbd);
    memory_region_init_io(&s->iomem, OBJECT(s), &ls7a1k_rtc_ops, (void *)s, TYPE_LS7A1K_RTC, SYS_RTC_IOMEM_SIZE);

    sysbus_init_irq(sbd, &s->toy_irq[0]);

    sysbus_init_mmio(sbd, &s->iomem);
    s->toy_timer = timer_new_ms(rtc_clock, toy_timer, s);
    s->offset = 0;
}

static int ls7a1k_rtc_pre_save(void *opaque)
{
    //Ls7a1kRTCState *s = opaque;
    // TODO add logic
    return 0;
}

static int ls7a1k_rtc_post_load(void *opaque, int version_id)
{
    //Ls7a1kRTCState *s = opaque;
    // TODO add logic
    return 0;
}

static const VMStateDescription vmstate_ls7a1k_rtc = {
    .name = TYPE_LS7A1K_RTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = ls7a1k_rtc_pre_save,
    .post_load = ls7a1k_rtc_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_INT64(offset, Ls7a1kRTCState),
        VMSTATE_INT64(save_alarm_offset, Ls7a1kRTCState),
        VMSTATE_UINT32(toymatch[0], Ls7a1kRTCState),
        VMSTATE_UINT32(cntrctl, Ls7a1kRTCState),
        VMSTATE_END_OF_LIST()
    }
};

static void ls7a1k_rtc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_ls7a1k_rtc;
    dc->realize = ls7a1k_rtc_realize;
    dc->desc = "Loongson 7A1000 RTC";
}

static const TypeInfo ls7a1k_rtc_info = {
    .name          = TYPE_LS7A1K_RTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Ls7a1kRTCState),
    .class_init    = ls7a1k_rtc_class_init,
};


static void ls7a1k_rtc_register_types(void)
{
    type_register_static(&ls7a1k_rtc_info);
}

type_init(ls7a1k_rtc_register_types)
