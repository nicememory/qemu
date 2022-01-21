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

#ifndef HW_LOONGSON_SPI_H
#define HW_LOONGSON_SPI_H

#define LOONGSON_SPI_REG_SPCR       0x0
#define LOONGSON_SPI_REG_SPSR       0x1
#define LOONGSON_SPI_REG_TXFIFO     0x2
#define LOONGSON_SPI_REG_SPER       0x3
#define LOONGSON_SPI_REG_SFC_PARAM  0x4
#define LOONGSON_SPI_REG_SFC_SOFTCS 0x5
#define LOONGSON_SPI_REG_SFC_TIMING 0x6
#define LOONGSON_SPI_REG_CTRL       0x8
#define LOONGSON_SPI_REG_CMD        0x9
#define LOONGSON_SPI_REG_BUF0       0xa
#define LOONGSON_SPI_REG_BUF1       0xb
#define LOONGSON_SPI_REG_TIMER0     0xc
#define LOONGSON_SPI_REG_TIMER1     0xd
#define LOONGSON_SPI_REG_TIMER2     0xe
#define LOONGSON_SPI_REG_LEN        0xf

typedef struct LoongsonSPIState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
	uint8_t spcr;
	uint8_t spsr;
	uint8_t txfifo;
	uint8_t sper;
	uint8_t sfc_param;
	uint8_t sfc_softcs;
	uint8_t sfc_timing;
    uint8_t ctrl;
    uint8_t cmd;
    uint8_t buf0;
    uint8_t buf1;
    uint8_t timer0;
    uint8_t timer1;
    uint8_t timer2;

    qemu_irq irq;
    qemu_irq cs_line[4];
    SSIBus *ssi;
} LoongsonSPIState;

#define TYPE_LOONGSON_SPI "loongson_spi"
OBJECT_DECLARE_SIMPLE_TYPE(LoongsonSPIState, LOONGSON_SPI)

#endif /* HW_LOONGSON_SPI_H */
