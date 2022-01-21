/*
 * Loongson-3 7A1000 Platform support
 *
 * Copyright (c) 2018-2020 Huacai Chen (chenhc@lemote.com)
 * Copyright (c) 2018-2020 Jiaxun Yang <jiaxun.yang@flygoat.com>
 * Copyright (c) 2021 Loongson Technology Corporation Limited
 * Copyright (c) 2021 Jintao Yin <jintao.yin@i-soft.com.cn>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * 7A1000 virtualized PC Platform based on Loongson-3 CPU (MIPS64R2 with
 * extensions, 1500~2000MHz)
 */

#define LOONGSON3_CPU_NAME         "Loongson-3B4000"

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "qemu/units.h"
#include "qemu/datadir.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/char/serial.h"
#include "sysemu/sysemu.h"
#include "sysemu/qtest.h"
#include "sysemu/runstate.h"
#include "sysemu/reset.h"
#include "hw/mips/cpudevs.h"
#include "hw/mips/fw_cfg.h"
#include "kvm_mips.h"
#include "hw/pci-host/ls7a1k.h"
#include "hw/intc/loongson_liointc.h"
#include "hw/intc/loongson_ipi.h"
#include "hw/intc/loongson_extioi.h"
#include "hw/intc/loongson_pch_pic.h"
#include "hw/intc/loongson_pch_msi.h"
#include "net/net.h"
#include "hw/usb.h"
#include "hw/misc/unimp.h"
#include "hw/i2c/smbus_eeprom.h"
#include "hw/gpio/loongson_gpio.h"
#include "hw/ssi/loongson_spi.h"
#include "hw/hw.h"

#define LS7A1K_IOAPIC_REG_BASE  0x10000000UL
#define LS7A1K_PCH_MSI_ADDR_LOW 0x10000000UL
#define PM_MMIO_ADDR            0x10080000UL
#define PM_MMIO_SIZE            0x100
#define PM_CNTL_MODE            0x10
#define FEATURE_REG             0x8
#define IOCSRF_TEMP             0
#define IOCSRF_NODECNT          1
#define IOCSRF_MSI              2
#define IOCSRF_EXTIOI           3
#define IOCSRF_CSRIPI           4
#define IOCSRF_FREQCSR          5
#define IOCSRF_FREQSCALE        6
#define IOCSRF_DVFSV1           7
#define IOCSRF_GMOD             9
#define IOCSRF_VM               11

#define VENDOR_REG              0x10
#define CPUNAME_REG             0x20
#define ROUTE_CONF_REG          0x400
#define DEFAULT_RAM_SIZE        (4 * GiB)
#define LOONGSON3_7A1K_FWNAME   "fw_loongson3_7a1k.bin"
#define FW_CFG_ADDR             0x10080100
#define LA_BIOS_BASE            0x1fc00000
#define LA_BIOS_SIZE            (3 * MiB)

#define LS3A_CONF_REG_BASE      0x1fe00000
#define LS3A_I2C0_IRQ           (8)
#define LS3A_I2C0_BASE          LS3A_CONF_REG_BASE + 0x120
#define LS3A_I2C1_IRQ           (9)
#define LS3A_I2C1_BASE          LS3A_CONF_REG_BASE + 0x130
#define LS3A_UART0_IRQ          (10)
#define LS3A_UART0_BASE         LS3A_CONF_REG_BASE + 0x1e0
#define LS3A_UART1_IRQ          (15)
#define LS3A_UART1_BASE         LS3A_CONF_REG_BASE + 0x1e8

// SPI control regs
#define LS3A_SPI_REG_BASE       LS3A_CONF_REG_BASE + 0x1F0

// GPIO control regs
#define LS3A_GPIO_REG_BASE      LS3A_CONF_REG_BASE + 0x500

typedef struct Ls3APerNodeState {
    uint32_t ht0lo_reg_send_buf_debug_0x1d8;
    uint32_t ht0hi_reg_send_buf_debug_0x1d8;
    uint32_t ht1hi_reg_send_buf_debug_0x1d8;

    uint32_t ht0lo_reg_link_conf_ctrl_0x44;
    uint32_t ht0hi_reg_link_conf_ctrl_0x44;
    uint32_t ht1lo_reg_link_conf_ctrl_0x44;
    uint32_t ht1hi_reg_link_conf_ctrl_0x44;

    uint32_t ht0lo_reg_err_retry_0x64;
    uint32_t ht0hi_reg_err_retry_0x64;
    uint32_t ht1hi_reg_err_retry_0x64;

    uint32_t ht0lo_reg_rev_id_0x6c;
    uint32_t ht0hi_reg_rev_id_0x6c;
    uint32_t ht1hi_reg_rev_id_0x6c;

    uint32_t ht0lo_reg_bridge_ctrl_0x3c;
    uint32_t ht1lo_reg_bridge_ctrl_0x3c;
    uint32_t ht1hi_reg_bridge_ctrl_0x3c;

    uint32_t ht0lo_reg_link_train_0xd0;
    uint32_t ht0hi_reg_link_train_0xd0;
    uint32_t ht1hi_reg_link_train_0xd0;

    uint32_t ht1hi_reg_app_conf_0_0x1c0;

    uint32_t ht0lo_reg_soft_freq_conf_0x1f4;
    uint32_t ht0hi_reg_soft_freq_conf_0x1f4;
    uint32_t ht1lo_reg_soft_freq_conf_0x1f4;

} Ls3APerNodeState;

typedef struct Loongson37a1kMachineState {
    MachineState parent_obj;
    AddressSpace address_space_iocsr;
    MemoryRegion system_iocsr;
    MemoryRegion system_mem_conf;
    MemoryRegion lowmem;
    MemoryRegion highmem;
    MemoryRegion fwmem;
    MemoryRegion pci_mem_low_alias;
    MemoryRegion pci_io_alias;
    MemoryRegion pci_cfg_alias;
    /* State for other subsystems/APIs: */
    Ls3APerNodeState per_node[4];
} Loongson37a1kMachineState;

#define TYPE_LOONGSON3_7A1K_MACHINE MACHINE_TYPE_NAME("loongson3-7a1k")
OBJECT_DECLARE_SIMPLE_TYPE(Loongson37a1kMachineState, LOONGSON3_7A1K_MACHINE)

#define INT_ROUTER_REGS_BASE 0x3ff01400
#define HT_CONTROL_REGS_BASE 0xefdfb000000LL

#define DEF_LOONGSON3_FREQ (1500 * 1000 * 1000)

static uint64_t get_cpu_freq_hz(void)
{
#ifdef CONFIG_KVM
    int ret;
    uint64_t freq;
    struct kvm_one_reg freq_reg = {
        .id = KVM_REG_MIPS_COUNT_HZ,
        .addr = (uintptr_t)(&freq)
    };

    if (kvm_enabled()) {
        ret = kvm_vcpu_ioctl(first_cpu, KVM_GET_ONE_REG, &freq_reg);
        if (ret >= 0) {
            return freq * 2;
        }
    }
#endif
    return DEF_LOONGSON3_FREQ;
}

#define LOONGSON_SIMPLE_MMIO_OPS(ADDR, NAME, SIZE) \
({\
     MemoryRegion *iomem = g_new(MemoryRegion, 1);\
     memory_region_init_io(iomem, NULL, &loongson_qemu_ops,\
                           (void *)ADDR, NAME, SIZE);\
     memory_region_add_subregion(&lsms->system_iocsr, ADDR, iomem);\
})
uint8_t  reg120;
uint8_t  reg121;
uint8_t  reg122 = 0;
uint8_t  reg123;
uint8_t  reg124;

uint64_t reg180 =0x3B000F00BB003DE0ULL;
uint64_t reg190 =0x22FF000000000000ULL;
uint64_t reg1a0;
uint64_t reg1c0 = 104100ULL;
uint32_t reg1d0 = 0xffffu;
uint32_t reg_route_config_0x400 = 0xc0000030U;
uint32_t reg_route_config_0x404 = 0;

uint32_t reg_other_func_config_0x420 = 0x38000000;
uint32_t reg_other_func_config_0x424 = 0;
uint16_t ht_retry = 33;
uint8_t ht_gpio_en[57];
bool isMemConfAdded = FALSE;
bool isMemInited = FALSE;
bool isDramInited = FALSE;
uint64_t lvl_mode = 0;
uint64_t lvl_rdy_done = 0;
uint64_t lvl_resp = 0;
uint64_t lvl_resp1 = 0;

uint64_t dll_wrdq[9];
uint64_t dll_gate[9];

uint8_t memcfg[0x3400] = {0};

uint8_t mem_start[0x40] = {0};

bool isInMemCmdMode = FALSE;

static void loongson_qemu_mem_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    uint64_t offset = addr & 0xFFFFFFFFFFFULL;
    uint64_t idx = offset - 0xff00000ULL;
    uint64_t idx2 = 0;
    switch (offset) {
    case 0x0ULL ... 0x38ULL:
        if (size == 8) {
            stq_le_p((void *)(mem_start + offset), val);
            return;
        }
        break;
    case 0xff00000ULL:
        if (size == 4 && ((0x10 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    case 0xff00004ULL:
        if (size == 4 && ((0x0 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    case 0xff00008ULL:
        if (size == 4 && ((0x0 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x20 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    case 0xff0000cULL:
        if (size == 4 && ((0x20 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x0 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    case 0xff00010ULL:
        if (size == 4 && ((0x0 == val))) {
            isMemInited = FALSE;
            isDramInited = FALSE;
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 8 && ((0x1 == val))) {
            isMemInited = TRUE;
            isDramInited = TRUE;
            stq_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 8 && ((0x2 == val))) {
            isMemInited = FALSE;
            isDramInited = FALSE;
            stq_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 8 && ((0x3 == val))) {
            isMemInited = TRUE;
            isDramInited = TRUE;
            stq_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 8 && ((0x11 == val))) {
            isMemInited = TRUE;
            isDramInited = TRUE;
            stq_le_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    case 0xff00014ULL ... 0xff006ffULL:
    case 0xff00720ULL ... 0xff00fffULL:
        if (offset >= 0xff00100ULL && offset <= 0xff00688ULL) {
            idx = (offset - 0xff00100ULL) / 0x80UL;
            idx2 = (offset - 0xff00100ULL) % 0x80UL;
            if (idx2 < 8) {
                if (size == 8 && idx2 == 0) {
                    dll_wrdq[idx] = val;
                    return;
                } else if (size == 4 && (idx2 == 0 || idx2 == 4)) {
                    stl_le_p((void *)&dll_wrdq[idx] + idx2, val);
                    return;
                } else if (size == 1) {
                    stb_p((void *)&dll_wrdq[idx] + idx2, val);
                    if (val > 0x30 && idx2 == 0) {
                        int resp = ldub_p((void *)&lvl_resp + idx);
                        if (resp == 0) {
                            stb_p((void *)&lvl_resp + idx, 1);
                        }
                        return;
                    }
                    return;
                }
            } else if (idx2 < 16) {
                if (size == 8 && idx2 == 8) {
                    dll_gate[idx] = val;
                    return;
                } else if (size == 4 && (idx2 == 8 || idx2 == 12)) {
                    stl_le_p((void *)&dll_gate[idx] + idx2 - 8, val);
                    return;
                } else if (size == 1) {
                    stb_p((void *)&dll_gate[idx] + idx2 - 8, val);
                    if (val > 0x30 && idx2 == 8) {
                        int resp = ldub_p((void *)&lvl_resp + idx - 8);
                        if (resp == 0) {
                            stb_p((void *)&lvl_resp + idx - 8, 1);
                        }
                        return;
                    }
                    return;
                }
            } else {
                return;
            }
            break;
        }
        if (size == 4 && ((val == 0xa010a01ULL || (val == 0xa01ULL) || (val == 0x0ULL) || (val == 0xb42ULL) || (val == 0x89bf89bfULL) || (val == 0x89bfULL) || (val == 0x26d2dfULL) ||
        (val == 0x506647fULL) || (val == 0x3ed79ULL) || (val == 0x20101ULL)))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 8 && ((val == 0xa010a01ULL) || (val == 0xa010a010a010a01ULL) || (val == 0x10ULL))) {
            stq_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 1) {
            stb_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x0 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x1 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x40510 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x580101 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x140 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x28 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x3041428 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0xf00 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x12002012ULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0xb1818ULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x80ULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x82040088ULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x280000ULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x22002022ULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x1818ULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x81818ULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x91818ULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x14402001ULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x61002061ULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x69002069ULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x7a00207aULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 8 && ((0x1000000031ULL == val))) {
            stq_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 8 && ((0x124ULL == val))) {
            stq_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 8 && ((0x10100ULL == val))) {
            stq_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 8 && ((0x2000ULL == val))) {
            stq_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 8 && ((0x1919ULL == val))) {
            stq_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 8 && ((0 == val))) {
            stq_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x42002042ULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 4 && ((0x3a00203aULL == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 8 && ((0xb42 == val) || (val == 0xb44ULL))) {
            stq_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 1 && ((0x45ULL == val))) {
            stb_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 1 && ((0x26ULL == val))) {
            stb_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    case 0xff00700ULL:
        if (size == 8 && ((0x1100000 == val))) {
            lvl_mode = val;
            for (int i = 0; i < 9; i++) {
                dll_wrdq[i] = 0;
                dll_gate[i] = 0;
            }
            return;
        }
        if (size == 4 && ((0x1100000 == val))) {
            stl_le_p((void *)&lvl_mode, val);
            for (int i = 0; i < 9; i++) {
                dll_wrdq[i] = 0;
                dll_gate[i] = 0;
            }
            return;
        }
        if (size == 8 && ((0x1100001 == val))) {
            lvl_mode = val;
            stb_p((void *)&lvl_rdy_done, 0x1);
            return;
        }
        if (size == 8 && ((0x1100100 == val))) {
            lvl_mode = val;
            lvl_resp = 0;
            stb_p((void *)&lvl_rdy_done, 0x1);
            return;
        }
        if (size == 8 && ((0x1100101 == val))) {
            lvl_mode = val;
            stb_p((void *)&lvl_rdy_done + 1, 0x1);
            return;
        }
        if (size == 8 && ((0x1100102 == val))) {
            lvl_mode = val;
            stb_p((void *)&lvl_rdy_done + 1, 0x1);
            return;
        }
        if (size == 1 && ((0x2 == val))) {
            stb_p((void *)&lvl_mode, val);
            stb_p((void *)&lvl_rdy_done + 1, 0x1);
            return;
        }
        break;
    case 0xff00701ULL:
        if (size == 1 && ((0 == val))) {
            stb_p(((void *)&lvl_mode) + 1, val);
            return;
        }
        if (size == 1 && ((1 == val))) {
            stb_p(((void *)&lvl_mode) + 1, val);
            return;
        }
        break;
    case 0xff00704ULL:
        if (size == 4 && ((0 == val))) {
            stl_le_p(((void *)&lvl_mode) + 4, val);
            return;
        }
        break;
    case 0xff00708ULL:
        if (size == 4 && ((0 == val))) {
            stl_le_p((void *)&lvl_rdy_done, val);
            return;
        }
        break;
    case 0xff0070cULL:
        if (size == 4 && ((0 == val))) {
            stl_le_p(((void *)&lvl_rdy_done) + 4, val);
            return;
        }
        break;
    case 0xff00710ULL:
        if (size == 4 && ((0 == val))) {
            stl_le_p((void *)&lvl_resp, val);
            return;
        }
        break;
    case 0xff00714ULL:
        if (size == 4 && ((0 == val))) {
            stl_le_p(((void *)&lvl_resp) + 4, val);
            return;
        }
        break;
    case 0xff00718ULL:
        if (size == 4 && ((0 == val))) {
            stl_le_p((void *)&lvl_resp1, val);
            return;
        }
        break;
    case 0xff0071cULL:
        if (size == 4 && ((0 == val))) {
            stl_le_p(((void *)&lvl_resp1) + 4, val);
            return;
        }
        break;
    case 0xff01000ULL:
        if (size == 4 && ((0x54fc4 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 1 && ((0x28 == val))) {
            stb_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    case 0xff01001ULL:
        if (size == 1 && ((0x62 == val))) {
            stb_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    case 0xff01002ULL:
        if (size == 2 && ((0x8 == val))) {
            stw_le_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    case 0xff01004ULL:
        if (size == 4 && ((0xc1918 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        if (size == 1 && ((0x18 == val))) {
            stb_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    case 0xff01005ULL:
        if (size == 1 && ((0xff == val))) {
            return;
        }
        break;
    case 0xff01006ULL:
        if (size == 1 && ((0xb == val))) {
            stb_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    case 0xff01008ULL:
        if (size == 4 && ((0xa == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    case 0xff0100cULL:
        if (size == 4 && ((0x0 == val))) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    case 0xff01010ULL ... 0xff03400ULL:
        if (offset == 0xff01120ULL) {
            if (val == 1) {
                if (!isInMemCmdMode) {
                    memcfg[0x1122] = 1;
                    isInMemCmdMode = TRUE;
                }
            }else if (val == 0) {
                isInMemCmdMode = FALSE;
            }
        }
        if (offset == 0xff01126ULL && val == 1) {
            memcfg[0x1127] = 1;
        }
        if (size == 1) {
            stb_p((void *)(memcfg + idx), val);
            return;
        } else if (size == 2) {
            stw_le_p((void *)(memcfg + idx), val);
            return;
        }  else if (size == 4) {
            stl_le_p((void *)(memcfg + idx), val);
            return;
        } else if (size == 8) {
            stq_le_p((void *)(memcfg + idx), val);
            return;
        }
        break;
    default:
        break;
    }
    error_report("loongson_qemu_mem_write opaue = '%p' addr = '0x%llx' val = '0x%llx' size = '%u'", opaque, addr, val, size);
    exit(1);
}

static uint64_t loongson_qemu_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    //Loongson37a1kMachineState *lsms = opaque;
    uint64_t ret = 0UL;
    //uint64_t node_id = (addr & 0xF00000000000ULL) >> 44;
    uint64_t offset = addr & 0xFFFFFFFFFFFULL;
    uint64_t idx;
    uint64_t idx2;
    {
        if (offset >= 0xff00100ULL && offset <= 0xff00688ULL) {
            idx = (offset - 0xff00100ULL) / 0x80UL;
            idx2 = (offset - 0xff00100ULL) % 0x80UL;
            if (idx2 < 8) {
                if (size == 8 && idx2 == 0) {
                    return dll_wrdq[idx];
                } else if (size == 4 && (idx2 == 0 || idx2 == 4)) {
                    return ldl_le_p((void *)&dll_wrdq[idx] + idx2);
                } else if (size == 1) {          
                    return ldub_p((void *)&dll_wrdq[idx] + idx2);
                }
            } else if (idx2 < 16) {
                if (size == 8 && idx2 == 8) {
                    return dll_gate[idx];
                } else if (size == 4 && (idx2 == 8 || idx2 == 12)) {
                    return ldl_le_p((void *)&dll_gate[idx] + idx2 - 8);
                } else if (size == 1) { 
                    return ldub_p((void *)&dll_gate[idx] + idx2 - 8);
                }
            } else {
                return 0;
            }
        }
    }
    idx = offset - 0xff00000ULL;
    switch (offset) {
    case 0x0ULL ... 0x38ULL:
        if (size == 8) {
            return ldq_le_p((void *)(mem_start + offset));
        }
        break;
    case 0xff00000ULL:
        if (size == 4) {
            return 16;
        } else if (size == 1) {
            return 16;
        } else if (size == 2) {
            return 16;
        }
        break;
    case 0xff00010ULL:
        if (size == 8) {
             if (isDramInited && isMemInited) {
                return 0x101;
            } else if (isDramInited){
                return 0x100;
            } else if (isMemInited) {
                return 1;
            } else {
                return 0;
            }
        }
        break;
    case 0xff00030ULL:
        if (size == 8) {
            return 0x10000000000ULL;
        }
        break;
    case 0xff00048ULL:
        if (size == 8) {
            return 0x10000;
        }
        break;
    case 0xff00078ULL:
        if (size == 1) {
            return 0;
        }
        if (size == 8) {
            return 0x10000;
        }
        break;
    case 0xff000b8ULL:
        if (size == 1) {
            return 0;
        } 
        if (size == 8) {
            return 0x10000;
        }
        break;
    case 0xff000e0ULL ... 0xff000e3ULL:
        if (size == 1) {
            return ldub_p((void *)(memcfg + idx));
        }
        break;
    case 0xff00700ULL:
        if (size == 8) {
            return lvl_mode;
        }
        break;
    case 0xff00708ULL:
        if (size == 1) {
            return ldub_p((void *)&lvl_rdy_done);
        }
        if (size == 8) {
            return lvl_rdy_done;
        }
        break;
    case 0xff00709ULL:
        if (size == 1) {
            return ldub_p((void *)&lvl_rdy_done + 1);
        }
        break;
    case 0xff00710ULL:
        if (size == 8) {
            return lvl_resp;
        }
        break;
    case 0xff00718ULL:
        if (size == 8) {
            if ((lvl_mode & 0x3) == 1) {
                hw_error("Here is not for you!");
            }
            return lvl_resp1;
        }
        break;
    case 0xff00832ULL:
        if (size == 2) {
            return lduw_le_p((void *)(memcfg + idx));
        }
        break;
    case 0xff00908ULL:
        if (size == 1) {
            return 0x0;
        }
        break;
    case 0xff00909ULL:
        if (size == 1) {
            return 0x20;
        }
        break;
    case 0xff0090aULL:
        if (size == 1) {
            return 0;
        }
        break;
    case 0xff01004ULL:
        if (size == 1) {
            return 0;
        }
        break;
    case 0xff01012ULL:
        if (size == 2) {
            return 0;
        }
        break;
    case 0xff01060ULL:
        if (size == 1) {
            return ldub_p((void *)(memcfg + idx));
        } else if (size == 8) {
            return ldq_le_p((void *)(memcfg + idx));
        }
        break;
    case 0xff01062ULL:
        if (size == 1) {
            return ldub_p((void *)(memcfg + idx));
        } else if (size == 2) {
            return lduw_le_p((void *)(memcfg + idx));
        }
        break;
    case 0xff01100ULL:
        if (size == 8) {
            return 1;
        }
        break;
    case 0xff01122ULL ... 0xff032ffULL:
        if (size == 1) {
            return ldub_p((void *)(memcfg + idx));
        } else if (size == 2) {
            return lduw_le_p((void *)(memcfg + idx));
        } else if (size == 8) {
            if (0xff03160ULL == offset) {
                return 0;
            }
            return ldq_le_p((void *)(memcfg + idx));
        }
        break;
    case 0xff03300ULL ... 0xff03400ULL:
        if (size == 1) {
            if (offset == 0xff03330ULL) {
                return 0xff;
            }
            if (offset == 0xff03302ULL) {
                return 51;
            }
            return ldub_p((void *)(memcfg + idx));
        } else if (size == 2) {
            if (offset == 0xff03330ULL) {
                return 0xff;
            }
            return lduw_le_p((void *)(memcfg + idx));
        } else if (size == 8) {
            return ldq_le_p((void *)(memcfg + idx));
        }
        break;
    default:
        break;
    }
    error_report("loongson_qemu_mem_read opaue = '%p' addr = '0x%llx' size = '%u'", opaque, addr, size);
    exit(1);
    return ret;
}
static void loongson_qemu_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    Loongson37a1kMachineState *lsms = opaque;
    uint64_t node_id = (addr & 0xF00000000000ULL) >> 44;
    uint64_t offset = addr & 0xFFFFFFFFFFFULL;
    uint64_t ret;
    if (node_id >= 4) {
        return;
    }
    switch (offset) {
    case 0xe00100e0900ULL ... 0xe00100e0939: // GPIO en
        if (1 == size && ((0 == val) || (1 == val))) {
            ht_gpio_en[offset - 0xe00100e0900ULL] = val;
            return;
        }
        break;
    case 0xe00100d000cULL: // ACPI PM1 Status
        if (4 == size && 0 == val) {
            return;
        }
        break;
    case 0xe00100d0028ULL: // ACPI
        if (4 == size && 0 == val) {
            return;
        }
        break;
    case 0xe00100d002CULL: // ACPI
        if (4 == size && 0 == val) {
            return;
        }
        break;
    case 0x1fe00008:
        if (val == 0x2e && size == 4) {
            return;
        } else if (val == 0x27 && size == 4) {
            return;
        } 
        break;
    case 0x1fe00120:
        if (size == 1 && 0x53 == val) {
            reg120 = val;
            return;
        }
        break;
    case 0x1fe00121:
        if (size == 1 && 0x2 == val) {
            reg121 = val;
            return;
        }
        break;
    case 0x1fe00122:
        if (size == 1 && ((0 == val) || (0x80 == val))) {
            reg122 = val;
            return;
        }
        break;
    case 0x1fe00123:
        if (size == 1 && 0xd6 == val) {
            reg123 = val;
            return;
        }
        break;
    case 0x1fe00124:
        if (size == 1 && ((0x40 == val) || (0x90 == val))) {
            if (0x40 == val) {
                return;
            }
            reg124 = val;
            return;
        }
        break;
    case 0x1fe00180:
    case 0x3ff00180:
        if (0xbb0035a0ULL == val && 4 == size) {
            if (!isMemConfAdded) {
                memory_region_add_subregion_overlap(&lsms->system_iocsr, 0, &lsms->system_mem_conf, 1024);
                isMemConfAdded = TRUE;
            }
            stl_le_p((void *)(&reg180), val);
        } else if (8 == size) {
            reg180 = val;
        }
        return;
    case 0x1fe001a0:
        reg1a0 = val;
        return;
    case 0x1fe001c0:
        if (4 == size) {
            reg1c0 = val;
            return;
        }
        break;
    case 0x1fe001d0:
        if (0xffffU == val || 0xfU == val || 0 == val) {
            reg1d0 = val;
            return;
        }
        break;
    case 0x1fe00400:
        if (4 == size) {
            reg_route_config_0x400 = val;
            return;
        } else if (8 == size) {
            reg_route_config_0x400 = val;
            reg_route_config_0x404 = val >> 32;
            return;
        }
        break;
    case 0x1fe00404:
        if (4 == size) {
            reg_route_config_0x404 = val;
        }
        break;
    case 0x1fe00420:
        if (4 == size && 0x38080000U == val) {
            reg_other_func_config_0x420 = val;
            return;
        } else if (8 == size) {
            reg_other_func_config_0x420 = val;
            reg_other_func_config_0x424 = val >> 32;
        }
        break;
    case 0x1fe00424:
        if (4 == size && 0x8000U == val) {
            reg_other_func_config_0x424 = val;
            return;
        }
        break;
    case 0x1fe00430:
        if (4 == size && 0x2be10faU == val) {
            return;
        }
        break;
    case 0x1fe00440:
        if (0x0U == val) {
            return;
        }
        break;
    case 0x1fe00448:
        if (0x0U == val) {
            return;
        }
        break;
    case 0x3ff00000U + ROUTE_CONF_REG:
        if (4 == size) {
            reg_route_config_0x400 = val;
            return;
        } else if (8 == size) {
            reg_route_config_0x400 = val;
            reg_route_config_0x404 = val >> 32;
            return;
        }
        break;
    case 0x3ff00403U:
        if (1 == size) {
            stb_p((void *)&reg_route_config_0x400 + 3, val); 
            return;
        }
        break;
    case 0x3ff00404U:
        if (1 == size) {
            stb_p((void *)&reg_route_config_0x404, val); 
            return;
        }
        break;
    case 0x3ff00000U + 0x208:
        if (8 == size && 0x8000000010000000ULL == val) {
            return;
        }
        break;
    case 0x3ff00000U + 0x248:
        if (8 == size && 0xfffffffffffff000ULL == val) {
            return;
        }
        break;
    case 0x3ff00000U + 0x2000:
    case 0x3ff00000U + 0x2100:
    case 0x3ff00000U + 0x2200:
    case 0x3ff00000U + 0x2300:
    case 0x3ff00000U + 0x2a00:
    case 0x3ff00000U + 0x2b00:
    case 0x3ff00000U + 0x2c00:
    case 0x3ff00000U + 0x2d00:
    case 0x3ff00000U + 0x2e00:
    case 0x3ff00000U + 0x2f00:
        if (8 == size && 0x18000000ULL == val) {
            return;
        }
        break;
    case 0x3ff00000U + 0x2008:
    case 0x3ff00000U + 0x2108:
    case 0x3ff00000U + 0x2208:
    case 0x3ff00000U + 0x2308:
    case 0x3ff00000U + 0x2a08:
    case 0x3ff00000U + 0x2b08:
    case 0x3ff00000U + 0x2c08:
    case 0x3ff00000U + 0x2d08:
    case 0x3ff00000U + 0x2e08:
    case 0x3ff00000U + 0x2f08:
        if (8 == size && 0x10000000ULL == val) {
            return;
        }
        break;
    case 0x3ff00000U + 0x2010:
    case 0x3ff00000U + 0x2110:
    case 0x3ff00000U + 0x2210:
    case 0x3ff00000U + 0x2310:
    case 0x3ff00000U + 0x2a10:
    case 0x3ff00000U + 0x2b10:
    case 0x3ff00000U + 0x2c10:
    case 0x3ff00000U + 0x2d10:
    case 0x3ff00000U + 0x2e10:
    case 0x3ff00000U + 0x2f10:
        if (8 == size && 0x40000000ULL == val) {
            return;
        }
        break;
    case 0x3ff00000U + 0x2040:
    case 0x3ff00000U + 0x2140:
    case 0x3ff00000U + 0x2240:
    case 0x3ff00000U + 0x2340:
    case 0x3ff00000U + 0x2a40:
    case 0x3ff00000U + 0x2b40:
    case 0x3ff00000U + 0x2c40:
    case 0x3ff00000U + 0x2d40:
    case 0x3ff00000U + 0x2e40:
    case 0x3ff00000U + 0x2f40:
        if (8 == size && 0xfffffffffc000000ULL == val) {
            return;
        }
        break;
    case 0x3ff00000U + 0x2048:
    case 0x3ff00000U + 0x2148:
    case 0x3ff00000U + 0x2248:
    case 0x3ff00000U + 0x2348:
    case 0x3ff00000U + 0x2a48:
    case 0x3ff00000U + 0x2b48:
    case 0x3ff00000U + 0x2c48:
    case 0x3ff00000U + 0x2d48:
    case 0x3ff00000U + 0x2e48:
    case 0x3ff00000U + 0x2f48:
        if (8 == size && 0xfffffffff8000000ULL == val) {
            return;
        }
        break;
    case 0x3ff00000U + 0x2050:
    case 0x3ff00000U + 0x2150:
    case 0x3ff00000U + 0x2250:
    case 0x3ff00000U + 0x2350:
    case 0x3ff00000U + 0x2a50:
    case 0x3ff00000U + 0x2b50:
    case 0x3ff00000U + 0x2c50:
    case 0x3ff00000U + 0x2d50:
    case 0x3ff00000U + 0x2e50:
    case 0x3ff00000U + 0x2f50:
        if (8 == size && 0xffffffffc0000000ULL == val) {
            return;
        }
        break;
    case 0x3ff00000U + 0x2080:
    case 0x3ff00000U + 0x2180:
    case 0x3ff00000U + 0x2280:
    case 0x3ff00000U + 0x2380:
    case 0x3ff00000U + 0x2a80:
    case 0x3ff00000U + 0x2b80:
    case 0x3ff00000U + 0x2c80:
    case 0x3ff00000U + 0x2d80:
    case 0x3ff00000U + 0x2e80:
    case 0x3ff00000U + 0x2f80:
        if (8 == size && ((0x1efdfc00008eULL == val) || (0xefdfc00008aULL == val) || (0x1efdfc00008bULL == val))) {
            return;
        }
        break;
    case 0x3ff00000U + 0x2088:
    case 0x3ff00000U + 0x2188:
    case 0x3ff00000U + 0x2288:
    case 0x3ff00000U + 0x2388:
    case 0x3ff00000U + 0x2a88:
    case 0x3ff00000U + 0x2b88:
    case 0x3ff00000U + 0x2c88:
    case 0x3ff00000U + 0x2d88:
    case 0x3ff00000U + 0x2e88:
    case 0x3ff00000U + 0x2f88:
        if (8 == size && ((0x1e001000008eULL == val) || (0xe001000008aULL == val) || (0x1e001000008bULL == val))) {
            return;
        }
        break;
    case 0x3ff00000U + 0x2090:
    case 0x3ff00000U + 0x2190:
    case 0x3ff00000U + 0x2290:
    case 0x3ff00000U + 0x2390:
    case 0x3ff00000U + 0x2a90:
    case 0x3ff00000U + 0x2b90:
    case 0x3ff00000U + 0x2c90:
    case 0x3ff00000U + 0x2d90:
    case 0x3ff00000U + 0x2e90:
    case 0x3ff00000U + 0x2f90:
        if (8 == size && ((0x1e004000008eULL == val) || (0xe004000008aULL == val) || (0x1e004000008bULL == val))) {
            return;
        }
        break;
    case 0x3ff00000U + 0x2018:
    case 0x3ff00000U + 0x2118:
    case 0x3ff00000U + 0x2218:
    case 0x3ff00000U + 0x2318:
    case 0x3ff00000U + 0x2a18:
    case 0x3ff00000U + 0x2b18:
    case 0x3ff00000U + 0x2c18:
    case 0x3ff00000U + 0x2d18:
    case 0x3ff00000U + 0x2e18:
    case 0x3ff00000U + 0x2f18:
        if (8 == size && ((0x1e000000U == val) || (0xe0000000000ULL == val))) {
            return;
        }
        break;
    case 0x3ff00000U + 0x2058:
    case 0x3ff00000U + 0x2158:
    case 0x3ff00000U + 0x2258:
    case 0x3ff00000U + 0x2358:
    case 0x3ff00000U + 0x2a58:
    case 0x3ff00000U + 0x2b58:
    case 0x3ff00000U + 0x2c58:
    case 0x3ff00000U + 0x2d58:
    case 0x3ff00000U + 0x2e58:
    case 0x3ff00000U + 0x2f58:
        if (8 == size && ((0xffffffffff000000ULL == val) || (0xffffff0000000000ULL == val))) {
            return;
        }
        break;
    case 0x3ff00000U + 0x2098:
    case 0x3ff00000U + 0x2198:
    case 0x3ff00000U + 0x2298:
    case 0x3ff00000U + 0x2398:
    case 0x3ff00000U + 0x2a98:
    case 0x3ff00000U + 0x2b98:
    case 0x3ff00000U + 0x2c98:
    case 0x3ff00000U + 0x2d98:
    case 0x3ff00000U + 0x2e98:
    case 0x3ff00000U + 0x2f98:
        if (8 == size && ((0xe000000008eULL == val) || (0x1e000000008eULL == val) || (0xe000000008aULL == val) || (0x1e000000008bULL == val))) {
            return;
        }
        break;
    case 0xafdfb000000ULL ... 0xafdfb000278ULL:
        ret = offset & 0xfff;
        if (ret == 0x44 && size == 4) {
            lsms->per_node[node_id].ht0lo_reg_link_conf_ctrl_0x44 = val;
            return;
        } else if (ret == 0x1d8 && size == 4) {
            lsms->per_node[node_id].ht0lo_reg_send_buf_debug_0x1d8 = val;
            return;
        } else if (ret == 0x1f4 && size == 4) {
            lsms->per_node[node_id].ht0lo_reg_soft_freq_conf_0x1f4 = val;
            return;
        } else if (ret == 0x6c && size == 4) {
            lsms->per_node[node_id].ht0lo_reg_rev_id_0x6c = val;
            return;
        } else if (ret == 0x64 && size == 1) {
            lsms->per_node[node_id].ht0lo_reg_err_retry_0x64 = val;
            return;
        } else if (ret == 0xd0 && size == 1) {
            stb_p((void *)(&lsms->per_node[node_id].ht0lo_reg_link_train_0xd0), val);
            return;
        } else if (ret == 0x1fc && size == 4) {
            return;
        } else if (ret == 0x3e && size == 1) {
            if ((lsms->per_node[node_id].ht0lo_reg_bridge_ctrl_0x3c & 0x4000) == 0 && (val & 0x40) != 0) {
                lsms->per_node[node_id].ht0lo_reg_link_conf_ctrl_0x44 = 0x00112020;
                lsms->per_node[node_id].ht0hi_reg_link_conf_ctrl_0x44 = 0x00112020;
                lsms->per_node[1].ht0lo_reg_link_conf_ctrl_0x44 = 0x00112020;
                lsms->per_node[1].ht0hi_reg_link_conf_ctrl_0x44 = 0x00112020;
                lsms->per_node[2].ht0lo_reg_link_conf_ctrl_0x44 = 0x00112020;
                lsms->per_node[2].ht0hi_reg_link_conf_ctrl_0x44 = 0x00112020;
                lsms->per_node[3].ht0lo_reg_link_conf_ctrl_0x44 = 0x00112020;
                lsms->per_node[3].ht0hi_reg_link_conf_ctrl_0x44 = 0x00112020;

            } else if ((lsms->per_node[node_id].ht0lo_reg_bridge_ctrl_0x3c & 0x4000) != 0 && (val & 0x40) == 0) {
                lsms->per_node[node_id].ht0lo_reg_link_conf_ctrl_0x44 = 0x00112000;
                lsms->per_node[node_id].ht0hi_reg_link_conf_ctrl_0x44 = 0x00112000;
                lsms->per_node[1].ht0lo_reg_link_conf_ctrl_0x44 = 0x00112000;
                lsms->per_node[1].ht0hi_reg_link_conf_ctrl_0x44 = 0x00112000;
                lsms->per_node[2].ht0lo_reg_link_conf_ctrl_0x44 = 0x00112000;
                lsms->per_node[2].ht0hi_reg_link_conf_ctrl_0x44 = 0x00112000;
                lsms->per_node[3].ht0lo_reg_link_conf_ctrl_0x44 = 0x00112000;
                lsms->per_node[3].ht0hi_reg_link_conf_ctrl_0x44 = 0x00112000;
            }
            stb_p((void*)(&lsms->per_node[node_id].ht0lo_reg_bridge_ctrl_0x3c) + 0x3e - 0x3c, val);
            return;
        }
        break;
    case 0xbfdfb000000ULL ... 0xbfdfb000278ULL:
        ret = offset & 0xfff;
        if (ret == 0x44 && size == 4) {
            lsms->per_node[node_id].ht0hi_reg_link_conf_ctrl_0x44 = val;
            return;
        } else if (ret == 0x1d8 && size == 4) {
            lsms->per_node[node_id].ht0hi_reg_send_buf_debug_0x1d8 = val;
            return;
        } else if (ret == 0x1f4 && size == 4) {
            lsms->per_node[node_id].ht0hi_reg_soft_freq_conf_0x1f4 = val;
            return;
        } else if (ret == 0x6c && size == 4) {
            lsms->per_node[node_id].ht0hi_reg_rev_id_0x6c = val;
            return;
        } else if (ret == 0x64 && size == 1) {
            lsms->per_node[node_id].ht0hi_reg_err_retry_0x64 = val;
            return;
        } else if (ret == 0xd0 && size == 1) {
            stb_p((void *)(&lsms->per_node[node_id].ht0hi_reg_link_train_0xd0), val);
            return;
        } else if (ret == 0x1fc && size == 4) {
            return;
        }
        break;
    case 0xefdfb000000ULL ... 0xefdfb000278ULL:
        ret = offset & 0xfff;
        if (ret == 0x1f4 && size == 4) {
            lsms->per_node[node_id].ht0lo_reg_soft_freq_conf_0x1f4 = val;
            return;
        } else if (ret == 0x3e && size == 1) {
            if ((lsms->per_node[node_id].ht1lo_reg_bridge_ctrl_0x3c & 0x4000) == 0 && (val & 0x40) != 0) {
                lsms->per_node[node_id].ht1lo_reg_link_conf_ctrl_0x44 = 0x00112020;

            } else if ((lsms->per_node[node_id].ht1lo_reg_bridge_ctrl_0x3c & 0x4000) != 0 && (val & 0x40) == 0) {
                lsms->per_node[node_id].ht1lo_reg_link_conf_ctrl_0x44 = 0x00112000;
            }
            stb_p((void*)(&lsms->per_node[node_id].ht1lo_reg_bridge_ctrl_0x3c) + 0x3e - 0x3c, val);
            return;
        }
        break;
    case 0xffdfb000000ULL ... 0xffdfb000278ULL:
        ret = offset & 0xfff;
        if (ret == 0x44 && size == 4) {
            lsms->per_node[node_id].ht1hi_reg_link_conf_ctrl_0x44 = val;
            return;
        } else if (ret == 0x1d8 && size == 4) {
            lsms->per_node[node_id].ht1hi_reg_send_buf_debug_0x1d8 = val;
            return;
        } else if (ret == 0x1c0 && size == 4) {
            lsms->per_node[node_id].ht1hi_reg_app_conf_0_0x1c0 = val;
            return;
        } else if (ret == 0x6c && size == 4) {
            lsms->per_node[node_id].ht1hi_reg_rev_id_0x6c = val;
            return;
        } else if (ret == 0x64 && size == 1) {
            lsms->per_node[node_id].ht1hi_reg_err_retry_0x64 = val;
            return;
        } else if (ret == 0xd0 && size == 1) {
            stb_p((void *)(&lsms->per_node[node_id].ht1hi_reg_link_train_0xd0), val);
            return;
        } else if (ret == 0x1fc && size == 4) {
            return;
        } else if (ret == 0x3e && size == 1) {
            if ((lsms->per_node[node_id].ht1hi_reg_bridge_ctrl_0x3c & 0x4000) == 0 && (val & 0x40) != 0) {
                lsms->per_node[node_id].ht1hi_reg_link_conf_ctrl_0x44 = 0x00112000;

            } else if ((lsms->per_node[node_id].ht1hi_reg_bridge_ctrl_0x3c & 0x4000) != 0 && (val & 0x40) == 0) {
                lsms->per_node[node_id].ht1hi_reg_link_conf_ctrl_0x44 = 0x00112000;
            }
            stb_p((void*)(&lsms->per_node[node_id].ht1hi_reg_bridge_ctrl_0x3c) + 0x3e - 0x3c, val);
            return;
        }
        break;
    case 0x4000ff00810ULL ... 0x4000ff00820ULL:
        if (2 == size) {
            return;
        }
        break;
    case 0x5000ff00810ULL ... 0x5000ff00820ULL:
        if (2 == size) {
            return;
        }
        break;
    default:
        break;
    }
    error_report("loongson_qemu_write opaue = '%p' addr = '0x%llx' val = '0x%llx' size = '%u'", opaque, addr, val, size);
    exit(1);
}

static uint64_t loongson_qemu_read(void *opaque, hwaddr addr, unsigned size)
{
    Loongson37a1kMachineState *lsms = opaque;
    uint64_t ret = 0UL;
    uint64_t node_id = (addr & 0xF00000000000ULL) >> 44;
    uint64_t offset = addr & 0xFFFFFFFFFFFULL;
    switch (offset) {
    case 0x1fe00008:
        if (size == 4) {
            return 0x2F;
        }
        break;
    case 0x1fe00122:
        if (size == 1) {
            return reg122;
        }
        break;
    case 0x1fe00124:
        if (size == 1) {
            return reg124;
        }
        break;
    case 0x1fe00180:
    case 0x3ff00180:
        return reg180;
        break;
    case 0x1fe00190:
        if (node_id == 0) {
            return 0;
        } else {
            if (4 == size) {
                return reg190 & 0xFFFFFFFFU;
            } else if (8 == size) {
                return reg190;
            }
        }
        break;
    case 0x1fe00194:
        if (node_id == 0) {
            return 0;
        } else {
            if (4 == size) {
                return reg190 >> 32;
            }
        }
        return 0;
        break;
    case 0x1fe001a0:
        return reg1a0;
        break;
    case 0x1fe001c4:
        if (4 == size) {
            return 0;
        } 
        break;
    case 0x1fe001d0:
        return reg1d0;
        break;
    case 0x1fe00420:
        if (4 == size) {
            return reg_other_func_config_0x420;
        } else if (8 == size) {
            ret = reg_other_func_config_0x424;
            ret <<= 32;
            ret |= reg_other_func_config_0x420;
            return ret;
        }
        break;
    case 0xe00100d000cULL: // ACPI PM1 Status
        if (4 == size) {
            return 0;
        }
        break;
    case 0xe00100d0028ULL: // ACPI
        if (4 == size) {
            return 0;
        }
        break;
    case 0xe00100d002CULL: // ACPI
        if (4 == size) {
            return 0;
        }
        break;
    case 0x4000ff00000ULL:
        if (1 == size) {
            return 0;
        }
        break;
    case 0x4000ff00810ULL:
        if (2 == size) {
            return 0;
        }
        break;
    case 0x5000ff00810ULL:
        if (2 == size) {
            return 0;
        }
        break;
    case 0x1fe00424:
        if (4 == size) {
            return reg_other_func_config_0x424;
        }
        break;
    case 0xafdfb000000ULL ... 0xafdfb000278ULL:
        ret = offset & 0xfff;
        if (ret == 0x44 && size == 4) {
            return lsms->per_node[node_id].ht0lo_reg_link_conf_ctrl_0x44;
        } else if (ret == 0x1d8 && size == 4) {
            return lsms->per_node[node_id].ht0lo_reg_send_buf_debug_0x1d8;
        } else if (ret == 0xd0 && size == 4) {
            return lsms->per_node[node_id].ht0lo_reg_link_train_0xd0;
        } else if (ret == 0x1fc && size == 4) {
            return 0x83308000U;
        } else if (ret == 0x68 && size == 4) {
            return 0;
        } else if (ret == 0x3c && size == 4) {
            return lsms->per_node[node_id].ht0lo_reg_bridge_ctrl_0x3c;
        } else if (ret == 0x3e && size == 1) {
            return ldub_p((void*)(&lsms->per_node[node_id].ht0lo_reg_bridge_ctrl_0x3c) + 0x3e - 0x3c);
        }
        break;
    case 0xbfdfb000000ULL ... 0xbfdfb000278ULL:
        ret = offset & 0xfff;
        if (ret == 0x44 && size == 4) {
            return lsms->per_node[node_id].ht0hi_reg_link_conf_ctrl_0x44;
        } else if (ret == 0x68 && size == 4) {
            return 0;
        }  else if (ret == 0x1d8 && size == 4) {
            return lsms->per_node[node_id].ht0hi_reg_send_buf_debug_0x1d8;
        } else if (ret == 0xd0 && size == 4) {
            return lsms->per_node[node_id].ht0hi_reg_link_train_0xd0;
        } else if (ret == 0x1fc && size == 4) {
            return 0x83308000U;
        }
        break;
    case 0xefdfb000000ULL ... 0xefdfb000278ULL:
        ret = offset & 0xfff;
        if (ret == 0x44 && size == 4) {
            return lsms->per_node[node_id].ht1lo_reg_link_conf_ctrl_0x44;
        } else if (ret == 0x3c && size == 4) {
            return lsms->per_node[node_id].ht1lo_reg_bridge_ctrl_0x3c;
        } else if (ret == 0x3e && size == 1) {
            return ldub_p((void*)(&lsms->per_node[node_id].ht1lo_reg_bridge_ctrl_0x3c) + 0x3e - 0x3c);
        }
        break;
    case 0xffdfb000000ULL ... 0xffdfb000278ULL:
        ret = offset & 0xfff;
        if (ret == 0x44 && size == 4) {
            return lsms->per_node[node_id].ht1hi_reg_link_conf_ctrl_0x44;
        } else if (ret == 0x1d8 && size == 4) {
            return lsms->per_node[node_id].ht1hi_reg_send_buf_debug_0x1d8;
        } else if (ret == 0x1c0 && size == 4) {
            return lsms->per_node[node_id].ht1hi_reg_app_conf_0_0x1c0;
        } else if (ret == 0xd0 && size == 4) {
            return lsms->per_node[node_id].ht1hi_reg_link_train_0xd0;
        } else if (ret == 0x1fc && size == 4) {
            return 0x83308000U;
        } else if (ret == 0x3c && size == 4) {
            return lsms->per_node[node_id].ht1hi_reg_bridge_ctrl_0x3c;
        } else if (ret == 0x3e && size == 1) {
            return ldub_p((void*)(&lsms->per_node[node_id].ht1hi_reg_bridge_ctrl_0x3c) + 0x3e - 0x3c);
        }
        break;
    case 0x3ff00000U + VENDOR_REG:
        ret = *((uint64_t *)"Loongson");
        return ret;
        break;
    case 0x3ff00000U + CPUNAME_REG:
        ret = *((uint64_t *)"3A4000\0");
        break;
    case 0x1fe00400:
    case 0x3ff00000U + ROUTE_CONF_REG:
        if (8 == size) {
            ret = reg_route_config_0x404;
            ret <<= 32;
            ret |= reg_route_config_0x400;
        }
        return ret;
        break;
    case 0x3ff00000U + 0x403U:
        if (1 == size) {
            return ldub_p((void *)(((uint8_t *)&reg_route_config_0x400) + 3));
        }
        break;
    case 0x3ff00000U + 0x404U:
        if (1 == size) {
            return ldub_p((void *)(((uint8_t *)&reg_route_config_0x404)));
        }
        break;
    default:
        break;
    }
    error_report("loongson_qemu_read opaue = '%p' addr = '0x%llx' size = '%u'", opaque, addr, size);
    exit(1);
    return ret;
}

static const MemoryRegionOps loongson_qemu_mem_ops = {
    .read = loongson_qemu_mem_read,
    .write = loongson_qemu_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = UINT32_MAX,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = UINT32_MAX,
    },
};

static const MemoryRegionOps loongson_qemu_ops = {
    .read = loongson_qemu_read,
    .write = loongson_qemu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = UINT32_MAX,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = UINT32_MAX,
    },
};

static PCIBus *loongson3_irq_init(Loongson37a1kMachineState *lsms)
{
    MachineState *machine = MACHINE(lsms);
    BusState * bus;
    DeviceState *ipi, *extioi, *pch_pic, *pch_msi, *cpudev;
    DeviceState *gpio, *i2c, *spi, *pciehost;
    SysBusDevice *d;
    PCIBus *pci_bus = NULL;
    MIPSCPU *mips_cpu;
    uint32_t i, cpu_id, pin, core_id, node_id;
    hwaddr dev_addr;

    const uint32_t cpu_num = machine->smp.cpus;
    const uint32_t node_num = (machine->smp.cpus +
                   LOONGSON3_CORES_PER_NODE - 1) /
                   LOONGSON3_CORES_PER_NODE;
    ipi = qdev_new(TYPE_LOONGSON_IPI);
    d = SYS_BUS_DEVICE(ipi);
    sysbus_realize_and_unref(d, &error_fatal);
    for (cpu_id = 0; cpu_id < cpu_num; cpu_id++) {
        cpudev = DEVICE(qemu_get_cpu(cpu_id));
        mips_cpu = MIPS_CPU(cpudev);
        node_id = cpu_id / LOONGSON3_CORES_PER_NODE;
        core_id = cpu_id % LOONGSON3_CORES_PER_NODE;
        dev_addr = node_id * 0x100000000000ULL + 0x3ff00000ULL +
                   SMP_IPI_MAILBOX + core_id * CORE_IPI_LEN;
        memory_region_add_subregion(&lsms->system_iocsr, dev_addr,
                                    sysbus_mmio_get_region(d, cpu_id));
        qdev_connect_gpio_out(ipi, cpu_id, mips_cpu->env.irq[6]);
    }

    extioi = qdev_new(TYPE_LOONGSON_EXTIOI);
    d = SYS_BUS_DEVICE(extioi);
    sysbus_realize_and_unref(d, &error_fatal);
    memory_region_add_subregion(&lsms->system_iocsr, APIC_BASE,
                                    sysbus_mmio_get_region(d, 0));
    /*
     * connect ext irq to the cpu irq
     * cpu_pin[5:2] <= intc_pin[3:0]
     */
    for (cpu_id = 0; cpu_id < cpu_num; cpu_id++) {
        cpudev = DEVICE(qemu_get_cpu(cpu_id));
        mips_cpu = MIPS_CPU(cpudev);
        for (pin = 0; pin < LS3A_INTC_IP; pin++) {
            qdev_connect_gpio_out(extioi, (cpu_id * LS3A_INTC_IP + pin),
                                  mips_cpu->env.irq[pin + 2]);
        }
    }

    for (node_id = 0; node_id < node_num; node_id++) {
        gpio = qdev_new(TYPE_LOONGSON_GPIO);
        d = SYS_BUS_DEVICE(gpio);
        sysbus_realize_and_unref(d, &error_fatal);
        dev_addr = node_id * 0x100000000000ULL + LS3A_GPIO_REG_BASE;
        memory_region_add_subregion(&lsms->system_iocsr, dev_addr,
                                    sysbus_mmio_get_region(d, 0));
    }

    spi = qdev_new(TYPE_LOONGSON_SPI);
    d = SYS_BUS_DEVICE(spi);
    sysbus_realize_and_unref(d, &error_fatal);
    memory_region_add_subregion(&lsms->system_iocsr, LS3A_SPI_REG_BASE,
                                sysbus_mmio_get_region(d, 0));
    pch_pic = qdev_new(TYPE_LOONGSON_PCH_PIC);
    d = SYS_BUS_DEVICE(pch_pic);
    sysbus_realize_and_unref(d, &error_fatal);
    memory_region_add_subregion(&lsms->system_iocsr, LS7A1K_IOAPIC_REG_BASE, sysbus_mmio_get_region(d, 0));
    {
        static uint8_t eeprom_spd[] = {
			0x23, 0x10, 0x0c, 0x02, 0x84, 0x19, 0x00, 0x08, 0x00, 0x60, 0x00, 0x03, 0x01, 0x03, 0x00, 0x00,
			0x00, 0x00, 0x08, 0x0c, 0xf4, 0x1b, 0x00, 0x00, 0x6c, 0x6c, 0x6c, 0x11, 0x00, 0x74, 0xf0, 0x0a,
			0x20, 0x08, 0x70, 0x03, 0x00, 0xa8, 0x1e, 0x2b, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x36,
			0x16, 0x36, 0x16, 0x36, 0x00, 0x00, 0x16, 0x36, 0x16, 0x36, 0x16, 0x36, 0x16, 0x36, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xec, 0xb5, 0xce, 0x00, 0x00, 0x00, 0x00, 0xc2, 0x48, 0xb1,
			0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xad, 0xb2,
        };
        uint8_t *pspd = eeprom_spd;
        i2c = sysbus_create_simple("loongson_i2c", LS3A_I2C0_BASE, qdev_get_gpio_in(pch_pic, LS3A_I2C0_IRQ));
        bus = qdev_get_child_bus(i2c, "i2c");
        //i2c_slave_create_simple(I2C_BUS(bus), "ds1338", 0x68);
        // pspd = spd_data_generate(DDR2, 2 * GiB);
        smbus_eeprom_init_one(I2C_BUS(bus), 0x50, pspd);
        i2c = sysbus_create_simple("loongson_i2c", LS3A_I2C1_BASE, qdev_get_gpio_in(pch_pic, LS3A_I2C1_IRQ));
        bus = qdev_get_child_bus(i2c, "i2c");
        //i2c_slave_create_simple(I2C_BUS(bus), "ds1338", 0x68);
    }
    if (serial_hd(0)) {
        serial_mm_init(&lsms->system_iocsr, LS3A_UART0_BASE, 0,qdev_get_gpio_in(pch_pic, LS3A_UART0_IRQ),115200,serial_hd(0), DEVICE_LITTLE_ENDIAN);
    }
    if (serial_hd(1)) {
        serial_mm_init(&lsms->system_iocsr, LS3A_UART1_BASE, 0,qdev_get_gpio_in(pch_pic, LS3A_UART1_IRQ),115200,serial_hd(1), DEVICE_LITTLE_ENDIAN);
    }

    /* Connect 64 pch_pic irqs to extioi */
    for (i = 0; i < PCH_PIC_IRQ_NUM; i++) {
        sysbus_connect_irq(d, i, qdev_get_gpio_in(extioi, i));
    }

    pch_msi = qdev_new(TYPE_LOONGSON_PCH_MSI);
    d = SYS_BUS_DEVICE(pch_msi);
    sysbus_realize_and_unref(d, &error_fatal);
    memory_region_add_subregion(&lsms->system_iocsr, LS7A1K_PCH_MSI_ADDR_LOW, sysbus_mmio_get_region(d, 0));
    for (i = 0; i < PCH_MSI_IRQ_NUM; i++) {
        /* Connect 192 pch_msi irqs to extioi */
        sysbus_connect_irq(d, i,
                           qdev_get_gpio_in(extioi, i + PCH_MSI_IRQ_START));
    }

    pciehost = qdev_new(TYPE_LS7A1K_PCIE_HOST_BRIDGE);
    d = SYS_BUS_DEVICE(pciehost);
    sysbus_realize_and_unref(d, &error_fatal);
    pci_bus = PCI_HOST_BRIDGE(pciehost)->bus;
    pci_create_simple_multifunction(pci_bus, PCI_DEVFN(0, 0), false, TYPE_LS7A1K_PCI_DEVICE);
    //pci_create_simple_multifunction(pci_bus, PCI_DEVFN(1, 0), false, TYPE_LS7A1K_PCI_DEVICE);
    
    memory_region_add_subregion(&lsms->system_iocsr, LS7A1K_PCI_MEM_BASE, sysbus_mmio_get_region(d, 0));
    memory_region_add_subregion(&lsms->system_iocsr, LS7A1K_PCI_IO_BASE, sysbus_mmio_get_region(d, 1));
    memory_region_add_subregion(&lsms->system_iocsr, LS7A1K_PCI_CFG_BASE, sysbus_mmio_get_region(d, 2));
    
    memory_region_init_alias(&lsms->pci_mem_low_alias, OBJECT(d), "pci_mem_low_alias", sysbus_mmio_get_region(d, 0), 0, sysbus_mmio_get_region(d, 0)->size);
    memory_region_add_subregion(&lsms->system_iocsr, 0xe0040000000ULL, &lsms->pci_mem_low_alias);
    memory_region_init_alias(&lsms->pci_io_alias, OBJECT(d), "pci_mem_low_alias", sysbus_mmio_get_region(d, 1), 0, sysbus_mmio_get_region(d, 1)->size);
    memory_region_add_subregion(&lsms->system_iocsr, 0xefdfc000000ULL, &lsms->pci_io_alias);
    memory_region_init_alias(&lsms->pci_cfg_alias, OBJECT(d), "pci_mem_low_alias", sysbus_mmio_get_region(d, 2), 0, sysbus_mmio_get_region(d, 2)->size);
    memory_region_add_subregion(&lsms->system_iocsr, 0xefdfe000000ULL, &lsms->pci_cfg_alias);
    for (i = 0; i < LS7A1K_PCI_IRQS; i++) {
        qdev_connect_gpio_out(pciehost, i,
                              qdev_get_gpio_in(pch_pic, i + LS7A1K_DEVICE_IRQS));
    }

    return pci_bus;
}

static void main_cpu_reset(void *opaque)
{
    MIPSCPU *cpu = opaque;
    //CPUMIPSState *env = &cpu->env;
    cpu_reset(CPU(cpu));
    /* Loongson-3 reset stuff */
    //env->CP0_Status &= ~((1 << CP0St_BEV) | (1 << CP0St_ERL));

}

static void mips_loongson3_7a1k_init(MachineState *machine)
{
    MIPSCPU *cpu;
    Clock *cpuclk;
    CPUMIPSState *env;
    ram_addr_t offset = 0;
    uint64_t lowram_size = 256 * MiB;
    uint64_t highram_size = machine->ram_size - lowram_size;
    Loongson37a1kMachineState *lsms = LOONGSON3_7A1K_MACHINE(machine);
    int i, node_id;
    PCIBus *pci_bus;
    char *filename;
    int bios_size;

    if (!machine->cpu_type) {
        machine->cpu_type = MIPS_CPU_TYPE_NAME(LOONGSON3_CPU_NAME);
    }
    if (strcmp(machine->cpu_type, MIPS_CPU_TYPE_NAME(LOONGSON3_CPU_NAME))) {
        error_report("Loongson/TCG needs cpu type %s", LOONGSON3_CPU_NAME);
        exit(1);
    }

    if (machine->smp.cpus != LOONGSON3_MAX_VCPUS) {
        error_report("Loongson-3 machine needs at least 16 cpus");
        exit(1);        
    }
    if (machine->ram_size < DEFAULT_RAM_SIZE) {
        error_report("Loongson-3 machine needs at least 2GiB memory");
        exit(1);
    }

    memory_region_init_io(&lsms->system_iocsr, OBJECT(lsms), &loongson_qemu_ops,
                          lsms, "iocsr", UINT64_MAX);
    memory_region_init_io(&lsms->system_mem_conf, NULL, &loongson_qemu_mem_ops,
                          lsms, "mem_conf", 256 * MiB);
    address_space_init(&lsms->address_space_iocsr,
                       &lsms->system_iocsr, "IOCSR");
    memory_region_add_subregion(get_system_memory(), 0, &lsms->system_iocsr);
    memory_region_init_alias(&lsms->lowmem, NULL, "loongson3.lowmem",
                           machine->ram, offset, lowram_size);
    offset += lowram_size;
    memory_region_init_alias(&lsms->highmem, NULL, "loongson3.highmem",
                           machine->ram, offset, highram_size);
    offset += highram_size;
    memory_region_init_alias(&lsms->fwmem, NULL, "loongson3.spi.boot",
                           machine->ram, LA_BIOS_BASE, 1 * MiB);
    memory_region_add_subregion(&lsms->system_iocsr,
                      0, &lsms->lowmem);
    memory_region_add_subregion(&lsms->system_iocsr,
                      0x90000000, &lsms->highmem);
    memory_region_add_subregion(&lsms->system_iocsr,
                      LA_BIOS_BASE, &lsms->fwmem);
    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(cpuclk, get_cpu_freq_hz());
    for (i = 0; i < machine->smp.cpus; i++) {
        /* init CPUs */
        cpu = mips_cpu_create_with_clock(machine->cpu_type, cpuclk);
        env = &MIPS_CPU(cpu)->env;
        printf("Create cpu env at %p\n", env);
        cpu_mips_irq_init_cpu(cpu);
        cpu_mips_clock_init(cpu);
        qemu_register_reset(main_cpu_reset, cpu);
    }

    //env = &MIPS_CPU(first_cpu)->env;
    //memory_region_init_alias(&lsms->lowmem, NULL, "loongson3.lowram",
    //                         machine->ram, 0, 256 * MiB);
    //memory_region_add_subregion(address_space_mem, offset, &lsms->lowmem);
    //offset = 0;
    //highram_size = machine->ram_size;
    //memory_region_init_alias(&lsms->highmem, NULL, "loongson3.highmem",
    //                         machine->ram, offset, highram_size);
    //offset = 0x80000000;
    //memory_region_add_subregion(address_space_mem, offset, &lsms->highmem);
    //offset += highram_size;
    //memory_region_init_ram(machine->ram, NULL, "loongson3.ram", machine->ram_size, &error_fatal);

    //memory_region_set_readonly(&lsms->fwmem, true);
   // memory_region_add_subregion(machine->ram, LA_BIOS_BASE, &lsms->fwmem);
    //memory_region_add_subregion(&lsms->system_iocsr, LA_BIOS_BASE, &lsms->fwmem);
    /* load the firmware image. */
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS,
                              machine->firmware ?: LOONGSON3_7A1K_FWNAME);
    if (filename) {
        bios_size = load_image_targphys_as(filename, 0x9fc00000, LA_BIOS_SIZE, &lsms->address_space_iocsr);
        g_free(filename);
    } else {
        bios_size = -1;
    }

    if ((bios_size < 0 || bios_size > LA_BIOS_SIZE) && !qtest_enabled()) {
        error_report("Could not load Loongson3 firmware '%s'", machine->firmware);
        exit(1);
    }

    pci_bus = loongson3_irq_init(lsms);
    printf("Create pci bus at %p\n", pci_bus);
    for (node_id = 0; node_id < 4; ++node_id) {
        Ls3APerNodeState *lspns = &lsms->per_node[node_id];
        lspns->ht0lo_reg_send_buf_debug_0x1d8 = 0x0;
        lspns->ht0hi_reg_send_buf_debug_0x1d8 = 0x0;
        lspns->ht1hi_reg_send_buf_debug_0x1d8 = 0x0;
        
        lspns->ht0lo_reg_link_conf_ctrl_0x44 = 0x00112000;
        lspns->ht0hi_reg_link_conf_ctrl_0x44 = 0x00112000;
        lspns->ht1lo_reg_link_conf_ctrl_0x44 = 0x00112000;
        lspns->ht1hi_reg_link_conf_ctrl_0x44 = 0x00112000;

        lspns->ht0lo_reg_err_retry_0x64 = 0;
        lspns->ht0hi_reg_err_retry_0x64 = 0;
        lspns->ht1hi_reg_err_retry_0x64 = 0;

        lspns->ht1hi_reg_app_conf_0_0x1c0 = 0x7004321;
        
        lspns->ht0lo_reg_soft_freq_conf_0x1f4 = 0;
        lspns->ht0hi_reg_soft_freq_conf_0x1f4 = 0;
        lspns->ht1lo_reg_soft_freq_conf_0x1f4 = 0;

        lspns->ht0lo_reg_rev_id_0x6c = 0x00200000;
        lspns->ht0hi_reg_rev_id_0x6c = 0x00200000;
        lspns->ht1hi_reg_rev_id_0x6c = 0x00200000;

        lspns->ht0lo_reg_link_train_0xd0 = 0x70;
        lspns->ht0hi_reg_link_train_0xd0 = 0x70;
        lspns->ht1hi_reg_link_train_0xd0 = 0x70;

        lspns->ht0lo_reg_bridge_ctrl_0x3c = 0;
        lspns->ht1lo_reg_bridge_ctrl_0x3c = 0;
        lspns->ht1hi_reg_bridge_ctrl_0x3c = 0;
    }

    /*
     * There are some invalid guest memory access.
     * Create some unimplemented devices to emulate this.
     */
    //create_unimplemented_device("ls7a1k-lpc", 0x10002000, 0x14);
    //create_unimplemented_device("pci-dma-cfg", 0x1001041c, 0x4);
    //create_unimplemented_device("node-bridge", 0xEFDFB000274, 0x4);
    //create_unimplemented_device("ls7a1k-lionlpc", 0x1fe01400, 0x38);
    //create_unimplemented_device("ls7a1k-node0", 0x0EFDFB000274, 0x4);

    /* VGA setup. Don't bother loading the bios. */
    //pci_vga_init(pci_bus);
    //if (defaults_enabled()) {
    //    pci_create_simple(pci_bus, -1, "pci-ohci");
    //    usb_create_simple(usb_bus_find(-1), "usb-kbd");
    //    usb_create_simple(usb_bus_find(-1), "usb-tablet");
    //}

    //LOONGSON_SIMPLE_MMIO_OPS(FEATURE_REG, "loongson3_feature", 0x8);
    //LOONGSON_SIMPLE_MMIO_OPS(VENDOR_REG, "loongson3_vendor", 0x8);
    //LOONGSON_SIMPLE_MMIO_OPS(CPUNAME_REG, "loongson3_cpuname", 0x8);
}

static void loongson3_7a1k_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Loongson 3 with 7A1000 Virtualization Platform";
    mc->init = mips_loongson3_7a1k_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME(LOONGSON3_CPU_NAME);
    mc->default_cpus = LOONGSON3_MAX_VCPUS;
    mc->max_cpus = LOONGSON3_MAX_VCPUS;
    mc->min_cpus = LOONGSON3_MAX_VCPUS;
    mc->default_ram_id = "loongson3.ram";
    mc->default_ram_size = DEFAULT_RAM_SIZE;
    mc->default_kernel_irqchip_split = false;
    mc->kvm_type = mips_kvm_type;
    mc->minimum_page_bits = 14;
}

static const TypeInfo loongson3_7a1k_machine_types[] = {
    {
        .name           = TYPE_LOONGSON3_7A1K_MACHINE,
        .parent         = TYPE_MACHINE,
        .instance_size  = sizeof(Loongson37a1kMachineState),
        .class_init     = loongson3_7a1k_machine_class_init,
    }
};

DEFINE_TYPES(loongson3_7a1k_machine_types)
