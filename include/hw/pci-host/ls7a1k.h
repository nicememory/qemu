/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * QEMU LoongArch CPU
 *
 * Copyright (c) 2021 Loongson Technology Corporation Limited
 * Copyright (c) 2021 Jintao Yin <jintao.yin@i-soft.com.cn>
 */

#ifndef HW_LS7A1K_H
#define HW_LS7A1K_H

#include "hw/pci/pci.h"
#include "hw/pci/pcie_host.h"
#include "qemu/units.h"
#include "qemu/range.h"
#include "qom/object.h"

#define LS7A1K_PCI_IO_BASE              0x18000000UL
#define LS7A1K_PCI_IO_SIZE              0x02000000UL

#define LS7A1K_PCI_CFG_BASE             0x1a000000UL
#define LS7A1K_PCI_CFG_SIZE             0x02000000UL

#define LS7A1K_PCI_MEM_BASE             0x40000000UL
#define LS7A1K_PCI_MEM_SIZE             0x40000000UL

/*
 * According to the kernel pch irq start from 64 offset
 * 0 ~ 16 irqs used for non-pci device while 16 ~ 64 irqs
 * used for pci device.
 */
#define PCH_PIC_IRQ_OFFSET              64
#define LS7A1K_DEVICE_IRQS              16
#define LS7A1K_PCI_IRQS                 48

typedef struct Ls7a1kPCIEHost Ls7a1kPCIEHost;

typedef struct Ls7a1kPCIState {
    PCIDevice parent_obj;
} Ls7a1kPCIState;

struct Ls7a1kPCIEHost {
    PCIExpressHost parent_obj;
    MemoryRegion pci_mem;
    MemoryRegion pci_conf;
    MemoryRegion pci_io;
    qemu_irq irqs[LS7A1K_PCI_IRQS];
};

#define TYPE_LS7A1K_PCIE_HOST_BRIDGE "ls7a1k_pcie_host_bridge"
OBJECT_DECLARE_SIMPLE_TYPE(Ls7a1kPCIEHost, LS7A1K_PCIE_HOST_BRIDGE)

#define TYPE_LS7A1K_PCI_DEVICE "ls7a1k_pci_device"
OBJECT_DECLARE_SIMPLE_TYPE(Ls7a1kPCIState, LS7A1K_PCI_DEVICE)

#endif /* HW_LS7A1K_H */