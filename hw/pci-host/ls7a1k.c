/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * QEMU Loongson 7A1000 Bridge Emulation
 *
 * Copyright (C) 2021 Loongson Technology Corporation Limited
 * Copyright (c) 2021 Jintao Yin <jintao.yin@i-soft.com.cn>
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/pci/pci.h"
#include "hw/pci/pcie_host.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_bus.h"
#include "sysemu/reset.h"
#include "hw/pci-host/ls7a1k.h"
#include "migration/vmstate.h"
#include "qemu/error-report.h"

static const VMStateDescription vmstate_ls7a1k_pci = {
    .name = TYPE_LS7A1K_PCI_DEVICE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, Ls7a1kPCIState),
        VMSTATE_END_OF_LIST()
    }
};

static PCIINTxRoute ls7a1k_route_intx_pin_to_irq(void *opaque, int pin)
{
    PCIINTxRoute route;
    route.irq = pin;
    route.mode = PCI_INTX_ENABLED;
    return route;
}

static int ls7a1k_pci_map_irq(PCIDevice *d, int irq_num)
{
    PCIBus *bus;
    int offset, irq;

    bus = pci_get_bus(d);
    if (bus->parent_dev) {
        irq = pci_swizzle_map_irq_fn(d, irq_num);
        return irq;
    }

    /* pci device start from irq 80 */
    offset = PCH_PIC_IRQ_OFFSET + LS7A1K_DEVICE_IRQS;
    irq = offset + ((PCI_SLOT(d->devfn) * 4 + irq_num)) % LS7A1K_PCI_IRQS;

    return irq;
}

static void ls7a1k_pci_set_irq(void *opaque, int irq_num, int level)
{
    Ls7a1kPCIEHost *pcie_host = opaque;
    int offset = PCH_PIC_IRQ_OFFSET + LS7A1K_DEVICE_IRQS;

    qemu_set_irq(pcie_host->irqs[irq_num - offset], level);
}

static void ls7a1k_pci_config_write(void *opaque, hwaddr addr,
                                    uint64_t val, unsigned size)
{
    error_report("ls7a1k_pci_config_write opaue = '%p' addr = '0x%llx' val = '0x%llx' size = '%u'", opaque, addr, val, size);
    //exit(1);
    pci_data_write(opaque, addr, val, size);
}

static uint64_t ls7a1k_pci_config_read(void *opaque,
                                       hwaddr addr, unsigned size)
{
    error_report("ls7a1k_pci_config_read opaue = '%p' addr = '0x%llx' size = '%u'", opaque, addr, size);
    //exit(1);
    return pci_data_read(opaque, addr, size);
}

static const MemoryRegionOps ls7a1k_pci_config_ops = {
    .read = ls7a1k_pci_config_read,
    .write = ls7a1k_pci_config_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void ls7a1k_pcie_host_realize(DeviceState *dev, Error **errp)
{
    Ls7a1kPCIEHost *pcie_host = LS7A1K_PCIE_HOST_BRIDGE(dev);
    PCIExpressHost *e = PCIE_HOST_BRIDGE(dev);
    PCIHostState *phb = PCI_HOST_BRIDGE(e);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    memory_region_init(&pcie_host->pci_mem, OBJECT(dev), "ls7a1k-pci-mem",
                             LS7A1K_PCI_MEM_SIZE);
    sysbus_init_mmio(sbd, &pcie_host->pci_mem);
    memory_region_init(&pcie_host->pci_io, OBJECT(dev), "ls7a1k-pci-io",
                             LS7A1K_PCI_IO_SIZE);
    sysbus_init_mmio(sbd, &pcie_host->pci_io);
    phb->bus = pci_register_root_bus(dev, "pcie.0", ls7a1k_pci_set_irq,
                                     ls7a1k_pci_map_irq, pcie_host,
                                     &pcie_host->pci_mem, &pcie_host->pci_io,
                                     PCI_DEVFN(0, 0), 128, TYPE_PCIE_BUS);

    pci_bus_set_route_irq_fn(phb->bus, ls7a1k_route_intx_pin_to_irq);
    memory_region_init_io(&pcie_host->pci_conf, OBJECT(dev),
                          &ls7a1k_pci_config_ops, phb->bus,
                          "ls7a1k_pci_conf", LS7A1K_PCI_CFG_SIZE);
    sysbus_init_mmio(sbd, &pcie_host->pci_conf);
}

static void ls7a1k_pci_reset(DeviceState *qdev)
{
    uint32_t wmask = 0U;
    PCIDevice *dev = PCI_DEVICE(qdev);
    pci_set_word(dev->config + PCI_STATUS, 0x0010);
    pci_set_word(dev->wmask + PCI_STATUS, wmask & 0xffff);
    pci_set_word(dev->cmask + PCI_STATUS, 0xffff);
    pci_set_byte(dev->config + PCI_HEADER_TYPE, 0x80);
    pci_set_byte(dev->wmask + PCI_HEADER_TYPE, wmask & 0xff);
    pci_set_byte(dev->cmask + PCI_HEADER_TYPE, 0xff);
    pci_set_word(dev->config + PCI_SUBSYSTEM_VENDOR_ID, PCI_VENDOR_ID_LOONGSON);
    pci_set_word(dev->wmask + PCI_SUBSYSTEM_VENDOR_ID, wmask & 0xffff);
    pci_set_word(dev->cmask + PCI_SUBSYSTEM_VENDOR_ID, 0xffff);
    pci_set_word(dev->config + PCI_SUBSYSTEM_ID, 0x7a00);
    pci_set_word(dev->wmask + PCI_SUBSYSTEM_ID, wmask & 0xffff);
    pci_set_word(dev->cmask + PCI_SUBSYSTEM_ID, 0xffff);
    pci_set_byte(dev->config + PCI_CAPABILITY_LIST, 0x40);
    pci_set_byte(dev->wmask + PCI_CAPABILITY_LIST, wmask & 0xff);
    pci_set_byte(dev->cmask + PCI_CAPABILITY_LIST, 0xff);
}

static void ls7a1k_pci_realize(PCIDevice *d, Error **errp)
{
    /* pci status */
    d->config[0x6] = 0x01;
    /* base class code */
    d->config[0xb] = 0x06;
    /* header type */
    d->config[0xe] = 0x80;
    /* capabilities pointer */
    d->config[0x34] = 0x40;
    /* link status and control register 0 */
    d->config[0x44] = 0x20;
}

static void ls7a1k_pci_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
    k->vendor_id = PCI_VENDOR_ID_LOONGSON;
    k->device_id = 0x7a00;
    k->revision = 0x00;
    k->class_id = PCI_CLASS_BRIDGE_HOST;
    k->realize = ls7a1k_pci_realize;
    dc->reset = ls7a1k_pci_reset;
    dc->desc = "Loongson 7A1000 Hyper Transport Bridge Controller";
    dc->vmsd = &vmstate_ls7a1k_pci;
    /*
     * PCI-facing part of the host bridge, not usable without the
     * host-facing part, which can't be device_add'ed, yet.
     */
    dc->user_creatable = false;
}

static const TypeInfo ls7a1k_pci_device_info = {
    .name          = TYPE_LS7A1K_PCI_DEVICE,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(Ls7a1kPCIState),
    .class_init    = ls7a1k_pci_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void ls7a1k_pcie_host_initfn(Object *obj)
{
    Ls7a1kPCIEHost *s = LS7A1K_PCIE_HOST_BRIDGE(obj);
    qdev_init_gpio_out(DEVICE(obj), s->irqs, LS7A1K_PCI_IRQS);
}

static const char *ls7a1k_pcie_host_root_bus_path(
                                        PCIHostState *host_bridge,
                                        PCIBus *rootbus)
{
    return "0000:00";
}

static void ls7a1k_pcie_host_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIHostBridgeClass *hc = PCI_HOST_BRIDGE_CLASS(klass);
    hc->root_bus_path = ls7a1k_pcie_host_root_bus_path;
    dc->realize = ls7a1k_pcie_host_realize;
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
    dc->fw_name = "pci";
    dc->user_creatable = false;
}

static const TypeInfo ls7a1k_pcie_host_info = {
    .name          = TYPE_LS7A1K_PCIE_HOST_BRIDGE,
    .parent        = TYPE_PCIE_HOST_BRIDGE,
    .instance_size = sizeof(Ls7a1kPCIEHost),
    .instance_init = ls7a1k_pcie_host_initfn,
    .class_init    = ls7a1k_pcie_host_class_init,
};

static void ls7a1k_register_types(void)
{
    type_register_static(&ls7a1k_pcie_host_info);
    type_register_static(&ls7a1k_pci_device_info);
}

type_init(ls7a1k_register_types)
