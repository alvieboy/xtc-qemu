/*
 * Model of PPro with XTC
 *
 * Copyright (c) 2015 Alvaro Lopes
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

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "net/net.h"
#include "hw/block/flash.h"
#include "sysemu/sysemu.h"
#include "hw/devices.h"
#include "hw/boards.h"
#include "sysemu/block-backend.h"
#include "hw/char/serial.h"
#include "exec/address-spaces.h"
#include "hw/ssi.h"
#include "hw/stream.h"
#include "xtc_rom.h"

#define RAM_SIZE_MB 4
#define MEMORY_BASEADDR 0x0
#define ROM_BASEADDR 0x40000000

static void machine_cpu_reset(XTCCPU *cpu)
{
    cpu->reset_address = ROM_BASEADDR;
    cpu_reset(CPU(cpu));
}


static void ppro_init(MachineState *machine)
{
    //ram_addr_t ram_size = machine->ram_size;
    MemoryRegion *address_space_mem = get_system_memory();
//    DeviceState *dev, *dma, *eth0;
//    Object *ds, *cs;
    XTCCPU *cpu;
    SysBusDevice *busdev;
//    DriveInfo *dinfo;
    //int i;
    MemoryRegion *phys_ram = g_new(MemoryRegion, 1);
    MemoryRegion *phys_ram_alias = g_new(MemoryRegion, 1);
    MemoryRegion *phys_rom = g_new(MemoryRegion, 1);
  //  qemu_irq irq[32];

    /* init CPUs */
    cpu = XTC_CPU(object_new(TYPE_XTC_CPU));

    object_property_set_bool(OBJECT(cpu), true, "realized", &error_abort);

    memory_region_init_ram(phys_ram, NULL, "ppro.ram", RAM_SIZE_MB*1024*1024, &error_abort);
    vmstate_register_ram_global(phys_ram);

    memory_region_init_alias(phys_ram_alias, NULL, "ppro.ramalias",
                             phys_ram,
                             0,
                             RAM_SIZE_MB*1024*1024);

    printf("RAM0 at 0x%08x, size 0x%08x\n", MEMORY_BASEADDR, RAM_SIZE_MB*1024*1024);
    memory_region_add_subregion(address_space_mem, MEMORY_BASEADDR, phys_ram);
    printf("RAM0 alias at 0x%08x, size 0x%08x\n", MEMORY_BASEADDR+(RAM_SIZE_MB*1024*1024), RAM_SIZE_MB*1024*1024);
    memory_region_add_subregion(address_space_mem, MEMORY_BASEADDR+(RAM_SIZE_MB*1024*1024), phys_ram_alias);

    memory_region_init_ram_ptr(phys_rom, NULL, "ppro.rom", 8192, (char*)xtc_rom);
    vmstate_register_ram_global(phys_rom);
    memory_region_add_subregion(address_space_mem, ROM_BASEADDR, phys_rom);
    memory_region_set_readonly(phys_rom, TRUE);

    sysbus_create_simple("xtc.uart", 0x90000000,
                         NULL);

    {
        SSIBus *spi;
        DeviceState *dev;

        dev = qdev_create(NULL, "xtc.spi");
        qdev_init_nofail(dev);
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, 0xA0000000);

        spi = (SSIBus *)qdev_get_child_bus(dev, "spi");

        qemu_irq cs_line;

        dev = ssi_create_slave(spi, "m25p128");
        cs_line = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
        sysbus_connect_irq(busdev, 1, cs_line);
    }

    {
        DeviceState *dev;

        dev = qdev_create(NULL, "xtc.spi");
        qdev_init_nofail(dev);
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, 0xB0000000);
        SSIBus *spi;
        spi = (SSIBus *)qdev_get_child_bus(dev, "spi");

        qemu_irq cs_line;
        dev = ssi_create_slave(spi, "ssi-sd");
        cs_line = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
        sysbus_connect_irq(busdev, 1, cs_line);

    }


    machine_cpu_reset(cpu);
}

static QEMUMachine ppro_machine = {
    .name = "ppro",
    .desc = "Papilio Pro running XTC",
    .init = ppro_init,
    .is_default = 1,
};

static void ppro_machine_init(void)
{
    qemu_register_machine(&ppro_machine);
}

machine_init(ppro_machine_init);
