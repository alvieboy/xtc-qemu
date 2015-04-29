/*
 * QEMU model of XTC SPI Controller
 *
 * Copyright (C) 2015 Alvaro Lopes
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
#include "sysemu/sysemu.h"
#include "qemu/log.h"
//#include "qemu/fifo8.h"

#include "hw/ssi.h"
//#define XTC_SPI_ERR_DEBUG

#ifdef XTC_SPI_ERR_DEBUG
#define DB_PRINT(...) do { \
    fprintf(stderr,  ": %s: ", __func__); \
    fprintf(stderr, ## __VA_ARGS__); \
    } while (0);
#else
    #define DB_PRINT(...)
#endif

#define TYPE_XTC_SPI "xtc.spi"
#define XTC_SPI(obj) OBJECT_CHECK(XTCSPI, (obj), TYPE_XTC_SPI)

#define R_MAX 32

typedef struct XTCSPI {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    qemu_irq irq;
    int irqline;

    uint8_t num_cs;
    qemu_irq *cs_lines;

    SSIBus *spi;

    uint32_t regs[R_MAX];
} XTCSPI;

static void xtc_spi_update_cs(XTCSPI *s)
{
    int i;

    DB_PRINT("CS=%d\n",s->regs[2] & 1);

    for (i = 0; i < s->num_cs; ++i) {
        qemu_set_irq(s->cs_lines[i], !(~s->regs[2] & 1 << i));
    }
}

static void xtc_spi_update_irq(XTCSPI *s)
{
}

static void xtc_spi_do_reset(XTCSPI *s)
{
    memset(s->regs, 0, sizeof s->regs);
    xtc_spi_update_irq(s);
    xtc_spi_update_cs(s);
}

static void xtc_spi_reset(DeviceState *d)
{
    xtc_spi_do_reset(XTC_SPI(d));
}

static uint64_t
spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    XTCSPI *s = opaque;
    uint32_t r = 0;

    addr >>= 2;
    switch (addr) {
    case 13:
    case 9:
    case 5:
    case 1:
        return s->regs[1];
    default:

        if (addr < ARRAY_SIZE(s->regs)) {
            r = s->regs[addr];
        }
        break;
    }
    DB_PRINT("addr=" TARGET_FMT_plx " = %x\n", addr , r);
    xtc_spi_update_irq(s);
    return r;
}

static void xtc_transfer(XTCSPI*s,uint32_t size,uint32_t val)
{
    uint8_t txb[4];
    txb[3] = val>>24;
    txb[2] = val>>16;
    txb[1] = val>>8;
    txb[0] = val;
    while (size--) {
        s->regs[1] <<= 8;
        s->regs[1] |= (ssi_transfer(s->spi, txb[size])&0xff);
        DB_PRINT("Xfer: -> %02x <-> %02x\n", txb[size],s->regs[1] & 0xff);
    }
}

static void
spi_write(void *opaque, hwaddr addr,
            uint64_t val64, unsigned int size)
{
    XTCSPI *s = opaque;
    uint32_t value = val64;
    uint32_t txsize = 1;
    addr >>= 2;
    DB_PRINT("addr=" TARGET_FMT_plx " = %x\n", addr, value);

    switch (addr) {
    case 0:
        break;
    case 13:
        txsize++;
    case 9:
        txsize++;
    case 5:
        txsize++;
    case 1:
        xtc_transfer( s, txsize, value );
        break;
    case 2:
        s->regs[2]=value;
        xtc_spi_update_cs(s);
    case 4:

    default:
        if (addr < ARRAY_SIZE(s->regs)) {
            s->regs[addr] = value;
        }
        break;
    }

    xtc_spi_update_irq(s);
}

static const MemoryRegionOps spi_ops = {
    .read = spi_read,
    .write = spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static int xtc_spi_init(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    XTCSPI *s = XTC_SPI(dev);
    int i;

    DB_PRINT("\n");

    s->spi = ssi_create_bus(dev, "spi");

    sysbus_init_irq(sbd, &s->irq);
    s->cs_lines = g_new0(qemu_irq, s->num_cs);
    ssi_auto_connect_slaves(dev, s->cs_lines, s->spi);

    for (i = 0; i < s->num_cs; ++i) {
        sysbus_init_irq(sbd, &s->cs_lines[i]);
    }

    memory_region_init_io(&s->mmio, OBJECT(s), &spi_ops, s,
                          "xtc-spi", R_MAX * 4);
    sysbus_init_mmio(sbd, &s->mmio);

    s->irqline = -1;

    return 0;
}

static const VMStateDescription vmstate_xtc_spi = {
    .name = "xtc_spi",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, XTCSPI, R_MAX),
        VMSTATE_END_OF_LIST()
    }
};

static Property xtc_spi_properties[] = {
    DEFINE_PROP_UINT8("num-ss-bits", XTCSPI, num_cs, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void xtc_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = xtc_spi_init;
    dc->reset = xtc_spi_reset;
    dc->props = xtc_spi_properties;
    dc->vmsd = &vmstate_xtc_spi;
}

static const TypeInfo xtc_spi_info = {
    .name           = TYPE_XTC_SPI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(XTCSPI),
    .class_init     = xtc_spi_class_init,
};

static void xtc_spi_register_types(void)
{
    type_register_static(&xtc_spi_info);
}

type_init(xtc_spi_register_types)
