/*
 * QEMU model of XTC UART
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
#include "sysemu/char.h"
#include <sys/time.h>

#define TYPE_XTC_UART "xtc.uart"
#define XTC_UART(obj) \
    OBJECT_CHECK(XTCUART, (obj), TYPE_XTC_UART)

#define R_MAX 4

typedef struct XTCUART {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    CharDriverState *chr;
    qemu_irq irq;

    uint8_t rx_fifo[2048];
    unsigned int rx_fifo_pos;
    unsigned int rx_fifo_len;

    uint32_t regs[R_MAX];
} XTCUART;

#define R_STATUS 1

static void uart_update_irq(XTCUART *s)
{
/*    unsigned int irq;

    if (s->rx_fifo_len)
        s->regs[R_STATUS] |= STATUS_IE;

    irq = (s->regs[R_STATUS] & STATUS_IE) && (s->regs[R_CTRL] & CONTROL_IE);
    qemu_set_irq(s->irq, irq);
    */
}

static void uart_update_status(XTCUART *s)
{
    uint32_t r = 0;

    r |= (!!s->rx_fifo_len);
    s->regs[R_STATUS] = r;
}

static void xtc_uart_reset(DeviceState *dev)
{
    uart_update_status(XTC_UART(dev));
}

static uint64_t gettime(void)
{
    static uint64_t start = 0;
    struct timeval tv;
    gettimeofday(&tv,NULL);
    uint64_t r = (tv.tv_sec * 1000) + (tv.tv_usec/1000);
    if (start==0) {
        start=r;
        return 0;
    }
    return r - start;
}

static uint64_t
uart_read(void *opaque, hwaddr addr, unsigned int size)
{
    XTCUART *s = opaque;
    uint32_t r = 0;
    addr >>= 2;
    switch (addr)
    {
    case 0:
        r = s->rx_fifo[(s->rx_fifo_pos - s->rx_fifo_len) & 7];
        if (s->rx_fifo_len)
            s->rx_fifo_len--;
        uart_update_status(s);
        uart_update_irq(s);
        qemu_chr_accept_input(s->chr);
        break;
    case 2:
    case 3:
        r =  gettime();

        break;
    default:
        if (addr < ARRAY_SIZE(s->regs))
            r = s->regs[addr];
        qemu_log("%s addr=%lx v=%x\n", __func__, addr, r);
        break;
    }
    return r;
}

static void
uart_write(void *opaque, hwaddr addr,
           uint64_t val64, unsigned int size)
{
    XTCUART *s = opaque;
    uint32_t value = val64;
    unsigned char ch = value;

    addr >>= 2;
    switch (addr)
    {
    case 0:
            if (s->chr)
                qemu_chr_fe_write(s->chr, &ch, 1);
            break;

        default:
            //printf("%s addr=%x v=%x\n", __func__, addr, value));
            if (addr < ARRAY_SIZE(s->regs))
                s->regs[addr] = value;
            break;
    }
    uart_update_status(s);
    uart_update_irq(s);
}

static const MemoryRegionOps uart_ops = {
    .read = uart_read,
    .write = uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4
    }
};

static void uart_rx(void *opaque, const uint8_t *buf, int size)
{
    XTCUART *s = opaque;

    /* Got a byte.  */
    if (s->rx_fifo_len >= 8) {
        printf("WARNING: UART dropped char.\n");
        return;
    }
    s->rx_fifo[s->rx_fifo_pos] = *buf;
    s->rx_fifo_pos++;
    s->rx_fifo_pos &= 0xff; // TODO: fix this please.
    s->rx_fifo_len++;

    uart_update_status(s);
    uart_update_irq(s);
}

static int uart_can_rx(void *opaque)
{
    XTCUART *s = opaque;

    return s->rx_fifo_len < sizeof(s->rx_fifo);
}

static void uart_event(void *opaque, int event)
{

}

static void xtc_uart_realize(DeviceState *dev, Error **errp)
{
    XTCUART *s = XTC_UART(dev);

    /* FIXME use a qdev chardev prop instead of qemu_char_get_next_serial() */
    s->chr = qemu_char_get_next_serial();
    if (s->chr)
        qemu_chr_add_handlers(s->chr, uart_can_rx, uart_rx, uart_event, s);
}

static void xtc_uart_init(Object *obj)
{
    XTCUART *s = XTC_UART(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    memory_region_init_io(&s->mmio, obj, &uart_ops, s,
                          "xtc.uart", R_MAX * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void xtc_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = xtc_uart_reset;
    dc->realize = xtc_uart_realize;
    /* Reason: realize() method uses qemu_char_get_next_serial() */
    dc->cannot_instantiate_with_device_add_yet = true;
}

static const TypeInfo xtc_uart_info = {
    .name          = TYPE_XTC_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XTCUART),
    .instance_init = xtc_uart_init,
    .class_init    = xtc_uart_class_init,
};

static void xtc_uart_register_types(void)
{
    type_register_static(&xtc_uart_info);
}

type_init(xtc_uart_register_types)
