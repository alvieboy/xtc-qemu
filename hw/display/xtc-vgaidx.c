/*
 * XTC Indexed VGA adaptor
 *
 * Copyright (c) 2015 Alvaro Lopes
 *
 * This code is licensed under the GNU LGPL
 */

#include "hw/sysbus.h"
#include "ui/console.h"
#include "framebuffer.h"
#include "ui/pixel_ops.h"

#define TYPE_XTCVGAIDX "xtc.vgaidx"
#define XTCVGAIDX(obj) OBJECT_CHECK(XTCVGAIDXState, (obj), TYPE_XTCVGAIDX)

typedef struct XTCVGAIDXState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion *fbmem;
    QemuConsole *con;

    uint32_t base;
    int cols;
    int rows;
    int invalidate;
    uint32_t palette[256];
    //uint32_t raw_palette[128];
} XTCVGAIDXState;

static int vmstate_xtcvgaidx_post_load(void *opaque, int version_id);
int xtcvgaidx_setmemoryregion(SysBusDevice *sbd, MemoryRegion*m);

static const VMStateDescription vmstate_xtcvgaidx = {
    .name = "xtc.vgaidx",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = vmstate_xtcvgaidx_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(palette, XTCVGAIDXState, 256),
        VMSTATE_END_OF_LIST()
    }
};

static void xtcvgaidx_draw_fn_24(void *opaque, uint8_t *d, const uint8_t *src, int width, int deststep)
{
    uint32_t *palette = opaque;
    while (width > 0) {
        uint32_t pixel = palette[*src];
        *d++=(pixel>>16)&0xff;
        *d++=(pixel>>8)&0xff;
        *d++=(pixel)&0xff;
        width--;
        src++;
    }
}

static void xtcvgaidx_draw_fn_32(void *opaque, uint8_t *d, const uint8_t *src, int width, int deststep)
{
    uint32_t *palette = opaque;
    //printf("Line update, width %d, deststep %d\n", width, deststep);
    while (width > 0) {
        uint32_t pixel = palette[*src];
        *d++=(pixel>>16)&0xff;
        *d++=(pixel>>8)&0xff;
        *d++=(pixel)&0xff;
        *d++=0xff;
        width--;
        src++;
    }

}

static void xtcvgaidx_update_display(void *opaque)
{
    XTCVGAIDXState *s = (XTCVGAIDXState *)opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    drawfn fn;
    int dest_width = 0;
    int src_width = 320;
    int first;
    int last;

    //sbd = SYS_BUS_DEVICE(s);

    switch (surface_bits_per_pixel(surface)) {
    case 0:
        return;
    case 24:
        fn = xtcvgaidx_draw_fn_24;
        dest_width = 3;
        
        break;
    case 32:
        fn = xtcvgaidx_draw_fn_32;
        dest_width = 4;
        break;
    default:
        fprintf(stderr, "xtcvgaidx: Bad color depth\n");
        exit(1);
    }
    src_width = s->cols;

    dest_width *= s->cols;

    if (true) {
        if (s->fbmem && s->base) {
#if 0
            if (s->invalidate) {
                printf("xtcvgaidx: invalidate %s offset 0x%08x\n",
                       s->fbmem->name, s->base);
            }
#endif
            first = 0;
            framebuffer_update_display(surface, s->fbmem,
                                       s->base, s->cols, s->rows,
                                       src_width, dest_width, 0,
                                       s->invalidate,
                                       fn, s->palette,
                                       &first, &last);

            if (first >= 0) {
                //printf("Update first %d count %d\n", first, last-first + 1);
                dpy_gfx_update(s->con, 0, first, s->cols, last - first + 1);
                //dpy_gfx_update(s->con, 0, 0, s->cols, s->rows);
            }
        }
        s->invalidate = 0;
    }
}

static void xtcvgaidx_invalidate_display(void * opaque)
{
    XTCVGAIDXState *s = (XTCVGAIDXState *)opaque;
    s->invalidate = 1;
    qemu_console_resize(s->con, s->cols, s->rows);
}
#if 0
static void xtcvgaidx_resize(XTCVGAIDXState *s, int width, int height)
{
    if (width != s->cols || height != s->rows) {
        qemu_console_resize(s->con, width, height);
    }
    s->cols = width;
    s->rows = height;
}
#endif

static uint64_t xtcvgaidx_read(void *opaque, hwaddr offset,
                               unsigned size)
{
    //XTCVGAIDXState *s = (XTCVGAIDXState *)opaque;

    abort();
}

static void xtcvgaidx_write(void *opaque, hwaddr offset,
                        uint64_t val, unsigned size)
{
    XTCVGAIDXState *s = (XTCVGAIDXState *)opaque;
    /* For simplicity invalidate the display whenever a control register
       is written to.  */
    s->invalidate = 1;
    static int init=0;

    offset>>=2;
/*    printf("XTCVGAIDX: write address 0x%08x, data %08x\n", (uint32_t)offset, (uint32_t)val); */

    if (offset == 0x0) {
        /* Framebuffer update */
        s->base = val;
        memory_region_set_dirty( s->fbmem, val, 320*240 );
    } else {
        if (offset & 0x100) {
            unsigned paloffset = offset & 0xff;
            printf("Pallete update %d -> 0x%08x\n", paloffset, (uint32_t)val);
            s->palette[ paloffset ] = val;
        }
    }
    if (!init) {
        qemu_console_resize(s->con, s->cols, s->rows);
        init = 1;
    }
}

static const MemoryRegionOps xtcvgaidx_ops = {
    .read = xtcvgaidx_read,
    .write = xtcvgaidx_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int vmstate_xtcvgaidx_post_load(void *opaque, int version_id)
{
    XTCVGAIDXState *s = opaque;
    /* Make sure we redraw, and at the right size */
    xtcvgaidx_invalidate_display(s);
    return 0;
}

static const GraphicHwOps xtcvgaidx_gfx_ops = {
    .invalidate  = xtcvgaidx_invalidate_display,
    .gfx_update  = xtcvgaidx_update_display,
};

static int xtcvgaidx_initfn(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    XTCVGAIDXState *s = XTCVGAIDX(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &xtcvgaidx_ops, s, "xtc.vgaidx", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
//    sysbus_init_irq(sbd, &s->irq);
//    qdev_init_gpio_in(dev, pl110_mux_ctrl_set, 1);
    s->con = graphic_console_init(dev, 0, &xtcvgaidx_gfx_ops, s);
    s->base = 0;
    return 0;
}

int xtcvgaidx_setmemoryregion(SysBusDevice *sbd, MemoryRegion*m)
{
    DeviceState *dev = DEVICE(sbd);
    XTCVGAIDXState *s = XTCVGAIDX(dev);
    s->fbmem = m;
    return 0;
}


static void xtcvgaidx_init(Object *obj)
{
    XTCVGAIDXState *s = XTCVGAIDX(obj);
    //    xtcvgaidx_resize(s, 320, 240);
    s->cols = 320;
    s->rows = 240;
    s->fbmem = NULL;
}

static void xtcvgaidx_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = xtcvgaidx_initfn;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    dc->vmsd = &vmstate_xtcvgaidx;
}

static const TypeInfo xtcvgaidx_info = {
    .name          = TYPE_XTCVGAIDX,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XTCVGAIDXState),
    .instance_init = xtcvgaidx_init,
    .class_init    = xtcvgaidx_class_init,
};

static void xtcvgaidx_register_types(void)
{
    type_register_static(&xtcvgaidx_info);
}

type_init(xtcvgaidx_register_types)
