#ifndef COMMON_H
#define COMMON_H

#include <linux/platform_device.h>
#include <linux/usb/gadget.h>

#ifdef DEBUG
#define INFO(...) pr_info(__VA_ARGS__)
#else
#define INFO(...)
#endif

static inline struct virt *get_platdata(struct platform_device *pdev)
{
    return *((void **) dev_get_platdata(&pdev->dev));
}

/* This is the structure which is used to emulate the state shared betwen
 * HCD and UDC. It will be referenced by
 * platform_device->dev.platform_data, which is registered by
 * platform_device_add_data(). */
struct virt {
    /* For UDC side */
    bool pullup;
    struct usb_gadget gadget;

    /* For HCD side */
    struct vhcd_priv *hs_hcd_priv;
};

#endif
