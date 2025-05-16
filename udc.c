#define pr_fmt(fmt) "vusb:" fmt

#include "common.h"

static void vudc_pdev_dummy_release(struct device *dev) {}

struct platform_device vudc_pdev = {
    .name = UDC_DEV_NAME,
    .id = PLATFORM_DEVID_NONE,
    .dev.release = vudc_pdev_dummy_release,
};

static void vudc_set_speed(struct usb_gadget *_gadget,
                           enum usb_device_speed speed)
{
    INFO("udc_set_speed");
}

static const struct usb_gadget_ops vudc_gadget_ops = {
    .udc_set_speed = vudc_set_speed,
};

static int vudc_probe(struct platform_device *pdev)
{
    int ret;
    struct virt *virt = get_platdata(pdev);

    INFO("Probe vudc\n");

    memset(&virt->gadget, 0, sizeof(struct usb_gadget));

    // Simulate as an USB 2.0 device
    virt->gadget.ops = &vudc_gadget_ops;
    virt->gadget.name = UDC_DEV_NAME;
    virt->gadget.max_speed = USB_SPEED_HIGH;

    ret = usb_add_gadget_udc(&pdev->dev, &virt->gadget);

    return ret;
}

static void vudc_remove(struct platform_device *pdev)
{
    struct virt *virt = get_platdata(pdev);

    INFO("Remove vudc\n");

    usb_del_gadget_udc(&virt->gadget);
}

struct platform_driver vudc_driver = {
    .probe = vudc_probe,
    .remove_new = vudc_remove,
    .driver =
        {
            .name = UDC_DEV_NAME,
        },
};
