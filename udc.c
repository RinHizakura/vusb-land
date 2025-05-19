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

static ssize_t function_show(struct device *dev,
                             struct device_attribute *attr,
                             char *buf)
{
    struct virt *virt = container_of(dev, struct virt, gadget.dev);

    return 0;
}
static DEVICE_ATTR_RO(function);

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
    virt->gadget.dev.parent = &pdev->dev;

    ret = usb_add_gadget_udc(&pdev->dev, &virt->gadget);
    if (ret < 0)
        goto err_add_gadget_udc;

    ret = device_create_file(&virt->gadget.dev, &dev_attr_function);
    if (ret < 0)
        goto err_dev;
    platform_set_drvdata(pdev, virt);

    return 0;

err_dev:
    usb_del_gadget_udc(&virt->gadget);
err_add_gadget_udc:
    return ret;
}

static void vudc_remove(struct platform_device *pdev)
{
    struct virt *virt = get_platdata(pdev);

    INFO("Remove vudc\n");

    device_remove_file(&virt->gadget.dev, &dev_attr_function);
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
