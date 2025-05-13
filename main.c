#define pr_fmt(fmt) "vusb:" fmt

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include "common.h"

extern struct platform_device vhcd_pdev;
extern struct platform_driver vhcd_driver;
extern struct platform_device vudc_pdev;
extern struct platform_driver vudc_driver;

static int __init vusb_init(void)
{
    struct virt *virt = NULL;
    int ret;

    if (usb_disabled())
        return -ENODEV;

    ret = platform_device_register(&vhcd_pdev);
    if (ret < 0)
        goto err_register_hcd_dev;

    virt = kzalloc(sizeof(struct virt), GFP_KERNEL);
    if (!virt) {
        ret = -ENOMEM;
        goto err_alloc_pdata;
    }
    ret = platform_device_add_data(&vhcd_pdev, &virt, sizeof(void *));
    if (ret) {
        goto err_add_pdata;
    }

    ret = platform_driver_register(&vhcd_driver);
    if (ret < 0)
        goto err_register_hcd_driver;

    return ret;

err_register_hcd_driver:
err_add_pdata:
    kfree(virt);
err_alloc_pdata:
    platform_device_unregister(&vhcd_pdev);

err_register_hcd_dev:
    return ret;
}
module_init(vusb_init);

static void __exit vusb_exit(void)
{
    struct virt *virt = get_platdata(&vhcd_pdev);
    platform_device_unregister(&vhcd_pdev);
    kfree(virt);
    platform_driver_unregister(&vhcd_driver);
}
module_exit(vusb_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("RinHizakura");
MODULE_DESCRIPTION("The virtual USB platform");
