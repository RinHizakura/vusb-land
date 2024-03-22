#define pr_fmt(fmt) "[vhcd] " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb.h>

static void hcd_pdev_dummy_release(struct device *dev) {}

static struct platform_device hcd_pdev = {
    .name = "vhcd-pdev",
    .id = PLATFORM_DEVID_NONE,
    .dev.release = hcd_pdev_dummy_release,
};

static int __init hcd_pdev_init(void)
{
    int ret;

    if (usb_disabled())
        return -ENODEV;

    ret = platform_device_register(&hcd_pdev);

    pr_info("Init hcd_pdev: %d\n", ret);

    return ret;
}
module_init(hcd_pdev_init);

static void __exit hcd_pdev_exit(void)
{
    platform_device_unregister(&hcd_pdev);
}
module_exit(hcd_pdev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("RinHizakura");
MODULE_DESCRIPTION("The virtual USB platform device and its driver");
