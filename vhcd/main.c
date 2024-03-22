#define pr_fmt(fmt) "[vhcd] " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb.h>

#define DEV_NAME "vhcd-pdev"

static void hcd_pdev_dummy_release(struct device *dev) {}

static struct platform_device hcd_pdev = {
    .name = DEV_NAME,
    .id = PLATFORM_DEVID_NONE,
    .dev.release = hcd_pdev_dummy_release,
};

static int hcd_probe(struct platform_device *pdev);
static int hcd_remove(struct platform_device *pdev);
static struct platform_driver hcd_driver = {
    .probe = hcd_probe,
    .remove = hcd_remove,
    .driver =
        {
            .name = DEV_NAME,
        },
};

static int hcd_probe(struct platform_device *pdev)
{
    pr_info("Probe hcd\n");
    return 0;
}

static int hcd_remove(struct platform_device *pdev)
{
    pr_info("Remove hcd\n");
    return 0;
}

static int __init hcd_pdev_init(void)
{
    int ret;

    if (usb_disabled())
        return -ENODEV;

    ret = platform_device_register(&hcd_pdev);
    if (ret < 0)
        goto err_register_hcd_dev;

    ret = platform_driver_register(&hcd_driver);
    if (ret < 0)
        goto err_register_hcd_driver;

    pr_info("Initialize hcd: %d\n", ret);
    return ret;

err_register_hcd_driver:
    platform_device_unregister(&hcd_pdev);
err_register_hcd_dev:
    return ret;
}
module_init(hcd_pdev_init);

static void __exit hcd_pdev_exit(void)
{
    platform_device_unregister(&hcd_pdev);
    platform_driver_unregister(&hcd_driver);
    pr_info("Exit hcd\n");
}
module_exit(hcd_pdev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("RinHizakura");
MODULE_DESCRIPTION("The virtual USB platform device and its driver");
