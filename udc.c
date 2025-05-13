#define pr_fmt(fmt) "vusb:" fmt

#include "common.h"

static void vudc_pdev_dummy_release(struct device *dev) {}

struct platform_device vudc_pdev = {
    .name = UDC_DEV_NAME,
    .id = PLATFORM_DEVID_NONE,
    .dev.release = vudc_pdev_dummy_release,
};

static int vudc_probe(struct platform_device *pdev)
{
    INFO("Probe vudc\n");

    return 0;
}

static int vudc_remove(struct platform_device *pdev)
{
    INFO("Remove vudc\n");

    return 0;
}

struct platform_driver vudc_driver = {
    .probe = vudc_probe,
    .remove = vudc_remove,
    .driver =
        {
            .name = UDC_DEV_NAME,
        },
};
