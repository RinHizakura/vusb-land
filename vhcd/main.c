#define pr_fmt(fmt) "vhcd:" fmt

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#define DEV_NAME "vhcd"

enum roothub_state {
    RH_RESET,
    RH_SUSPENDED,
    RH_RUNNING
};

/* This is the sturcture which will be referenced by usb_hcd->hcd_priv
 * as private data */
struct vhcd_hcd_priv {
    spinlock_t lock;
    struct vhcd_data *data;

    enum roothub_state rh_state;
    struct usb_port_status port_status;
    int active;
};

static inline struct usb_hcd *priv_to_hcd(struct vhcd_hcd_priv *hcd_priv)
{
    return container_of((void *) hcd_priv, struct usb_hcd, hcd_priv);
}

static inline struct vhcd_hcd_priv *hcd_to_priv(struct usb_hcd *hcd)
{
    return (struct vhcd_hcd_priv *) (hcd->hcd_priv);
}

static inline void hcd_priv_init(struct vhcd_hcd_priv *priv, struct vhcd_data *data)
{
    spin_lock_init(&priv->lock);
    priv->data = data;
    priv->rh_state = RH_RESET;
    priv->port_status = (struct usb_port_status) {
        .wPortStatus = 0,
        .wPortChange = 0,
    };
    priv->active = 0;
}

static void set_link_state(struct vhcd_hcd_priv *priv)
    __must_hold(&priv->lock)
{
    /* Record status and active, so we can compare the changes for them. */
    struct usb_port_status port_status_old = priv->port_status;
    int active_old = priv->active;

    if (priv->port_status.wPortStatus & USB_PORT_STAT_POWER) {
        /* A device is present on this port */
        priv->port_status.wPortStatus |= USB_PORT_STAT_CONNECTION;

        /* The port status change from disconnect to connect. */
        if (!(port_status_old.wPortStatus & USB_PORT_STAT_CONNECTION))
            priv->port_status.wPortChange |= USB_PORT_STAT_C_CONNECTION;

        priv->active = 1;
    }
}

/* This is the structure which will be referenced by
 * platform_device->dev.platform_data, which is registered by
 * platform_device_add_data(). */
struct vhcd_data {
    struct vhcd_hcd_priv *hs_hcd_priv;
};

static inline struct vhcd_data *get_platdata(struct platform_device *pdev)
{
    return *((void **) dev_get_platdata(&pdev->dev));
}

static void vhcd_pdev_dummy_release(struct device *dev) {}

static struct platform_device vhcd_pdev = {
    .name = DEV_NAME,
    .id = PLATFORM_DEVID_NONE,
    .dev.release = vhcd_pdev_dummy_release,
};

static int vhcd_setup(struct usb_hcd *hcd)
{
    struct vhcd_data *data;

    pr_info("Setup hcd\n");

    data = *((void **) dev_get_platdata(hcd->self.controller));
    hcd->self.sg_tablesize = ~0;

    data->hs_hcd_priv = (struct vhcd_hcd_priv *) (hcd->hcd_priv);
    hcd_priv_init(data->hs_hcd_priv, data);

    hcd->speed = HCD_USB2;
    hcd->self.root_hub->speed = USB_SPEED_HIGH;

    return 0;
}

static int vhcd_start(struct usb_hcd *hcd)
{
    struct vhcd_hcd_priv *priv = hcd_to_priv(hcd);

    pr_info("Start hcd\n");

    priv->rh_state = RH_RUNNING;
    hcd->state = HC_STATE_RUNNING;

    return 0;
}

static void vhcd_stop(struct usb_hcd *hcd)
{
    pr_info("Stop hcd\n");
}

int vhcd_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
    pr_info("URB enqueue\n");
    return 0;
}

int vhcd_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
    pr_info("URB dequeue\n");
    return 0;
}

int vhcd_h_get_frame(struct usb_hcd *hcd)
{
    pr_info("get frame\n");
    return 0;
}

int vhcd_hub_status(struct usb_hcd *hcd, char *buf)
{
    pr_info("hub status\n");
    return 0;
}

static inline void hub_descriptor(struct usb_hub_descriptor *desc)
{
    memset(desc, 0, sizeof(struct usb_hub_descriptor));

    /* Number of bytes in this descriptor. Including 7 bytes basic
     * with 2 bytes extra(DeviceRemovable) */
    desc->bDescLength = 7 + 2;
    /* Descriptor Type: hub */
    desc->bDescriptorType = USB_DT_HUB;
    /* Number of downstream facing ports that this hub supports */
    desc->bNbrPorts = 1;
    /* All ports power control at once */
    desc->wHubCharacteristics |= cpu_to_le16(HUB_CHAR_COMMON_LPSM);
    /* All ports Over-Current reporting */
    desc->wHubCharacteristics |= cpu_to_le16(HUB_CHAR_COMMON_OCPM);
    /* How long to wait before accessing a powered-on port.For a hub
     * with no power switches, bPwrOn2PwrGood must be set to zero. */
    desc->bPwrOn2PwrGood = 0;
    /* Maximum current requirements of the Hub Controller electronics
     * in mA */
    desc->bHubContrCurrent = 0;
    /* The bitmaps that indicate if a port has a removable device attached.
     * 0 for removable and 1 for non-removable. */
    desc->u.hs.DeviceRemovable[0] = 0;
    /* usb 1.0 legacy PortPwrCtrlMask */
    desc->u.hs.DeviceRemovable[1] = 0xff;
}

static inline void hub_status(struct usb_hub_status *status)
{
    memset(status, 0, sizeof(struct usb_hub_status));

    /* Local power supply good, No over-current condition currently
     * exists */
    status->wHubStatus = 0;
    /* No change has occurred to Local Power Status, No change has
     * occurred to the Over-Current Status. */
    status->wHubChange = 0;
}


static inline int set_port_feature(struct usb_hcd *hcd, u16 feat)
{
    int error = 0;
    struct vhcd_hcd_priv *priv = hcd_to_priv(hcd);

    switch (feat) {
        case USB_PORT_FEAT_POWER:
            /* The port is powered on. */
            priv->port_status.wPortStatus |= USB_PORT_STAT_POWER;
            set_link_state(priv);
            break;
        default:
            /* Invalid port feature */
            pr_info("SetPortFeature %04x fail\n", feat);
            error = -EPIPE;
            break;
    }

    return error;
}

int vhcd_hub_control(struct usb_hcd *hcd,
                     u16 typeReq,
                     u16 wValue,
                     u16 wIndex,
                     char *buf,
                     u16 wLength)
{
    int ret = 0;
    struct vhcd_hcd_priv *priv = hcd_to_priv(hcd);
    unsigned long flags;

    if (!HCD_HW_ACCESSIBLE(hcd))
        return -ETIMEDOUT;

    spin_lock_irqsave(&priv->lock, flags);
    switch (typeReq) {
    case GetHubDescriptor:
        pr_info("hub_control/GetHubDescriptor\n");
        hub_descriptor((struct usb_hub_descriptor *) buf);
        break;
    case GetHubStatus:
        pr_info("hub_control/GetHubStatus\n");
        hub_status((struct usb_hub_status *) buf);
        break;
    case SetPortFeature:
        pr_info("hub_control/SetPortFeature\n");
        ret = set_port_feature(hcd, wValue);
        break;
    default:
        pr_info("hub control req%04x v%04x i%04x l%d\n", typeReq, wValue,
                wIndex, wLength);
        ret = -EPIPE;
        break;
    }
    spin_unlock_irqrestore(&priv->lock, flags);

    return ret;
}

int vhcd_bus_suspend(struct usb_hcd *)
{
    pr_info("bus suspend\n");
    return 0;
}

int vhcd_bus_resume(struct usb_hcd *)
{
    pr_info("bus resume\n");
    return 0;
}

int vhcd_alloc_streams(struct usb_hcd *hcd,
                       struct usb_device *udev,
                       struct usb_host_endpoint **eps,
                       unsigned int num_eps,
                       unsigned int num_streams,
                       gfp_t mem_flags)
{
    pr_info("alloc streams\n");
    return 0;
}

int vhcd_free_streams(struct usb_hcd *hcd,
                      struct usb_device *udev,
                      struct usb_host_endpoint **eps,
                      unsigned int num_eps,
                      gfp_t mem_flags)
{
    pr_info("free streams\n");
    return 0;
}

static struct hc_driver vhcd_hc_driver = {
    .description = DEV_NAME,
    .product_desc = "vhcd host controller",
    .hcd_priv_size = sizeof(struct vhcd_hcd_priv),

    .reset = vhcd_setup,
    .start = vhcd_start,
    .stop = vhcd_stop,

    .urb_enqueue = vhcd_urb_enqueue,
    .urb_dequeue = vhcd_urb_dequeue,

    .get_frame_number = vhcd_h_get_frame,

    .hub_status_data = vhcd_hub_status,
    .hub_control = vhcd_hub_control,
    .bus_suspend = vhcd_bus_suspend,
    .bus_resume = vhcd_bus_resume,

    .alloc_streams = vhcd_alloc_streams,
    .free_streams = vhcd_free_streams,
};

static int vhcd_probe(struct platform_device *pdev)
{
    struct vhcd_data *data;
    struct usb_hcd *hs_hcd;
    int ret;

    pr_info("Probe vhcd\n");

    // Simulate as an USB 2.0 device
    vhcd_hc_driver.flags = HCD_USB2;

    hs_hcd = usb_create_hcd(&vhcd_hc_driver, &pdev->dev, dev_name(&pdev->dev));
    if (!hs_hcd)
        return -ENOMEM;

    /* Register usb_hcd after structure initialization. This will
     * trigger the ->reset()/vhcd_setup and ->start()/vhcd_start
     * callback. Note that ->start() is a must-needed callback,
     * otherwise the kernel will crash without it. */
    ret = usb_add_hcd(hs_hcd, 0, 0);
    if (ret) {
        usb_put_hcd(hs_hcd);
        /* If usb_add_hcd() return error because ->reset() success but ->start()
         * fail, this relationship could be set incorrectly. The driver needs to
         * reset it. */
        data = get_platdata(pdev);
        data->hs_hcd_priv = NULL;
        return ret;
    }

    return 0;
}

static int vhcd_remove(struct platform_device *pdev)
{
    struct vhcd_data *data;
    pr_info("Remove vhcd\n");

    data = get_platdata(pdev);

    /* Shutdown usb_hcd by reverseing what we do at usb_add_hcd().
     * This will trigger ->stop()/vhcd_stop which is a must-needed
     * callback. */
    usb_remove_hcd(priv_to_hcd(data->hs_hcd_priv));
    usb_put_hcd(priv_to_hcd(data->hs_hcd_priv));

    data->hs_hcd_priv = NULL;

    return 0;
}

static struct platform_driver vhcd_driver = {
    .probe = vhcd_probe,
    .remove = vhcd_remove,
    .driver =
        {
            .name = DEV_NAME,
        },
};

static int __init hcd_init(void)
{
    struct vhcd_data *data = NULL;
    int ret;

    if (usb_disabled())
        return -ENODEV;

    ret = platform_device_register(&vhcd_pdev);
    if (ret < 0)
        goto err_register_hcd_dev;

    data = kzalloc(sizeof(struct vhcd_data), GFP_KERNEL);
    if (!data) {
        ret = -ENOMEM;
        goto err_alloc_pdata;
    }
    ret = platform_device_add_data(&vhcd_pdev, &data, sizeof(void *));
    if (ret) {
        goto err_add_pdata;
    }

    ret = platform_driver_register(&vhcd_driver);
    if (ret < 0)
        goto err_register_hcd_driver;

    pr_info("Initialize vhcd = %d\n", ret);
    return ret;

err_register_hcd_driver:
err_add_pdata:
    kfree(data);
err_alloc_pdata:
    platform_device_unregister(&vhcd_pdev);

err_register_hcd_dev:
    return ret;
}
module_init(hcd_init);

static void __exit hcd_exit(void)
{
    struct vhcd_data *data = get_platdata(&vhcd_pdev);
    platform_device_unregister(&vhcd_pdev);
    kfree(data);
    platform_driver_unregister(&vhcd_driver);
    pr_info("Exit vhcd\n");
}
module_exit(hcd_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("RinHizakura");
MODULE_DESCRIPTION("The virtual USB platform device and its driver");
