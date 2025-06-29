#define pr_fmt(fmt) "vusb:" fmt

#include "common.h"

static const char hcd_name[] = "vhcd";

static inline struct usb_hcd *vhcd_to_hcd(struct vhcd *vhcd)
{
    return container_of((void *) vhcd, struct usb_hcd, hcd_priv);
}

static inline struct vhcd *hcd_to_vhcd(struct usb_hcd *hcd)
{
    return (struct vhcd *) (hcd->hcd_priv);
}

static inline void vhcd_init(struct vhcd *vhcd, struct virt *virt)
{
    vhcd->virt = virt;
    vhcd->rh_state = RH_RESET;
    vhcd->port_status = (struct usb_port_status){
        .wPortStatus = 0,
        .wPortChange = 0,
    };
    vhcd->active = 0;
}

static void set_link_state(struct vhcd *vhcd) __must_hold(&virt->lock)
{
    /* Record status and active, so we can compare the changes for them. */
    struct virt *virt = vhcd->virt;
    struct usb_port_status port_status_old = vhcd->port_status;
    int active_old = vhcd->active;

    if (vhcd->port_status.wPortStatus & USB_PORT_STAT_POWER) {
        if (!virt->pullup) {
            vhcd->port_status.wPortStatus &=
                ~(USB_PORT_STAT_CONNECTION | USB_PORT_STAT_ENABLE);

            if (port_status_old.wPortStatus & USB_PORT_STAT_CONNECTION)
                vhcd->port_status.wPortChange |= USB_PORT_STAT_C_CONNECTION;
        } else {
            /* A device is present on this port */
            vhcd->port_status.wPortStatus |= USB_PORT_STAT_CONNECTION;

            /* The port status change from disconnect to connect. */
            if (!(port_status_old.wPortStatus & USB_PORT_STAT_CONNECTION))
                vhcd->port_status.wPortChange |= USB_PORT_STAT_C_CONNECTION;

            vhcd->active = 1;
        }
    } else {
        vhcd->port_status.wPortStatus = 0;
        vhcd->port_status.wPortChange = 0;
    }
}

static void vhcd_pdev_dummy_release(struct device *dev) {}

struct platform_device vhcd_pdev = {
    .name = hcd_name,
    .id = PLATFORM_DEVID_NONE,
    .dev.release = vhcd_pdev_dummy_release,
};

static int vhcd_setup(struct usb_hcd *hcd)
{
    struct virt *virt;

    INFO("Setup hcd\n");

    virt = *((void **) dev_get_platdata(hcd->self.controller));
    hcd->self.sg_tablesize = ~0;

    virt->hs_hcd = (struct vhcd *) (hcd->hcd_priv);
    vhcd_init(virt->hs_hcd, virt);

    hcd->speed = HCD_USB2;
    hcd->self.root_hub->speed = USB_SPEED_HIGH;

    return 0;
}

static int vhcd_start(struct usb_hcd *hcd)
{
    struct vhcd *vhcd = hcd_to_vhcd(hcd);
    struct virt *virt = vhcd->virt;

    INFO("Start hcd\n");

    spin_lock_init(&virt->lock);
    vhcd->rh_state = RH_RUNNING;
    hcd->state = HC_STATE_RUNNING;

    return 0;
}

static void vhcd_stop(struct usb_hcd *hcd)
{
    INFO("Stop hcd\n");
}

int vhcd_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
    INFO("URB enqueue\n");
    return 0;
}

int vhcd_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
    INFO("URB dequeue\n");
    return 0;
}

int vhcd_h_get_frame(struct usb_hcd *hcd)
{
    INFO("get frame\n");
    return 0;
}

#define PORT_C_MASK                                          \
    (USB_PORT_STAT_C_CONNECTION | USB_PORT_STAT_C_ENABLE |   \
     USB_PORT_STAT_C_SUSPEND | USB_PORT_STAT_C_OVERCURRENT | \
     USB_PORT_STAT_C_RESET)

#define HUB_PORTS_NUM 1

int vhcd_hub_status(struct usb_hcd *hcd, char *buf)
{
    int port;
    /* Here default ret to the valid bytes of buf */
    int ret = (HUB_PORTS_NUM + 8) / 8;
    int status = 0;
    unsigned long flags;
    struct vhcd *vhcd = hcd_to_vhcd(hcd);
    struct virt *virt = vhcd->virt;

    INFO("hub status\n");

    if (!HCD_HW_ACCESSIBLE(hcd))
        return 0;

    memset(buf, 0, ret);

    spin_lock_irqsave(&virt->lock, flags);
    if ((vhcd->port_status.wPortChange & PORT_C_MASK) != 0) {
        /* The port 0 has status change. Note that ports are
         * 1-indexed from the USB core pointer of view. */
        status = 1;
        port = 0;
        *buf = ((port + 1) << 1);
    }

    spin_unlock_irqrestore(&virt->lock, flags);

    return status ? ret : 0;
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
    desc->bNbrPorts = HUB_PORTS_NUM;
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

static inline void get_port_status(struct vhcd *vhcd,
                                   struct usb_port_status *status)
{
    /* Since the usbcore will poll the port status to finish reset.
     * Here we simulate reset process by waiting a few time and
     * updating the port status. */
    if ((vhcd->port_status.wPortStatus & USB_PORT_STAT_RESET) &&
        time_after_eq(jiffies, vhcd->re_timeout)) {
        vhcd->port_status.wPortChange |= USB_PORT_STAT_C_RESET;
        vhcd->port_status.wPortStatus &= ~USB_PORT_STAT_RESET;
    }
    set_link_state(vhcd);

    status->wPortStatus = vhcd->port_status.wPortStatus;
    status->wPortChange = vhcd->port_status.wPortChange;
}


static inline int set_port_feature(struct usb_hcd *hcd, u16 feat)
{
    int error = 0;
    struct vhcd *vhcd = hcd_to_vhcd(hcd);

    switch (feat) {
    case USB_PORT_FEAT_POWER:
        INFO("SetPortFeature POWER\n");
        /* The port is powered on. */
        vhcd->port_status.wPortStatus |= USB_PORT_STAT_POWER;
        set_link_state(vhcd);
        break;
    case USB_PORT_FEAT_RESET:
        INFO("SetPortFeature RESET\n");
        /* The port should only be reset under connecting */
        if (!(vhcd->port_status.wPortStatus & USB_PORT_STAT_CONNECTION))
            break;
        vhcd->port_status.wPortStatus |= USB_PORT_STAT_RESET;

        /* TDRSTR = 50ms */
        vhcd->re_timeout = jiffies + msecs_to_jiffies(50);
        set_link_state(vhcd);
        break;
    default:
        /* Invalid port feature */
        INFO("SetPortFeature %04x fail\n", feat);
        error = -EPIPE;
        break;
    }

    return error;
}

static inline int clear_port_feature(struct usb_hcd *hcd, u16 feat)
{
    int error = 0;
    struct vhcd *vhcd = hcd_to_vhcd(hcd);

    switch (feat) {
    case USB_PORT_FEAT_ENABLE:
        vhcd->port_status.wPortStatus &= ~USB_PORT_STAT_ENABLE;
        break;
    case USB_PORT_FEAT_POWER:
        INFO("ClearPortFeature POWER\n");
        /* The port is powered off. */
        vhcd->port_status.wPortStatus &= ~USB_PORT_STAT_POWER;
        set_link_state(vhcd);
        break;
    case USB_PORT_FEAT_C_CONNECTION:
        INFO("ClearPortFeature C_CONN\n");
        vhcd->port_status.wPortChange &= ~USB_PORT_STAT_C_CONNECTION;
        set_link_state(vhcd);
        break;
    case USB_PORT_FEAT_C_RESET:
        INFO("ClearPortFeature C_RESET\n");
        vhcd->port_status.wPortChange &= ~USB_PORT_STAT_C_RESET;
        set_link_state(vhcd);
        break;
    default:
        /* Invalid port feature */
        INFO("ClearPortFeature %04x fail\n", feat);
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
    struct vhcd *vhcd = hcd_to_vhcd(hcd);
    struct virt *virt = vhcd->virt;
    unsigned long flags;

    if (!HCD_HW_ACCESSIBLE(hcd))
        return -ETIMEDOUT;

    spin_lock_irqsave(&virt->lock, flags);
    switch (typeReq) {
    case GetHubDescriptor:
        INFO("hub_control/GetHubDescriptor\n");
        hub_descriptor((struct usb_hub_descriptor *) buf);
        break;
    case GetHubStatus:
        INFO("hub_control/GetHubStatus\n");
        hub_status((struct usb_hub_status *) buf);
        break;
    case SetPortFeature:
        INFO("hub_control/SetPortFeature\n");
        ret = set_port_feature(hcd, wValue);
        break;
    case GetPortStatus:
        INFO("hub_control/GetPortStatus\n");
        /* The port number(wIndex) should always be 1 as we simulate
         * an one port hub. */
        if (wIndex != 1)
            ret = -EPIPE;
        get_port_status(vhcd, (struct usb_port_status *) buf);
        break;
    case ClearPortFeature:
        INFO("hub_control/ClearPortFeature\n");
        ret = clear_port_feature(hcd, wValue);
        break;
    default:
        INFO("hub control req%04x v%04x i%04x l%d\n", typeReq, wValue, wIndex,
             wLength);
        ret = -EPIPE;
        break;
    }
    spin_unlock_irqrestore(&virt->lock, flags);

    return ret;
}

int vhcd_bus_suspend(struct usb_hcd *)
{
    INFO("bus suspend\n");
    return 0;
}

int vhcd_bus_resume(struct usb_hcd *)
{
    INFO("bus resume\n");
    return 0;
}

int vhcd_alloc_streams(struct usb_hcd *hcd,
                       struct usb_device *udev,
                       struct usb_host_endpoint **eps,
                       unsigned int num_eps,
                       unsigned int num_streams,
                       gfp_t mem_flags)
{
    INFO("alloc streams\n");
    return 0;
}

int vhcd_free_streams(struct usb_hcd *hcd,
                      struct usb_device *udev,
                      struct usb_host_endpoint **eps,
                      unsigned int num_eps,
                      gfp_t mem_flags)
{
    INFO("free streams\n");
    return 0;
}

static struct hc_driver vhcd_hc_driver = {
    .description = hcd_name,
    .product_desc = "vhcd host controller",
    .hcd_priv_size = sizeof(struct vhcd),

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
    struct virt *virt;
    struct usb_hcd *hs_hcd;
    int ret;

    INFO("Probe vhcd\n");

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
        virt = get_platdata(pdev);
        virt->hs_hcd = NULL;
        return ret;
    }

    return 0;
}

static int vhcd_remove(struct platform_device *pdev)
{
    struct virt *virt;
    INFO("Remove vhcd\n");

    virt = get_platdata(pdev);

    /* Shutdown usb_hcd by reverseing what we do at usb_add_hcd().
     * This will trigger ->stop()/vhcd_stop which is a must-needed
     * callback. */
    usb_remove_hcd(vhcd_to_hcd(virt->hs_hcd));
    usb_put_hcd(vhcd_to_hcd(virt->hs_hcd));

    virt->hs_hcd = NULL;

    return 0;
}

struct platform_driver vhcd_driver = {
    .probe = vhcd_probe,
    .remove = vhcd_remove,
    .driver =
        {
            .name = hcd_name,
        },
};
