#define pr_fmt(fmt) "vusb:" fmt

#include "common.h"

struct virt_request {
    struct usb_request req;
};

static const char udc_name[] = "vudc";

static void vudc_pdev_dummy_release(struct device *dev) {}

struct platform_device vudc_pdev = {
    .name = udc_name,
    .id = PLATFORM_DEVID_NONE,
    .dev.release = vudc_pdev_dummy_release,
};

static inline struct virt *get_gadget_dev_data(struct device *dev)
{
    return container_of(dev, struct virt, gadget.dev);
}

static int virt_enable(struct usb_ep *_ep,
                       const struct usb_endpoint_descriptor *desc)
{
    INFO("EP enable");
    return 0;
}

static int virt_disable(struct usb_ep *_ep)
{
    INFO("EP disable");
    return 0;
}

static struct usb_request *virt_alloc_request(struct usb_ep *ep,
                                              gfp_t mem_flags)
{
    struct virt_request *virt_req;

    INFO("EP alloc request");

    if (!ep)
        return NULL;

    virt_req = kzalloc(sizeof(struct virt_request), mem_flags);
    if (!virt_req)
        return NULL;

    return &virt_req->req;
}

static void virt_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
    INFO("EP free request");
    return;
}

static int virt_queue(struct usb_ep *_ep,
                      struct usb_request *_req,
                      gfp_t mem_flags)
{
    INFO("EP queue");
    return 0;
}

static int virt_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
    INFO("EP dequeue");
    return 0;
}

static int virt_set_halt(struct usb_ep *_ep, int value)
{
    INFO("EP set_halt");
    return 0;
}

static int virt_set_wedge(struct usb_ep *_ep)
{
    INFO("EP set_wedge");
    return 0;
}

static const struct usb_ep_ops virt_ep_ops = {
    .enable = virt_enable,
    .disable = virt_disable,

    .alloc_request = virt_alloc_request,
    .free_request = virt_free_request,

    .queue = virt_queue,
    .dequeue = virt_dequeue,

    .set_halt = virt_set_halt,
    .set_wedge = virt_set_wedge,
};

static int vudc_get_frame(struct usb_gadget *_gadget)
{
    INFO("UDC get frame");
    return 0;
}

static int vudc_wakeup(struct usb_gadget *_gadget)
{
    INFO("UDC wakeup");
    return 0;
}

static int vudc_set_selfpowered(struct usb_gadget *_gadget, int value)
{
    INFO("UDC set selfpowered");
    return 0;
}

static int vudc_pullup(struct usb_gadget *_gadget, int value)
{
    INFO("UDC pullup");
    return 0;
}

static int vudc_udc_start(struct usb_gadget *g,
                          struct usb_gadget_driver *driver)
{
    INFO("UDC start");
    return 0;
}

static int vudc_udc_stop(struct usb_gadget *g)
{
    INFO("UDC stop");
    return 0;
}



static void vudc_set_speed(struct usb_gadget *gadget,
                           enum usb_device_speed speed)
{
    struct virt *virt = get_gadget_dev_data(&gadget->dev);

    /* We expect the speed is USB_SPEED_HIGH(= 3) here, so the
     * implemention just simply designed on the assumption. */
    INFO("UDC set_speed %d", speed);

    gadget->speed = speed;
    /* TODO: how to decide the maxpacket number? */
    virt->ep.ep.maxpacket = 64;
}

static void vudc_udc_async_callbacks(struct usb_gadget *_gadget, bool enable)
{
    INFO("UDC async callbacks");
    return;
}

static const struct usb_gadget_ops vudc_gadget_ops = {
    .get_frame = vudc_get_frame,
    .wakeup = vudc_wakeup,
    .set_selfpowered = vudc_set_selfpowered,
    .pullup = vudc_pullup,
    .udc_start = vudc_udc_start,
    .udc_stop = vudc_udc_stop,
    .udc_set_speed = vudc_set_speed,
    .udc_async_callbacks = vudc_udc_async_callbacks,
};

static ssize_t function_show(struct device *dev,
                             struct device_attribute *attr,
                             char *buf)
{
    struct virt *virt = get_gadget_dev_data(dev);

    return 0;
}
static DEVICE_ATTR_RO(function);

/* EP0 is a control endpoint, and should be the endpoint that support
 * bidirection. */
static const struct usb_ep_caps usb_ep0_cap =
    USB_EP_CAPS(USB_EP_CAPS_TYPE_CONTROL, USB_EP_CAPS_DIR_ALL);

static void vudc_init_virt_ep(struct virt *virt)
{
    struct virt_ep *virt_ep = &virt->ep;
    struct usb_ep *usb_ep = &virt_ep->ep;

    INIT_LIST_HEAD(&virt->gadget.ep_list);

    /* Initialize EP0 */
    usb_ep->name = "ep0";
    usb_ep->caps = usb_ep0_cap;
    usb_ep->ops = &virt_ep_ops;
    list_add_tail(&usb_ep->ep_list, &virt->gadget.ep_list);
    usb_ep_set_maxpacket_limit(usb_ep, ~0);
    usb_ep->max_streams = 16;
    virt->gadget.ep0 = usb_ep;
}

static int vudc_probe(struct platform_device *pdev)
{
    int ret;
    struct virt *virt = get_platdata(pdev);

    INFO("Probe vudc\n");

    memset(&virt->gadget, 0, sizeof(struct usb_gadget));

    // Simulate as an USB 2.0 device
    virt->gadget.ops = &vudc_gadget_ops;
    virt->gadget.name = udc_name;
    virt->gadget.max_speed = USB_SPEED_HIGH;
    virt->gadget.dev.parent = &pdev->dev;
    vudc_init_virt_ep(virt);

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
            .name = udc_name,
        },
};
