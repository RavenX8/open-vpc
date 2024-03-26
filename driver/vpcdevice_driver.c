//
// Created by raven on 3/25/24.
//

#include <linux/dmi.h>
#include <linux/hid.h>
#include <linux/init.h>
#include <linux/input-event-codes.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb/input.h>

#include "vpccommon.h"
#include "vpcdevice_driver.h"

/*
 * Version Information
 */
#define DRIVER_DESC "VirPil Controls Device Driver"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE(DRIVER_LICENSE);


/**
 * Get request/response indices and timing parameters for the device
 */
static void vpc_get_report_params(struct usb_device *usb_dev, uint *report_index, uint *response_index, ulong *wait_min,
                                  ulong *wait_max) {
    switch (usb_dev->descriptor.idProduct) {
        case USB_DEVICE_ID_VIRPIL_STICK_MT_50CM2:
        case USB_DEVICE_ID_VIRPIL_STICK_MT_50CM3:
        case USB_DEVICE_ID_VIRPIL_THROTTLE_MT_50CM3:
        default:
            *report_index = 0x01;
            *response_index = 0x01;
            *wait_min = VPC_WAIT_MIN_US;
            *wait_max = VPC_WAIT_MAX_US;
            break;
    }
}

/**
 * Send report to the device
 */
static int vpc_get_report(struct usb_device *usb_dev, struct vpc_report *request, struct vpc_report *response) {
    return vpc_get_usb_response(usb_dev, 0x00, request, 0x00, response, VPC_WAIT_MIN_US, VPC_WAIT_MAX_US);
}

/**
 * Send report to the device, but without even reading the response
 */
static int vpc_send_payload_no_response(struct vpc_device *device, struct vpc_report *request) {
    uint report_index, response_index;
    ulong wait_min, wait_max;

    // vpc_get_report_params(device->usb_dev, &report_index, &response_index, &wait_min, &wait_max);
    return vpc_send_control_msg(device->usb_dev, request, report_index, wait_min, wait_max);
}

/**
 * Function to send to device, get response, and actually check the response
 */
static int vpc_send_payload(struct vpc_device *device, struct vpc_report *request, struct vpc_report *response) {
    int err;

    mutex_lock(&device->lock);
    err = vpc_get_report(device->usb_dev, request, response);
    mutex_unlock(&device->lock);
    if (err) {
        print_erroneous_report(response, "vpcdevice", "Invalid Report Length");
        return err;
    }

    // switch (response->feature_id) {
    //     default:
    //         print_erroneous_report(response, "vpcdevice", "Response recvd");
    //         break;
    // }

    return 0;
}

/**
 * Write device file "test"
 *
 * Does nothing
 */
static ssize_t vpc_attr_write_test(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
    return count;
}

/**
 * Read device file "test"
 *
 * Returns a string
 */
static ssize_t vpc_attr_read_test(struct device *dev, struct device_attribute *attr, char *buf) {
    struct vpc_device *device = dev_get_drvdata(dev);
    struct vpc_report request = {0};
    struct vpc_report response = {0};

    request = get_vpc_report(0x03, REPORT_MODE);

    vpc_send_payload(device, &request, &response);

    print_erroneous_report(&response, "vpcdevice", "Test");
    return sprintf(buf, "%02x%02x\n", response.data.arguments[0], response.data.arguments[0]);
}

/**
 * Write device file "test"
 *
 * Does nothing
 */
static ssize_t vpc_attr_write_shift_mode(struct device *dev, struct device_attribute *attr, const char *buf,
                                         size_t count) {
    struct vpc_device *device = dev_get_drvdata(dev);
    struct vpc_report request = {0};
    unsigned long mode = 0;
    int res = (unsigned char) kstrtoul(buf, 10, &mode);

    if (res != 0) {
        printk(KERN_WARNING "vpcdevice: error while writing shift mode. kstrtoul returned: %i .\n", res);
        return count;
    }

    request.report_id = 0x03;
    request.feature_id = REPORT_MODE;
    request.data.command_id = 0x04;
    request.data.arguments[0] = mode;
    request.data.arguments[1] = 0x00;

    vpc_send_payload_no_response(device, &request);
    return count;
}

/**
 * Read device file "test"
 *
 * Returns a string
 */
static ssize_t vpc_attr_read_shift_mode(struct device *dev, struct device_attribute *attr, char *buf) {
    struct vpc_device *device = dev_get_drvdata(dev);
    struct vpc_report request = {0};
    struct vpc_report response = {0};

    request = get_vpc_report(0x03, REPORT_MODE);

    vpc_send_payload(device, &request, &response);

    print_erroneous_report(&response, "vpcdevice", "shift_mode");
    return sprintf(buf, "%02x%02x\n", response.data.arguments[0], response.data.arguments[1]);
}

/**
 * Read device file "device_type"
 *
 * Returns friendly string of device type
 */
static ssize_t vpc_attr_read_device_type(struct device *dev, struct device_attribute *attr, char *buf) {
    struct vpc_device *device = dev_get_drvdata(dev);

    char *device_type;

    switch (device->usb_pid) {

        case USB_DEVICE_ID_VIRPIL_STICK_MT_50CM2:
            device_type = "Virpil Stick MT-50CM2\n";
            break;

        case USB_DEVICE_ID_VIRPIL_STICK_MT_50CM3:
            device_type = "Virpil Stick MT-50CM3\n";
            break;

        case USB_DEVICE_ID_VIRPIL_THROTTLE_MT_50CM3:
            device_type = "Virpil Throttle MT-50CM3\n";
            break;

        case USB_DEVICE_ID_VIRPIL_PEDALS_ACE:
            device_type = "Virpil Ace Rudder Pedals\n";
            break;

        default:
            device_type = "Unknown Device\n";
    }

    return sprintf(buf, device_type);
}

/**
 * Set up the device driver files

 *
 * Read only is 0444
 * Write only is 0220
 * Read and write is 0660
 */
static DEVICE_ATTR(test, 0660, vpc_attr_read_test, vpc_attr_write_test);
static DEVICE_ATTR(device_type, 0440, vpc_attr_read_device_type, NULL);
static DEVICE_ATTR(shift_mode, 0660, vpc_attr_read_shift_mode, vpc_attr_write_shift_mode);


static int vpc_event(struct hid_device *hdev, struct hid_field *field, struct hid_usage *usage, __s32 value) {
    struct vpc_device *device = hid_get_drvdata(hdev);
    // TODO: maybe there is something to do here
    //  printk(KERN_WARNING "vpcdevice: normal event recv'd. usage.code: %02x .\n",
    //  usage->code);
    return 0;
}

/**
 * Raw event function
 *
 * Handles provided HID reports, branched out for specific keyboard models, since some keyboards need specific handling.
 */
static int vpc_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int size) {
    struct vpc_device *device = hid_get_drvdata(hdev);
    struct usb_interface *intf = to_usb_interface(hdev->dev.parent);

    if (size > 0) {
        printk(KERN_WARNING "vpcdevice: raw event recv'd. size: %i data: %02x .\n", size, data[0]);
        return 0;
    }

    printk(KERN_WARNING "vpcdevice: raw event recv'd with zero data. .\n");
    return 0;
}

static int vpc_device_input_mapping(struct hid_device *hdev, struct hid_input *hidinput, struct hid_field *field,
                                    struct hid_usage *usage, unsigned long **bit, int *max) {
    switch (hdev->product) {
        default:
            return 0;
    }
}

static void vpc_device_init(struct vpc_device *dev, struct usb_interface *intf, struct hid_device *hdev) {
    printk(KERN_WARNING "vpcdevice: start device init. .\n");
    struct usb_device *usb_dev = interface_to_usbdev(intf);

    // Initialise mutex
    mutex_init(&dev->lock);
    // Setup values
    dev->usb_dev = usb_dev;
    dev->usb_vid = usb_dev->descriptor.idVendor;
    dev->usb_pid = usb_dev->descriptor.idProduct;
    dev->usb_interface_protocol = intf->cur_altsetting->desc.bInterfaceProtocol;
    printk(KERN_WARNING "vpcdevice: device interface: %01x.\n", dev->usb_interface_protocol);

    printk(KERN_WARNING "vpcdevice: Finished device init. .\n");
}

/**
 * Probe method is ran whenever a device is binded to the driver
 */
static int vpc_device_probe(struct hid_device *hdev, const struct hid_device_id *id) {
    int retval = 0;
    struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
    struct usb_device *usb_dev = interface_to_usbdev(intf);
    struct vpc_device *dev = NULL;

    dev = kzalloc(sizeof(struct vpc_device), GFP_KERNEL);
    if (dev == NULL) {
        dev_err(&intf->dev, "out of memory\n");
        return -ENOMEM;
    }

    // Init data
    vpc_device_init(dev, intf, hdev);

    // TODO: create device files here
    if (intf->cur_altsetting->desc.bInterfaceProtocol == 0x00) {
        CREATE_DEVICE_FILE(&hdev->dev, &dev_attr_test); // Test mode
        CREATE_DEVICE_FILE(&hdev->dev, &dev_attr_device_type); // Get string of device type
        CREATE_DEVICE_FILE(&hdev->dev, &dev_attr_shift_mode); // Shift mode
    }

    hid_set_drvdata(hdev, dev);
    dev_set_drvdata(&hdev->dev, dev);

    if (hid_parse(hdev)) {
        hid_err(hdev, "parse failed\n");
        goto exit_free;
    }

    if (hid_hw_start(hdev, HID_CONNECT_DEFAULT)) {
        hid_err(hdev, "hw start failed\n");
        goto exit_free;
    }
    return 0;

exit_free:
    kfree(dev);
    return retval;
}

/**
 * Unbind function
 */
static void vpc_device_disconnect(struct hid_device *hdev) {
    struct vpc_device *dev;
    struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
    struct usb_device *usb_dev = interface_to_usbdev(intf);

    dev = hid_get_drvdata(hdev);

    // TODO: Remove device files
    if (intf->cur_altsetting->desc.bInterfaceProtocol == 0x00) {
        device_remove_file(&hdev->dev, &dev_attr_test); // Test mode
        device_remove_file(&hdev->dev, &dev_attr_device_type); // Get string of device type
        device_remove_file(&hdev->dev, &dev_attr_shift_mode); // Shift mode
    }

    hid_hw_stop(hdev);
    kfree(dev);
    dev_info(&intf->dev, "VPC Device disconnected\n");
}

/**
 * Setup input device keybit mask
 */
static void vpc_setup_key_bits(struct input_dev *input) { __set_bit(EV_KEY, input->evbit); }

/**
 * Setup the input device now that its been added to our struct
 */
static int vpc_input_configured(struct hid_device *hdev, struct hid_input *hi) {
    vpc_setup_key_bits(hi->input);
    return 0;
}


/**
 * Device ID mapping table
 */
static const struct hid_device_id vpc_devices[] = {
        {HID_USB_DEVICE(USB_VENDOR_ID_VPC, USB_DEVICE_ID_VIRPIL_STICK_MT_50CM2)},
        {HID_USB_DEVICE(USB_VENDOR_ID_VPC, USB_DEVICE_ID_VIRPIL_STICK_MT_50CM3)},
        {HID_USB_DEVICE(USB_VENDOR_ID_VPC, USB_DEVICE_ID_VIRPIL_THROTTLE_MT_50CM3)},
        {HID_USB_DEVICE(USB_VENDOR_ID_VPC, USB_DEVICE_ID_VIRPIL_PEDALS_ACE)},
        {0}};

MODULE_DEVICE_TABLE(hid, vpc_devices);

/**
 * Describes the contents of the driver
 */
static struct hid_driver vpc_device_driver = {
        .name = "vpcdevice",
        .id_table = vpc_devices,
        .input_mapping = vpc_device_input_mapping,
        .probe = vpc_device_probe,
        .remove = vpc_device_disconnect,
        .event = vpc_event,
        .raw_event = vpc_raw_event,
        .input_configured = vpc_input_configured,
};

module_hid_driver(vpc_device_driver);
