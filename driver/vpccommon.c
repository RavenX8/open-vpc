// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2024 Christopher Torres <ctorres@azgstudio.com>
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/hid.h>

#include "vpccommon.h"

/**
 * Send USB control report
 * USUALLY index = 0x02
 * FIREFLY is 0
 */
int vpc_send_control_msg(struct usb_device *usb_dev,struct vpc_report const *data, uint report_index, ulong wait_min, ulong wait_max)
{
    uint request = HID_REQ_SET_REPORT; // 0x09
    uint request_type = USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT; // 0x21
    uint value = ((uint)data->report_id << 8) | data->feature_id;
    uint size = 0;

    switch (data->feature_id) {
        case REPORT_MODE:
            size = VPC_USB_REPORT_MODE_LEN;
            break;
        default:
            printk(KERN_WARNING "vpcdriver: feature_id is unsupported. USB Report feature: %02x\n", data->feature_id);
            return -ENXIO;
    }

    char *buf;
    int len;

    buf = kmemdup(&data->data, size, GFP_KERNEL);
    if (buf == NULL)
        return -ENOMEM;

    // Send usb control message
    len = usb_control_msg(usb_dev, usb_sndctrlpipe(usb_dev, 0),
                       request,      // Request
                       request_type, // RequestType
                       value,        // Value
                       report_index, // Index
                       buf,          // Data
                       size,         // Length
                       USB_CTRL_SET_TIMEOUT);

    // Wait
    usleep_range(wait_min, wait_max);

    kfree(buf);
    if(len!=size)
    printk(KERN_WARNING "vpc driver: Device data transfer failed. len: %02x\n", len);

    return ((len < 0) ? len : ((len != size) ? -EIO : 0));
}

/**
 * Get a response from the vpc device
 *
 * Makes a request like normal, this must change a variable in the device as then we
 * tell it give us data and it gives us a report.
 *
 * Supported Devices:
 *   MT-50CM2
 *   MT-50CM3
 *
 * Request report is the report sent to the device specifying what response we want
 * Response report will get populated with a response
 *
 * Returns 0 when successful, 1 if the report feture is not supported, 2 if the report length is invalid.
 */
int vpc_get_usb_response(struct usb_device *usb_dev, uint report_index, struct vpc_report* request_report, uint response_index, struct vpc_report* response_report, ulong wait_min, ulong wait_max)
{
    uint request = HID_REQ_GET_REPORT; // 0x01
    uint request_type = USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_IN; // 0xA1
    uint value = ((uint)request_report->report_id << 8) | request_report->feature_id;
    uint size = 0;

    switch (request_report->feature_id) {
        case REPORT_MODE:
            size = VPC_USB_REPORT_MODE_LEN;
            break;
        default:
            printk(KERN_WARNING "vpcdriver: feature_id is unsupported. USB Report feature: %02x\n", request_report->feature_id);
            return 1;
    }

    int len;
    int retval;
    int result = 0;
    char *buf;

    buf = kzalloc(sizeof(struct vpc_report_data), GFP_KERNEL);
    if (buf == NULL)
        return -ENOMEM;

    // retval = vpc_send_control_msg(usb_dev, request_report, report_index, wait_min, wait_max);
    // Now ask for response
    len = usb_control_msg(usb_dev, usb_rcvctrlpipe(usb_dev, 0),
                          request,         // Request
                          request_type,    // RequestType
                          value,           // Value
                          response_index,  // Index
                          buf,             // Data
                          size,
                          USB_CTRL_SET_TIMEOUT);

    memcpy(&response_report->data, buf, sizeof(struct vpc_report_data));
    kfree(buf);

    // Error if report is wrong length
    if(len != size-1) {
        printk(KERN_WARNING "vpcdriver: Invalid USB response. USB Report length: %d\n", len);
        result = 2;
    }

    return result;
}

int vpc_get_usb_discriptor_device(struct usb_device *usb_dev,  unsigned char index, struct usb_device_descriptor* response) {
    int len = 0;
    int result = 0;
    char *buf = NULL;

    buf = kzalloc(USB_DT_DEVICE_SIZE, GFP_KERNEL);
    if (buf == NULL)
        return -ENOMEM;

    len = usb_get_descriptor(usb_dev, USB_DT_DEVICE, index, buf, USB_DT_DEVICE_SIZE);

    if(len < 0) {
        printk(KERN_WARNING "vpcdriver: Invalid USB response. USB Response length: %d\n", len);
        result = 1;
    }
    else {
        memcpy(&response, buf, len);
    }

    kfree(buf);
    return result;
}

int vpc_get_usb_discriptor_string(struct usb_device *usb_dev, unsigned char index, unsigned char* response) {
    int len = 0;
    int result = 0;
    char *buf = NULL;

    buf = kzalloc(USB_MAX_STRING_LEN, GFP_KERNEL);
    if (buf == NULL)
        return -ENOMEM;

    len = usb_string(usb_dev, index, buf, USB_MAX_STRING_LEN);

    if(len < 0) {
        printk(KERN_WARNING "vpcdriver: Invalid USB response. USB Response length: %d\n", len);
        result = -1;
    }
    else {
        memcpy(response, buf, len <= USB_MAX_STRING_LEN ? len : USB_MAX_STRING_LEN);
    }

    kfree(buf);
    return len;
}

/**
 * Get initialised vpc report
 */
struct vpc_report get_vpc_report(unsigned char report_id, unsigned char feature)
{
    struct vpc_report new_report = {0};
    memset(&new_report, 0, sizeof(struct vpc_report));
    new_report.report_id = report_id;
    new_report.feature_id = feature;

    return new_report;
}

/**
 * Get empty vpc report
 */
struct vpc_report get_empty_vpc_report(void)
{
 struct vpc_report new_report = {0};
 memset(&new_report, 0, sizeof(struct vpc_report));

 return new_report;
}

/**
 * Print report to syslog
 */
void print_erroneous_report(struct vpc_report* report, char* driver_name, char* message)
{
 printk(KERN_WARNING "%s: %s. report_id: %02x feature_id: %02x .\n",
        driver_name,
        message,
        report->report_id,
        report->feature_id);
}
