//
// Created by raven on 3/25/24.
//

#ifndef VPCCOMMON_H
#define VPCCOMMON_H

#include <linux/usb/input.h>

#define DRIVER_VERSION "0.1.0"
#define DRIVER_LICENSE "GPL v2"
#define DRIVER_AUTHOR "Christopher Torres <ctorres@azgstudio.com>"

// Compatbility for fallthrough pseudo keyword for Linux versions older than v5.4
// See also https://git.kernel.org/torvalds/c/294f69e
#ifndef fallthrough
#if __has_attribute(__fallthrough__)
# define fallthrough                    __attribute__((__fallthrough__))
#else
# define fallthrough                    do {} while (0)  /* fallthrough */
#endif
#endif

// Macro to create device files
#define CREATE_DEVICE_FILE(dev, type) \
do { \
if(device_create_file(dev, type)) { \
goto exit_free; \
} \
} while (0)

#define USB_VENDOR_ID_VPC 0x3344

// Report response
#define VPC_USB_REPORT_FIRMWARE_LEN 0x64
#define VPC_USB_REPORT_MODE_LEN 0x03

#define VPC_DESC_DEVICE_MAIN 0x00

#define VPC_DESC_STRING_CHECKSUM 0x00
#define VPC_DESC_STRING_FIRMWARE_ID 0x01
#define VPC_DESC_STRING_NAME 0x02
#define VPC_DESC_STRING_UNIQ 0x03 // Not sure what this is for. Always is 0xFF

enum vpc_report_id {
    REPORT_FIRMWARE = 0x02,
    REPORT_STATE = 0x03,
    REPORT_MODE = 0x04,
};

struct vpc_report_data {
    unsigned char command_id;
    unsigned char arguments[2];
};

struct vpc_report {
    unsigned char report_id;
    unsigned char feature_id;
    struct vpc_report_data data;
};

int vpc_get_usb_discriptor_device(struct usb_device *usb_dev, unsigned char index, struct usb_device_descriptor* response);
int vpc_get_usb_discriptor_string(struct usb_device *usb_dev, unsigned char index, unsigned char* response);
int vpc_send_control_msg(struct usb_device *usb_dev,struct vpc_report const *data, unsigned int report_index, unsigned long wait_min, unsigned long wait_max);
int vpc_get_usb_response(struct usb_device *usb_dev, uint report_index, struct vpc_report* request_report, unsigned int response_index, struct vpc_report* response_report, unsigned long wait_min, unsigned long wait_max);
struct vpc_report get_vpc_report(unsigned char report_id, unsigned char feature);
struct vpc_report get_empty_vpc_report(void);
void print_erroneous_report(struct vpc_report* report, char* driver_name, char* message);


#endif //VPCCOMMON_H
