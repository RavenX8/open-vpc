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

// Report Responses
#define VPC_CMD_BUSY          0x01
#define VPC_CMD_SUCCESSFUL    0x02
#define VPC_CMD_FAILURE       0x03
#define VPC_CMD_TIMEOUT       0x04
#define VPC_CMD_NOT_SUPPORTED 0x05

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

int vpc_send_control_msg(struct usb_device *usb_dev,struct vpc_report const *data, unsigned int report_index, unsigned long wait_min, unsigned long wait_max);
int vpc_get_usb_response(struct usb_device *usb_dev, uint report_index, struct vpc_report* request_report, unsigned int response_index, struct vpc_report* response_report, unsigned long wait_min, unsigned long wait_max);
struct vpc_report get_vpc_report(unsigned char report_id, unsigned char feature);
struct vpc_report get_empty_vpc_report(void);
void print_erroneous_report(struct vpc_report* report, char* driver_name, char* message);


#endif //VPCCOMMON_H
