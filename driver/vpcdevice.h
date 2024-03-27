//
// Created by raven on 3/25/24.
//

#ifndef VPCDEVICE_DRIVER_H
#define VPCDEVICE_DRIVER_H

#define USB_DEVICE_ID_VIRPIL_STICK_MT_50CM2 0x4138
#define USB_DEVICE_ID_VIRPIL_STICK_MT_50CM3 0x838f
#define USB_DEVICE_ID_VIRPIL_THROTTLE_MT_50CM3 0x0197
#define USB_DEVICE_ID_VIRPIL_PEDALS_ACE 0x01f8


#define VPC_WAIT_MS 1
#define VPC_WAIT_MIN_US 600
#define VPC_WAIT_MAX_US 4000

#define VPC_SHIFT_MODES 8
#define VPC_SHIFT_MODE_1    0x01
#define VPC_SHIFT_MODE_2    0x02
#define VPC_SHIFT_MODE_3    0x04
#define VPC_SHIFT_MODE_4    0x08
#define VPC_SHIFT_MODE_5    0x10
#define VPC_SHIFT_MODE_DTNT 0x20
#define VPC_SHIFT_MODE_ZOOM 0x40
#define VPC_SHIFT_MODE_TRM  0x80

struct vpc_device {
    struct usb_device *usb_dev;
    struct mutex lock;
    unsigned char usb_interface_protocol;
    unsigned short usb_vid;
    unsigned short usb_pid;

    DECLARE_BITMAP(shift_mode, VPC_SHIFT_MODES);
};

#endif //VPCDEVICE_DRIVER_H
