# OpenVPC project

This project aims to support virpil control devices by recreating the windows software in an open source driver and tools.

Currently the driver is a WIP for linux only. 

The shift tool is functionally complete and works on both Windows and Linux.

For the shift tool to work on linux, you need to set the permissions of the controllers to read write for your user. I created some udev rules that set all VPC devices that have a vendor ID of 0x3344 to the correct permissions.

## Device Support

The following is a list of devices that I have tested personally. It should work with any VPC device, however, as they use basic USB protocal functions. The only thing I wouldn't be comfortable saying this about would be the Firmware update tool (I haven't created it yet)

### Stick Bases
| Device                  |
|-------------------------|
| VPC MongoosT-50CM2 Base |
| VPC MongoosT-50CM3 Base |

### Throttles
| Device                      |
|-----------------------------|
| VPC MongoosT-50CM3 Throttle |

### Rudder Pedals
| Device                |
|-----------------------|
| VPC ACE Flight Pedals |

#### Determining the Device ID
VPC devices use a USB VID (Vendor ID) of `3344`. You can identify the USB PID (Product ID) by typing:

    lsusb | grep '3344:'

This will output something similar to this:

    Bus 003 Device 022: ID 3344:838f Leaguer Microelectronics (LME) L-VPC Stick MT-50CM3


---

