# OpenVPC project

This project aims to support virpil control devices by recreating the windows software in an open source driver and tools.

Currently the driver is a WIP for linux only. 

The shift tool is functionally complete and works on both Windows and Linux.

## Device Support

The following is a list of devices that I have tested personally.

### Stick Bases
| Device                  | USB VID:PID |
|-------------------------|-------------|
| VPC MongoosT-50CM2 Base | 3344:4138   |
| VPC MongoosT-50CM3 Base | 3344:838f   |

### Throttles
| Device                      | USB VID:PID |
|-----------------------------|-------------|
| VPC MongoosT-50CM3 Throttle | 3344:0197   |

### Rudder Pedals
| Device                | USB VID:PID |
|-----------------------|-------------|
| VPC ACE Flight Pedals | 3344:01f8   |

#### Determining the Device ID
VPC devices use a USB VID (Vendor ID) of `3344`. You can identify the USB PID (Product ID) by typing:

    lsusb | grep '3344:'

This will output something similar to this:

    Bus 003 Device 022: ID 3344:838f Leaguer Microelectronics (LME) L-VPC Stick MT-50CM3


---

