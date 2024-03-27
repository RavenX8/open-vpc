# PREFIX is used to prefix where the files will be installed under DESTDIR
PREFIX?=/usr
# UDEV_PREFIX is specifically a prefix for udev files
UDEV_PREFIX?=$(PREFIX)
# DESTDIR is used to install into a different root directory
DESTDIR?=/
# Specify the kernel directory to use
KERNELDIR?=/lib/modules/$(shell uname -r)/build
# Need the absolute directory do the driver directory to build kernel modules
DRIVERDIR?=$(shell pwd)/driver

# Where kernel drivers are going to be installed
MODULEDIR?=/lib/modules/$(shell uname -r)/kernel/drivers/hid

DKMS_NAME?=openvpc-driver
DKMS_VER?=0.1.0


# Build all target
all: driver

# Driver compilation
driver:
	@echo -e "\n::\033[32m Compiling OpenVPC kernel modules\033[0m"
	@echo "========================================"
	$(MAKE) -C $(KERNELDIR) M=$(DRIVERDIR) modules

driver_clean:
	@echo -e "\n::\033[32m Cleaning OpenVPC kernel modules\033[0m"
	@echo "========================================"
	$(MAKE) -C "$(KERNELDIR)" M="$(DRIVERDIR)" clean

# Clean target
clean: driver_clean

.PHONY: driver