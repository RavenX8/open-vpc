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
all: tools

# Driver compilation
driver:
	@echo -e "\n::\033[32m Compiling OpenVPC kernel modules\033[0m"
	@echo "========================================"
	$(MAKE) -C $(KERNELDIR) M=$(DRIVERDIR) modules

driver_clean:
	@echo -e "\n::\033[32m Cleaning OpenVPC kernel modules\033[0m"
	@echo "========================================"
	$(MAKE) -C "$(KERNELDIR)" M="$(DRIVERDIR)" clean

tools:
	@echo -e "\n::\033[32m Compiling OpenVPC tools\033[0m"
	@echo "========================================"
	$(MAKE) -C "$(shell pwd)/tools"

tools_clean:
	@echo -e "\n::\033[32m Cleaning OpenVPC tools\033[0m"
	@echo "========================================"
	$(MAKE) -C "$(shell pwd)/tools" clean

# Clean target
clean: driver_clean tools_clean

.PHONY: driver tools
