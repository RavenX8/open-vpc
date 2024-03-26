#!/usr/bin/python3


import argparse
import glob
import os

shift_mode = glob.glob('/sys/bus/hid/drivers/vpcdevice/0*/shift_mode')


def write_binary(driver_path, device_file, payload):
    with open(os.path.join(driver_path, device_file), 'wb') as open_file:
        open_file.write(payload)


def read_string(driver_path, device_file):
    with open(os.path.join(driver_path, device_file), 'r') as open_file:
        return open_file.read().rstrip('\n')


def write_string(driver_path, device_file, payload):
    with open(os.path.join(driver_path, device_file), 'w') as open_file:
        open_file.write(payload)


def find_devices(vid):
    driver_paths = glob.glob(os.path.join('/sys/bus/hid/drivers/vpcdevice', '*:{0:04X}:.*'.format(vid)))

    for driver_path in driver_paths:
        device_type_path = os.path.join(driver_path, 'device_type')

        if os.path.exists(device_type_path):
            yield driver_path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')

    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()

    found_device = False

    for index, driver_path in enumerate(find_devices(0x3344), start=1):
        found_device = True

        print("VPC Device {0}\n".format(index))

        print("Driver version: {0}".format(read_string(driver_path, 'version')))
        print("Driver firmware version: {0}".format(read_string(driver_path, 'firmware_version')))
        print("Device type: {0}".format(read_string(driver_path, 'device_type')))
        print("Device mode: {0}".format(read_string(driver_path, 'shift_mode')))

        print("Finished")

    if not found_device:
        print("No VPC Device found")
