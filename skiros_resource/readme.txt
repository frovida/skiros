Device Manager: skiros_mng_device
Last ReadME update: 10/04/2014

Overview: The device manager creates the HW abstraction for the primitive manager.

Note: It must be populated with plugins. To add a plugin follow the readME in skiros_lib/skiros_lib_device

Files description:

skiros_dev_config.xml - First template of the configuration file

src/
skiros_mng_device_node.cpp - Main
configuration_handler.cpp - Function to read/modify the configuration settings

include/skiros_mng_device/

base_arm_proxy.h - Define the structure of an arm's proxy
Standard commands:
move_ptp
move_lin

base_gripper_proxy.h: Define the structure of a gripper's proxy
Standard commands:
open
close

