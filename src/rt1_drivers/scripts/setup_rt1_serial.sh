#!/usr/bin/env python3
# File: src/rt1_drivers/scripts/setup_rt1_serial.py

import os
import sys
import subprocess
import grp
import pwd
import re

def get_current_username():
    """Get the current username."""
    return os.getenv('USER') or pwd.getpwuid(os.getuid()).pw_name

def is_user_in_dialout():
    """Check if the current user is in the dialout group."""
    username = get_current_username()
    try:
        dialout_group = grp.getgrnam('dialout')
        return username in dialout_group.gr_mem
    except KeyError:
        print("Warning: dialout group not found")
        return False

def add_user_to_dialout():
    """Add the current user to the dialout group."""
    username = get_current_username()
    try:
        subprocess.run(['sudo', 'usermod', '-a', '-G', 'dialout', username], check=True)
        print(f"Added user {username} to dialout group.")
        print("Please log out and log back in for changes to take effect.")
    except subprocess.CalledProcessError as e:
        print(f"Error adding user to dialout group: {e}")
        sys.exit(1)

def setup_udev_rule():
    """Set up udev rule for RT1 serial communication."""
    rule_file = "/etc/udev/rules.d/99-rt1-serial.rules"
    # Updated rule with correct vendor ID (0403) and product ID (6001) for FTDI device
    rule_content = 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="rt1", MODE="0666", GROUP="dialout"\n'

    # Check if rule file already exists with correct content
    try:
        if os.path.exists(rule_file):
            with open(rule_file, 'r') as f:
                current_content = f.read()
                if current_content.strip() == rule_content.strip():
                    print("Udev rule for RT1 serial already exists and is correct.")
                    return
    except PermissionError:
        print(f"Error: Unable to read {rule_file}. Please run the script with sudo.")
        sys.exit(1)

    # Create or update the rule file
    try:
        with open(rule_file, 'w') as f:
            f.write(rule_content)
    except PermissionError:
        print(f"Error: Unable to write to {rule_file}. Please run the script with sudo.")
        sys.exit(1)

    # Reload udev rules
    try:
        subprocess.run(['sudo', 'udevadm', 'control', '--reload-rules'], check=True)
        subprocess.run(['sudo', 'udevadm', 'trigger'], check=True)
        print("Udev rule created and applied successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error reloading udev rules: {e}")
        sys.exit(1)

def main():
    """Main function to set up RT1 serial communication."""
    print("Setting up RT1 serial communication...")

    # Check if running with sudo
    if os.geteuid() != 0:
        print("This script requires sudo privileges.")
        print("Please run with: sudo python3 setup_rt1_serial.py")
        sys.exit(1)

    # Check and setup dialout group membership
    if not is_user_in_dialout():
        print("User is not in dialout group.")
        add_user_to_dialout()
    else:
        print("User is already in dialout group.")

    # Setup udev rule
    setup_udev_rule()

    print("\nRT1 serial setup complete.")
    print("The device should now be accessible at /dev/rt1")
    if not is_user_in_dialout():
        print("Remember to log out and log back in for group changes to take effect.")

if __name__ == "__main__":
    main()