#!/bin/bash
# File: src/rt1_drivers/scripts/setup_hokuyo.sh

# Function to check if user is in the dialout group
check_dialout_group() {
    if groups $USER | grep &>/dev/null '\bdialout\b'; then
        echo "User is already in the dialout group."
    else
        echo "Adding user to the dialout group..."
        sudo usermod -a -G dialout $USER
        echo "User added to dialout group. Please log out and log back in for changes to take effect."
    fi
}

# Function to set up udev rule for Hokuyo LiDAR
setup_udev_rule() {
    local rule_file="/etc/udev/rules.d/99-hokuyo.rules"
    if [ -f "$rule_file" ]; then
        echo "Udev rule for Hokuyo LiDAR already exists."
    else
        echo "Creating udev rule for Hokuyo LiDAR..."
        echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="15d1", ATTRS{idProduct}=="0000", SYMLINK+="hokuyo", MODE="0666", GROUP="dialout"' | sudo tee "$rule_file"
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        echo "Udev rule created and applied."
    fi
}

# Main script execution
echo "Setting up Hokuyo LiDAR..."
check_dialout_group
setup_udev_rule

echo "Hokuyo LiDAR setup complete. The device should now be accessible at /dev/hokuyo"
echo "If you've just been added to the dialout group, please log out and log back in for changes to take effect."