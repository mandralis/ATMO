#!/bin/bash

# Search for Atmel Corp. in lsusb output
results=$(lsusb | grep "Atmel Corp. The Micro")

# Check if the result is not empty
if [ -n "$results" ]; then
    # Initialize arrays to store bus and device numbers
    bus_numbers=()
    device_numbers=()

    # Loop through each result
    while read -r result; do
        # Extract the bus and device numbers using awk
        bus_number=$(echo "$result" | awk '{print $2}')
        device_number=$(echo "$result" | awk '{print $4}' | tr -d ':')

        # Store bus and device numbers in arrays
        bus_numbers+=("$bus_number")
        device_numbers+=("$device_number")

        # Print the bus and device numbers
        echo "Bus Number: $bus_number"
        echo "Device Number: $device_number"
        echo "---"
    done <<< "$results"

    # Example: Access the first bus and device numbers from the arrays
    first_bus="${bus_numbers[0]}"
    first_device="${device_numbers[0]}"

    echo "First Bus Number: $first_bus"
    echo "First Device Number: $first_device"

else
    echo "Atmel Corp. devices not found in lsusb output."
fi

# Reset both roboclaws
sudo ./usbreset /dev/bus/usb/${bus_numbers[0]}/${device_numbers[0]}
sudo ./usbreset /dev/bus/usb/${bus_numbers[1]}/${device_numbers[1]}
