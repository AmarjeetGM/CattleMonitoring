# BLE Observer with AWS IoT Integration

## Overview
This project implements a Bluetooth Low Energy (BLE) observer device based on Zephyr RTOS that collects advertising data from configured BLE devices, processes it, stores it in flash memory, and forwards it to AWS IoT. The system is designed for IoT sensor networks where data is collected locally and then transmitted to the cloud for analysis and storage.

## Features
- BLE passive scanning with filter accept list support
- Flash memory management for data buffering
- AWS IoT integration with shadow updates
- GSM/cellular connectivity for remote deployments
- SNTP time synchronization
- Duplicate packet detection and filtering
- LED indication for packet reception and status
- Circular buffer implementation in flash for efficient data handling

## Configuration
The following configuration parameters can be adjusted in your project's configuration:

### BLE Configuration
- `CONFIG_BLE_RECEIVING_PAYLOAD`: Size of BLE payload buffer
- `CONFIG_DEVICE_ID`: Gateway device identifier
- `CONFIG_GROUP_ID`: Group identifier for the network
- `CONFIG_BLE_ADD`: BLE address to filter for (accept list)

### AWS IoT Configuration
- `CONFIG_AWS_IOT_SAMPLE_PUBLICATION_INTERVAL_SECONDS`: Interval between AWS IoT publications
- `CONFIG_AWS_IOT_SAMPLE_JSON_MESSAGE_SIZE_MAX`: Maximum JSON message size
- `CONFIG_AWS_QOS`: Quality of Service level for MQTT messages

### Flash Memory Configuration
- `CONFIG_FLASH_WRITE_ADD`: Flash address for storing write index
- Flash limit and sector size (4096 bytes)

## Memory Management
The system implements a circular buffer in flash memory:
- Data is stored in fixed-size chunks of `CONFIG_BLE_RECEIVING_PAYLOAD` bytes
- Flash sectors are erased as needed (sector size: 4096 bytes)
- Write and read indexes are tracked to manage data flow
- When flash limit is reached, writing continues from the beginning (address 0x102000)
- A mutex protects flash operations to prevent concurrent access

## Data Flow
1. **Data Collection**:
   - BLE packets are received through the scan callback
   - Duplicate packets are detected and filtered out
   - Device ID, Group ID, RSSI, and timestamp are appended to the data
   - Data is written to flash memory at `writeIndex`
   - The write index is updated and stored in flash

2. **Data Transmission**:
   - The AWS IoT client periodically checks if there's enough data to send
   - If data is available (`writeIndex - readIndex >= publishPayloadSize` or `memFilled` is true):
     - Data is read from flash memory at `readIndex`
     - Data is formatted as JSON
     - JSON message is sent to AWS IoT Shadow
     - `readIndex` is updated for the next read

## Time Synchronization
The system periodically synchronizes time using SNTP protocol (server: "0.pool.ntp.org"), which is used to timestamp collected data packets.

## Error Handling
- Network connectivity errors trigger reconnection attempts
- Fatal errors cause system reboot
- Flash read/write failures are logged and reported

## Thread Management
The system uses Zephyr's work queues and threads:
- Observer thread: Priority 7, 4096 stack size
- Shadow update work: Scheduled periodically to publish data
- Connect work: Manages AWS IoT connectivity
- Mutex protection for flash operations

## Custom Modules
- `observer.h`: Main BLE scanning functionality
- `aws.h`: AWS integration for data transmission
- `gsm.h`: GSM communication module

## Logging
The application uses Zephyr's logging system with configurable log levels:
- `CONFIG_AWS_IOT_SAMPLE_LOG_LEVEL`: Controls AWS IoT client logging verbosity
- Console output provides detailed debug information

## AWS IoT Integration
The system connects to AWS IoT using MQTT protocol and publishes data to device shadows. The data is formatted as JSON arrays containing the sensor readings collected from BLE advertisements.

## Power Management
To optimize for battery-powered operation, the system:
- Uses passive BLE scanning
- Schedules data transmissions at configurable intervals
- Only erases flash sectors when necessary
