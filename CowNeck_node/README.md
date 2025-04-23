Bluetooth Low Energy Accelerometer Data Advertisement

 Overview

This application demonstrates collection and advertisement of accelerometer data over Bluetooth Low Energy (BLE) using the Zephyr RTOS. The system collects accelerometer data from an ST LIS2DH sensor and periodically transmits it using extended advertising features of Bluetooth 5.0.

 Key Features

- **Accelerometer Data Collection**: Captures X, Y, Z axis data from ST LIS2DH sensor
- **Extended Advertising**: Utilizes BLE extended advertising to transmit large chunks of sensor data
- **Data Chunking**: Splits data into multiple advertisement packets for transmission
- **Configurable Parameters**: Customizable sampling frequency and device identification

 Hardware Requirements

- nRF52 series development board (or compatible)
- ST LIS2DH accelerometer sensor
- Bluetooth 5.0 compatible receiver device

 Configuration Parameters

The following parameters can be configured in the code:

payload_len: 241 bytes - Total payload size (240 bytes for accelerometer data + 1 byte for device ID)
motion_idx_len: 80 - Number of motion data samples to collect before advertising
frequency: 500ms - Sampling period for accelerometer data collection
device_Id: 100 - Device identifier included in transmitted data

 Bluetooth Configuration

- **Advertisement Type**: Non-connectable, non-scannable extended advertising
- **Advertisement Address**: Random static address (configurable in code)
- **Advertisement Interval**: Approximately 3 seconds
- **Data Format**: Manufacturer-specific data format

 Data Structure

The accelerometer data is collected in a structured format:

```c
typedef struct {
  uint8_t x_lsb;
  uint8_t y_lsb;
  uint8_t z_lsb;
} motion_data;
```

 Operation Flow

1. Initialize BLE with a random static address
2. Create extended advertising set with specified parameters
3. Start accelerometer data collection at specified frequency (500ms)
4. After collecting `motion_idx_len` samples, package data into advertisement packets
5. Transmit advertisement packets
6. Repeat the collection-advertisement cycle

 Timing Considerations

- Accelerometer data is sampled every 500ms
- Advertisements are transmitted after collecting a full batch of samples
- Each advertisement event consists of a single extended advertisement

 Implementation Details

- Uses Zephyr's work queue system for asynchronous operation
- Employs timers for periodic data collection and advertising
- Splits large data payloads across multiple advertisement fields
- Scales raw accelerometer values to appropriate ranges

 Data Processing

Raw accelerometer data undergoes the following processing:
- Conversion from sensor values to scaled integers
- Rounding to nearest integer value
- Distribution across multiple advertisement data fields

 Limitations

- Maximum data payload is limited by BLE extended advertising specifications
- Only LSB values of acceleration data are transmitted to conserve bandwidth
- Single advertising event per data collection cycle

 Usage Notes

This code is suitable for applications requiring periodic transmission of motion data without establishing a connection, such as:
- Motion activity tracking
- Environmental monitoring
- Asset tracking
- Industrial sensor applications

 Development Environment

- Zephyr RTOS
- nRF SDK
- Bluetooth Core Specification 5.0 or later
