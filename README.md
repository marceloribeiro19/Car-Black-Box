# Black Box for Automotive Applications

## Overview

The ASM330LHB and BMI088 IMU are designed for high-performance automotive applications. This drivers, tailored for the Raspberry Pi 4, offer robust data logging and real-time orientation tracking capabilities, ensuring that vehicle movements are continuously monitored and stored. 

## Key Features

- **Comprehensive Data Storage**:
  - Capable of storing 15 minutes of vehicle orientation data.
  - Continuous data updates to ensure real-time accuracy.

- **Replay Mode**:
  - In the event of a car crash, the "Replay Mode" allows for precise replication of vehicle movements, aiding in accident analysis and reconstruction.

- **High Performance**:
  - Sensor fusion is achieved using the Madgwick filter for optimal Euler angles or, in this case, quaternion values.
  - Data can be further filtered with a Kalman filter to enhance accuracy and reliability.

- **Integration with Raspberry Pi 4**:
  - Optimized for seamless operation with the Raspberry Pi 4 with a custom Buildroot Linux Image, leveraging its processing power and storage capabilities.

## Technical Specifications

- **Sensor Type**: 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)
- **Data Logging**: Stores up to 15 minutes of orientation data.
- **Update Frequency**: Continuous real-time updates.
- **Replay Function**: Allows playback of stored data to replicate vehicle movements.
- **Sensor Fusion**: Madgwick filter for sensor fusion.
- **Data Filtering**: Optional Kalman filter for enhanced data accuracy.

## Usage

1. **Real-Time Tracking**:
   - Initiate the driver to start real-time tracking of vehicle orientation.
   - Data is continuously updated and stored in the Raspberry Pi 4's memory card.

2. **Replay Mode**:
   - In the event of a crash, activate "Replay Mode" to review and replicate the vehicle's movements leading up to the incident.
   - Utilize the recorded data for detailed analysis and accident reconstruction.

## Applications

- **Accident Analysis**: Provides critical data for reconstructing vehicle movements during crashes.
- **Performance Monitoring**: Continuous tracking of vehicle orientation for performance assessments.
- **Safety Systems**: Integration with automotive safety systems to enhance response mechanisms in critical situations.

## Conclusion

The ASM330LHB and BMI088 IMUs are a powerful tool, offering precise orientation tracking and robust data logging capabilities. Although these, when applyed to the automotive industry are not reliable, since neither contain an magnetometer to help measure the Z-axis rotation of the car effectively. However it was possible to complete the project with the available accelerometer and gyroscope on both IMUs by ignoring the Z-axis rotation of the veicule and only dealing with X and Y values.

## Some Demonstration videos are presented bellow:

https://github.com/user-attachments/assets/b1f0b5a4-201c-4d5a-a2d6-d7317bd221bb



https://github.com/user-attachments/assets/780a9504-2f6a-4e16-b68b-55b9c58bb7d3

