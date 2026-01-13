# LASER VINS IMU Filter

This package provides a **ROS 2 lifecycle node** responsible for conditioning and filtering IMU data before it is fed into Visual-Inertial Odometry (VIO) systems like VINS-Fusion.

## Overview

The main objective of this package is to improve the quality of raw IMU measurements by removing high-frequency noise and specific vibration frequencies (e.g., from motors) using digital filters.

### Key Functionalities
1.  **Signal Conditioning:** Applies configurable IIR (Infinite Impulse Response) and Notch filters to both accelerometer and gyroscope data.
2.  **Flexible Input Handling:** Can handle IMU data coming from a single unified topic or split between accelerometer and gyroscope topics (common in some Realsense configurations).
3.  **Lifecycle Management:** Implemented as a managed Lifecycle Node, allowing for controlled state transitions (configure, activate, deactivate).

## Provided Nodes

### 1. `vins_imu_filter`
-   **Description:** This node subscribes to raw IMU topics, applies the configured filters using the `laser_uav_lib` filtering library, and republishes the clean data.

-   **Subscribed Topics:**
    -   `imu_in` (`sensor_msgs/msg/Imu`):
        Unified IMU data input (used if `imu_data_united` is `true`).
    -   `gyro_in` (`sensor_msgs/msg/Imu`):
        Raw gyroscope data input (used if `imu_data_united` is `false`).
    -   `accel_in` (`sensor_msgs/msg/Imu`):
        Raw accelerometer data input (used if `imu_data_united` is `false`).

-   **Published Topics:**
    -   `imu_out` (`sensor_msgs/msg/Imu`):
        The filtered and time-synchronized IMU data ready for the VIO estimator.

-   **Configurable Parameters:**
    ```yaml
    vins_imu_filter:
      ros__parameters:
        # Input Data Configuration
        # Set to true if accel and gyro arrive on the same topic ('imu_in')
        # Set to false if they arrive on separate topics ('accel_in', 'gyro_in')
        imu_data_united: false

        # Accelerometer Filtering Configuration
        accelerometer:
          iir_filter:
            enable: true
            # Coefficients for the IIR filter (e.g., Butterworth low-pass)
            # a: Feedback coefficients, b: Feedforward coefficients
            a: [1.0, -0.5095]
            b: [0.2452, 0.2452]

          notch_filter:
            enable: false
            sample_rate: 1000 # Hz
            frequencies: [100, 200, 300] # Frequencies to attenuate (Hz)
            bandwidth: 100 # Stopband width (Hz)

        # Gyroscope Filtering Configuration
        gyro:
          iir_filter:
            enable: true
            a: [1.0, -0.7265]
            b: [0.1367, 0.1367]

          notch_filter:
            enable: false
            sample_rate: 1000 # Hz
            frequencies: [100, 200, 300] # Hz
            bandwidth: 100 # Hz
    ```