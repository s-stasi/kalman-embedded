# kalman-embedded

A high-performance, header-only Kalman Filter library specifically designed for embedded systems (ARM Cortex-M, ESP32, etc.). It features both a lightweight 1D implementation and a robust 2D implementation with Sensor Fusion, optimized for redundant sensors like dual-potentiometers (APPS) or IMUs.

## Features

* **Header-only**: Easy integration, requires only `kalman_filter.h`.
* **Dual Mode**:
* **1D Scalar**: Ultra-lightweight, minimal CPU/RAM footprint.
* **2D Matrix**: State-space model (position/velocity) with Sensor Fusion support.

* **SIMD Ready**: Data structures are aligned to 16-byte boundaries to facilitate ARM CMSIS-DSP or NEON optimizations.
* **Zero Dynamic Allocation**: No `malloc` usage; completely static and deterministic memory management.

## Installation

Copy `kalman_filter.h` into your project source tree. To include the implementation, define `KALMAN_FILTER_IMPLEMENTATION` in **one** C file before including the header:

```c
#define KALMAN_FILTER_IMPLEMENTATION
#include "kalman_filter.h"

```

In other files where you need to access the filter types or functions, simply include the header:

```c
#include "kalman_filter.h"

```

## Usage Example: Dual APPS Sensor Fusion

This library is suitable for Formula Student APPS (Accelerator Pedal Position Sensor) plausibility checks. It handles inverse-slope sensors (e.g., Sensor 1: 0V-2.4V, Sensor 2: 5V-2.6V) using the 2D Sensor Fusion model to estimate position and velocity.

### Initialization

```c
// Define system matrices
// State Transition (F): Position = Position + Velocity * dt
float dt = 0.01f; // 10ms loop
float F[4] = {1.0f, dt, 0.0f, 1.0f};

// Observation Matrix (H): Maps state to sensor voltage
// Example slopes: Sens1 = 580*Pos, Sens2 = -570*Pos
float H[4] = {580.0f, 0.0f, -570.0f, 0.0f};

// Noise Matrices
float Q[4] = {0.001f, 0.0f, 0.0f, 0.001f}; // Process noise
float R[4] = {25.0f, 0.0f, 0.0f, 25.0f};   // Sensor noise (Variance)
float P[4] = {1.0f, 0.0f, 0.0f, 1.0f};     // Initial covariance

KalmanFilter2D_t k2d;

// Initialize with offsets for the sensors
Kalman2DInit(&k2d, F, H, Q, R, P, 80.0f, 4060.0f);

```

### Main Loop

```c
while(1) {
    // Acquire raw ADC values
    float z1 = Read_ADC(CH1);
    float z2 = Read_ADC(CH2);
    
    // Prediction Step
    Kalman2DPredict(&k2d);
    
    // Update Step (Fusion)
    Kalman2DUpdate(&k2d, z1, z2);
    
    // Retrieve estimated state
    float estimated_position = k2d.state.x[0];
    float estimated_velocity = k2d.state.x[1];
    
    // Use estimated_position for torque command
}

```

## Mathematical Model

The library implements the standard discrete Kalman Filter equations.

### Prediction Phase

Project the state ahead:

$$\hat{x}_{k} = F \hat{x}_{k-1}$$

Project the error covariance ahead:

$$P_{k} = F P_{k-1} F^T + Q$$

### Update Phase

Compute the Kalman Gain:

$$K_k = P_k H^T (H P_k H^T + R)^{-1}$$

Update the estimate via measurement :

$$\hat{x}_k = \hat{x}_k + K_k (z_k - H \hat{x}_k)$$

Update the error covariance:

$$P_k = (I - K_k H) P_k$$

## License

This project is licensed under the MIT License. See the LICENSE file for details.
