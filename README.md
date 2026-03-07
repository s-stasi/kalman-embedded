# kalman-embedded

A high-performance Kalman Filter library specifically designed for embedded systems (ARM Cortex-M, ESP32, etc.). It features both a lightweight 1D implementation and a robust N-dimensional implementation with Sensor Fusion, relying on a custom, bare-metal matrix/vector math library.

## Features

* **Standard C Architecture**: Clean separation between interface (`.h`) and implementation (`.c`) for scalable projects.
* **Dual Mode**:
  * **1D Scalar**: Ultra-lightweight, minimal CPU/RAM footprint.
  * **N-Dimensional Matrix**: State-space model capable of handling multiple states and redundant sensors dynamically.
* **Real-time Safe**: Zero dynamic allocation during the control loop. `malloc` (or your custom allocator) is strictly used only once during the initialization phase (`kalman_init`).
* **Memory Aliasing Protection**: Smart internal buffer management to strictly avoid memory aliasing issues during matrix operations.

## Integration

The library uses a standard structure. To use it in your project:

1. Add the `include/` and `lib/linear-algebra-embedded/include/` directories to your compiler's include path.
2. Compile and link the source files:
   * `src/kalman_filter.c`
   * `lib/linear-algebra-embedded/src/matrix_math.c`
   * `lib/linear-algebra-embedded/src/vector_math.c`

```c
#include "kalman_filter.h"

```

## Usage Example (N-Dimensional Filter)

Example of estimating Position and Velocity ($N=2$) using a single noisy position sensor ($M=1$).

### Initialization

```c
#include "kalman_filter.h"

// Define system parameters
uint8_t num_states = 2;       // Position, Velocity
uint8_t num_measurements = 1; // Position sensor only
float dt = 0.01f;             // 10ms loop

// State Transition (F): Position = Position + Velocity * dt
float F[4] = {1.0f, dt, 
              0.0f, 1.0f};

// Observation Matrix (H): Maps state to sensor reading
float H[2] = {1.0f, 0.0f};

// Noise Matrices
float Q[4] = {0.01f, 0.0f, 
              0.0f, 0.01f}; // Process noise
float R[1] = {4.0f};        // Sensor noise (Variance)
float P[4] = {100.0f, 0.0f, 
              0.0f, 100.0f}; // Initial covariance

KalmanFilter_t kf;

// Initialize the filter (allocates required buffers)
kalman_init(&kf, num_states, num_measurements, F, H, Q, R, P);

```

### Main Loop

```c
// Allocate a vector for our measurements
float sensor_data[1];
Vector_t z = {
    .length = num_measurements,
    .data = sensor_data
};

while(1) {
    // 1. Acquire raw sensor values
    z.data[0] = Read_Sensor();
    
    // 2. Prediction Step
    kalman_predict(&kf);
    
    // 3. Update Step
    kalman_update(&kf, &z);
    
    // 4. Retrieve estimated state
    float estimated_position = kf.state.data[0];
    float estimated_velocity = kf.state.data[1];
    
    // Use estimates...
}

// If the filter is destroyed, free the memory
// kalman_free(&kf);

```

## Mathematical Model

The library implements the standard discrete Kalman Filter equations.

### Prediction Phase

Project the state ahead:

$$x_{k} = F x_{k-1}$$

Project the error covariance ahead:

$$P_{k} = F P_{k-1} F^T + Q$$

### Update Phase

Compute the Kalman Gain:

$$K_k = P_k H^T (H P_k H^T + R)^{-1}$$

Update the estimate via measurement:

$$x_k = x_k + K_k (z_k - H x_k)$$

Update the error covariance:

$$P_k = (I - K_k H) P_k$$

## License

This project is licensed under the MIT License. See the LICENSE file for details.
