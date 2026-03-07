#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define MATRIX_MATH_IMPLEMENTATION
#define VECTOR_MATH_IMPLEMENTATION
#define KALMAN_FILTER_IMPLEMENTATION
#include "../include/kalman_filter.h"

// Helper function to generate pseudo-random noise between -range and +range
float generate_noise(float range)
{
  return (((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f) * range;
}

int main()
{
  // Initialize random seed
  srand((unsigned int)time(NULL));

  KalmanFilter_t kf;
  uint8_t num_states = 2;       // Position, Velocity
  uint8_t num_measurements = 1; // Only position sensor

  float dt = 0.1f; // Time step (10 Hz)

  // 1. Transition Matrix F (Kinematic model: p = p + v*dt, v = v)
  float F_init[4] = {
      1.0f, dt,
      0.0f, 1.0f};

  // 2. Observation Matrix H (We only measure position, so 1 for pos, 0 for vel)
  float H_init[2] = {
      1.0f, 0.0f};

  // 3. Process Noise Covariance Q (Small uncertainty in our kinematic model)
  float Q_init[4] = {
      0.01f, 0.0f,
      0.0f, 0.01f};

  // 4. Measurement Noise Covariance R (Variance of our noisy sensor)
  // We expect noise around +/- 2.0 meters, so variance is roughly 4.0
  float R_init[1] = {
      4.0f};

  // 5. Initial Error Covariance P (We don't trust our initial state much)
  float P_init[4] = {
      1.0f, 0.0f,
      0.0f, 1.0f};

  // Initialize the filter
  kalman_init(&kf, num_states, num_measurements, F_init, H_init, Q_init, R_init, P_init);

  // Simulation variables
  float true_position = 0.0f;
  float true_velocity = 5.0f; // Moving at 5 m/s

  // Allocate a vector for our measurements
  Vector_t z;
  z.length = 1;
  z.data = (float *)malloc(1 * sizeof(float));

  printf("Time,True_Position,Noisy_Measurement,Estimated_Position,Estimated_Velocity\n");

  // Run simulation for 50 steps
  for (int step = 0; step < 50; step++)
  {
    float time_sec = step * dt;

    // --- 1. SIMULATE THE REAL WORLD ---
    true_position += true_velocity * dt;

    // Sensor reading: true position + random noise (+/- 2.0m)
    float sensor_noise = generate_noise(2.0f);
    z.data[0] = true_position + sensor_noise;

    // --- 2. KALMAN FILTER ALGORITHM ---
    kalman_predict(&kf);
    kalman_update(&kf, &z);

    // --- 3. LOG RESULTS ---
    printf("%.1f,%.3f,%.3f,%.3f,%.3f\n",
           time_sec,
           true_position,
           z.data[0],
           kf.state.data[0], // Estimated Position
           kf.state.data[1]  // Estimated Velocity
    );
  }

  // Clean up memory
  free(z.data);
  kalman_free(&kf);

  return 0;
}