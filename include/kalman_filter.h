/*
 * kalman-embedded - Kalman filter library for embedded systems
 * Copyright (C) 2025 s-stasi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#define KALMAN_FILTER_VERSION "1.0.0"

// User should define the best allocator for their platform, default is malloc/free
#ifndef KALMAN_MALLOC
#include <stdlib.h>
#define KALMAN_MALLOC malloc
#define KALMAN_FREE free
#endif

#ifndef KALMAN_QUICKACCESS_CODE
#define KALMAN_QUICKACCESS_CODE(funct) funct
#endif

#include <stdint.h>
#include <string.h>
#include <math.h>

#define MATRIX_MATH_IMPLEMENTATION
#include "../lib/linear-algebra-embedded/include/matrix_math.h"

#define VECTOR_MATH_IMPLEMENTATION
#include "../lib/linear-algebra-embedded/include/vector_math.h"

typedef struct
{
  float q, r, p;
  float state;
} KalmanFilter1D_t;

typedef struct
{
  Matrix_t F; // Transition matrix
  Matrix_t H; // Observation matrix
  Matrix_t Q; // Process noise
  Matrix_t R; // Sensor noise
  Matrix_t P; // Error covariance
  Matrix_t K; // Kalman gain
  Matrix_t transpose_buffer;
  Matrix_t mult_buffer;
  Matrix_t S;

  Vector_t state;
  Vector_t state_buffer;
  Vector_t y;

  uint8_t num_states;
  uint8_t num_measurements;
} KalmanFilter_t;

#ifdef __cplusplus
extern "C"
{
#endif

  KALMAN_QUICKACCESS_CODE(void kalman_1DInit(KalmanFilter1D_t *kalman, float q, float r, float p));
  KALMAN_QUICKACCESS_CODE(void kalman_1DPredict(KalmanFilter1D_t *kalman));
  KALMAN_QUICKACCESS_CODE(float kalman_1DUpdate(KalmanFilter1D_t *kalman, float z));

  KALMAN_QUICKACCESS_CODE(void kalman_init(KalmanFilter_t *kalman, uint8_t num_states, uint8_t num_measurements, float *F_init, float *H_init, float *Q_init, float *R_init, float *P_init));
  KALMAN_QUICKACCESS_CODE(void kalman_predict(KalmanFilter_t *kalman));
  KALMAN_QUICKACCESS_CODE(void kalman_update(KalmanFilter_t *kalman, Vector_t *updates));
  KALMAN_QUICKACCESS_CODE(void kalman_free(KalmanFilter_t *kalman));

#ifdef __cplusplus
}
#endif

#endif