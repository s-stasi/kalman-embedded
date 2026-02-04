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

#include <stdint.h>
#include <string.h>
#include <math.h>

#if defined(__GNUC__) || defined(__clang__)
#define ALIGN16 __attribute__((aligned(16)))
#else
#define ALIGN16
#endif

typedef struct {
  float q, r, p;
  float state;
} KalmanFilter1D_t;

typedef struct
{
  float x[2]; // State position and velocity
} KalmanState2D_t;

typedef struct
{
  ALIGN16 float F[4]; // Transition matrix (2x2)
  ALIGN16 float H[4]; // Observation matrix (2x2)
  ALIGN16 float Q[4]; // Process noise (2x2)
  ALIGN16 float R[4]; // Sensor noise (2x2)
  ALIGN16 float P[4]; // Error covariance (2x2)
  ALIGN16 float K[4]; // Kalman gain (2x2)

  KalmanState2D_t state;

  // Offset
  float offset[2];
} KalmanFilter2D_t;

#ifdef __cplusplus
extern "C"
{
#endif

  void Kalman1DInit(KalmanFilter1D_t *kalman, float q, float r, float p);
  void Kalman1DPredict(KalmanFilter1D_t *kalman);
  float Kalman1DUpdate(KalmanFilter1D_t *kalman, float z);

  void Kalman2DInit(KalmanFilter2D_t *kalman, float *F, float *H, float *Q, float *R, float *P, float offset1, float offset2);
  void Kalman2DPredict(KalmanFilter2D_t *kalman);
  void Kalman2DUpdate(KalmanFilter2D_t *kalman, float z1, float z2);
  void Kalman2DUpdateVector(KalmanFilter2D_t *kalman, float *measurements);

#ifdef __cplusplus
}
#endif

#endif // KALMAN_FILTER_H

#ifdef KALMAN_FILTER_IMPLEMENTATION

void Kalman1DInit(KalmanFilter1D_t *kalman, float q, float r, float p) {
  kalman->q = q;
  kalman->r = r;
  kalman->p = p;
  kalman->state = 0;
}

void Kalman1DPredict(KalmanFilter1D_t *kalman) {
  kalman->p = kalman->p + kalman->q;
}

float Kalman1DUpdate(KalmanFilter1D_t *kalman, float z) {
  float k = kalman->p / (kalman->p + kalman->r);
  kalman->state = kalman->state + k * (z - kalman->state);
  kalman->p = (1 - k) * kalman->p;

  return kalman->state;
}

void Kalman2DInit(KalmanFilter2D_t *kalman, float *F, float *H, float *Q, float *R, float *P, float offset1, float offset2)
{
  memcpy(kalman->F, F, sizeof(float) * 4);
  memcpy(kalman->H, H, sizeof(float) * 4);
  memcpy(kalman->Q, Q, sizeof(float) * 4);
  memcpy(kalman->R, R, sizeof(float) * 4);
  memcpy(kalman->P, P, sizeof(float) * 4);

  memset(&kalman->state, 0, sizeof(float) * 2);

  kalman->K[0] = 0;
  kalman->K[1] = 0;
  kalman->K[2] = 0;
  kalman->K[3] = 0;

  kalman->offset[0] = offset1;
  kalman->offset[1] = offset2;
}

void Kalman2DPredict(KalmanFilter2D_t *kalman)
{
  // x = F * x
  float x0 = kalman->F[0] * kalman->state.x[0] + kalman->F[1] * kalman->state.x[1];
  float x1 = kalman->F[2] * kalman->state.x[0] + kalman->F[3] * kalman->state.x[1];
  kalman->state.x[0] = x0;
  kalman->state.x[1] = x1;

  // P = F * P * F^T + Q
  float m0 = kalman->F[0] * kalman->P[0] + kalman->F[1] * kalman->P[2];
  float m1 = kalman->F[0] * kalman->P[1] + kalman->F[1] * kalman->P[3];
  float m2 = kalman->F[2] * kalman->P[0] + kalman->F[3] * kalman->P[2];
  float m3 = kalman->F[2] * kalman->P[1] + kalman->F[3] * kalman->P[3];

  // (F * P) * F^T + Q
  // [F0, F2, F1, F3]
  kalman->P[0] = m0 * kalman->F[0] + m1 * kalman->F[1] + kalman->Q[0];
  kalman->P[1] = m0 * kalman->F[2] + m1 * kalman->F[3] + kalman->Q[1];
  kalman->P[2] = m2 * kalman->F[0] + m3 * kalman->F[1] + kalman->Q[2];
  kalman->P[3] = m2 * kalman->F[2] + m3 * kalman->F[3] + kalman->Q[3];
}

void Kalman2DUpdate(KalmanFilter2D_t *kalman, float z1, float z2) {
    // 1. Innovazione y = z - (H * x + offset)
    float hxo1 = kalman->H[0] * kalman->state.x[0] + kalman->H[1] * kalman->state.x[1] + kalman->offset[0];
    float hxo2 = kalman->H[2] * kalman->state.x[0] + kalman->H[3] * kalman->state.x[1] + kalman->offset[1];
    float y1 = z1 - hxo1;
    float y2 = z2 - hxo2;

    // 2. S = H * P * H^T + R
    // Passo 1: M = H * P (Matrice temporanea 2x2)
    float m0 = kalman->H[0] * kalman->P[0] + kalman->H[1] * kalman->P[2];
    float m1 = kalman->H[0] * kalman->P[1] + kalman->H[1] * kalman->P[3];
    float m2 = kalman->H[2] * kalman->P[0] + kalman->H[3] * kalman->P[2];
    float m3 = kalman->H[2] * kalman->P[1] + kalman->H[3] * kalman->P[3];

    // Passo 2: S = M * H^T + R
    float s0 = m0 * kalman->H[0] + m1 * kalman->H[1] + kalman->R[0];
    float s1 = m0 * kalman->H[2] + m1 * kalman->H[3] + kalman->R[1];
    float s2 = m2 * kalman->H[0] + m3 * kalman->H[1] + kalman->R[2];
    float s3 = m2 * kalman->H[2] + m3 * kalman->H[3] + kalman->R[3];

    // 3. Inversione di S
    float detS = s0 * s3 - s1 * s2;
    if (detS > -1e-6f && detS < 1e-6f) return; // Protezione divisione per zero
    float invDet = 1.0f / detS;

    float invS0 =  s3 * invDet;
    float invS1 = -s1 * invDet;
    float invS2 = -s2 * invDet;
    float invS3 =  s0 * invDet;

    // 4. Guadagno di Kalman K = P * H^T * S^-1
    // Passo 1: N = P * H^T
    float n0 = kalman->P[0] * kalman->H[0] + kalman->P[1] * kalman->H[1];
    float n1 = kalman->P[0] * kalman->H[2] + kalman->P[1] * kalman->H[3];
    float n2 = kalman->P[2] * kalman->H[0] + kalman->P[3] * kalman->H[1];
    float n3 = kalman->P[2] * kalman->H[2] + kalman->P[3] * kalman->H[3];

    // Passo 2: K = N * S^-1
    kalman->K[0] = n0 * invS0 + n1 * invS2;
    kalman->K[1] = n0 * invS1 + n1 * invS3;
    kalman->K[2] = n2 * invS0 + n3 * invS2;
    kalman->K[3] = n2 * invS1 + n3 * invS3;

    // 5. Aggiornamento Stato x = x + K * y
    kalman->state.x[0] += kalman->K[0] * y1 + kalman->K[1] * y2;
    kalman->state.x[1] += kalman->K[2] * y1 + kalman->K[3] * y2;

    // 6. Aggiornamento Covarianza P = (I - K * H) * P
    // IKH = (I - K * H)
    float ikh0 = 1.0f - (kalman->K[0] * kalman->H[0] + kalman->K[1] * kalman->H[2]);
    float ikh1 = - (kalman->K[0] * kalman->H[1] + kalman->K[1] * kalman->H[3]);
    float ikh2 = - (kalman->K[2] * kalman->H[0] + kalman->K[3] * kalman->H[2]);
    float ikh3 = 1.0f - (kalman->K[2] * kalman->H[1] + kalman->K[3] * kalman->H[3]);

    // P = IKH * P
    float p0_new = ikh0 * kalman->P[0] + ikh1 * kalman->P[2];
    float p1_new = ikh0 * kalman->P[1] + ikh1 * kalman->P[3];
    float p2_new = ikh2 * kalman->P[0] + ikh3 * kalman->P[2];
    float p3_new = ikh2 * kalman->P[1] + ikh3 * kalman->P[3];

    kalman->P[0] = p0_new; kalman->P[1] = p1_new;
    kalman->P[2] = p2_new; kalman->P[3] = p3_new;
}

void Kalman2DUpdateVector(KalmanFilter2D_t *kalman, float *measurements) {
  Kalman2DUpdate(kalman, measurements[0], measurements[1]);
}

#endif // KALMAN_FILTER_IMPLEMENTATION