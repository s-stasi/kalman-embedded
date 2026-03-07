#include "../include/kalman_filter.h"

void kalman_1DInit(KalmanFilter1D_t *kalman, float q, float r, float p)
{
  kalman->q = q;
  kalman->r = r;
  kalman->p = p;
  kalman->state = 0;
}

void kalman_1DPredict(KalmanFilter1D_t *kalman)
{
  kalman->p = kalman->p + kalman->q;
}

float kalman_1DUpdate(KalmanFilter1D_t *kalman, float z)
{
  float k = kalman->p / (kalman->p + kalman->r);
  kalman->state = kalman->state + k * (z - kalman->state);
  kalman->p = (1 - k) * kalman->p;

  return kalman->state;
}

void kalman_init(KalmanFilter_t *kalman, uint8_t num_states, uint8_t num_measurements,
                float *F_init, float *H_init, float *Q_init, float *R_init, float *P_init)
{
  kalman->num_states = num_states;
  kalman->num_measurements = num_measurements;

  // Init state vector (N)
  kalman->state.length = num_states;
  kalman->state.data = (float *)KALMAN_MALLOC(num_states * sizeof(float));
  memset(kalman->state.data, 0, num_states * sizeof(float));

  kalman->state_buffer.length = num_states;
  kalman->state_buffer.data = (float *)KALMAN_MALLOC(num_states * sizeof(float));
  memset(kalman->state_buffer.data, 0, num_states * sizeof(float));

  kalman->y.length = num_measurements;
  kalman->y.data = (float *)KALMAN_MALLOC(num_measurements * sizeof(float));
  memset(kalman->y.data, 0, num_measurements * sizeof(float));

  // Init state matrices (N x N)
  uint16_t n_sq_bytes = num_states * num_states * sizeof(float);
  uint16_t n_max_bytes = 0;
  if (num_measurements >= num_states) {
    n_max_bytes = num_measurements * num_measurements * sizeof(float);
    kalman->transpose_buffer.rows = num_measurements;
    kalman->transpose_buffer.cols = num_measurements;
  } else {
    n_max_bytes = num_states * num_states * sizeof(float);
    kalman->transpose_buffer.rows = num_states;
    kalman->transpose_buffer.cols = num_states;
  }

  kalman->F.rows = num_states;
  kalman->F.cols = num_states;
  kalman->F.data = (float *)KALMAN_MALLOC(n_sq_bytes);
  memcpy(kalman->F.data, F_init, n_sq_bytes);

  kalman->transpose_buffer.data = (float *)KALMAN_MALLOC(n_max_bytes);
  matrix_set_zero(&kalman->transpose_buffer);

  kalman->P.rows = num_states;
  kalman->P.cols = num_states;
  kalman->P.data = (float *)KALMAN_MALLOC(n_sq_bytes);
  memcpy(kalman->P.data, P_init, n_sq_bytes);

  kalman->mult_buffer.rows = num_states;
  kalman->mult_buffer.cols = num_states;
  kalman->mult_buffer.data = (float *)KALMAN_MALLOC(n_max_bytes);
  matrix_set_zero(&kalman->mult_buffer);

  kalman->Q.rows = num_states;
  kalman->Q.cols = num_states;
  kalman->Q.data = (float *)KALMAN_MALLOC(n_sq_bytes);
  memcpy(kalman->Q.data, Q_init, n_sq_bytes);

  uint16_t n_hk_bytes = num_measurements * num_states * sizeof(float);
  uint16_t n_r_bytes = num_measurements * num_measurements * sizeof(float);

  kalman->H.rows = num_measurements;
  kalman->H.cols = num_states;
  kalman->H.data = (float *)KALMAN_MALLOC(n_hk_bytes);
  memcpy(kalman->H.data, H_init, n_hk_bytes);

  kalman->R.rows = num_measurements;
  kalman->R.cols = num_measurements;
  kalman->R.data = (float *)KALMAN_MALLOC(n_r_bytes);
  memcpy(kalman->R.data, R_init, n_r_bytes);
  
  kalman->K.rows = num_states;
  kalman->K.cols = num_measurements;
  kalman->K.data = (float *)KALMAN_MALLOC(n_hk_bytes);
  matrix_set_zero(&kalman->K);
  
  kalman->S.rows = num_measurements;
  kalman->S.cols = num_measurements;
  kalman->S.data = (float *)KALMAN_MALLOC(n_r_bytes);
  matrix_set_zero(&kalman->S);
}

void kalman_predict(KalmanFilter_t *kalman)
{
  // x = F * x
  matrix_multiply_vector(&kalman->F, &kalman->state, &kalman->state_buffer);
  vector_copy(&kalman->state_buffer, &kalman->state);

  // P = F * P * F^T + Q
  kalman->transpose_buffer.rows = kalman->num_states;
  kalman->transpose_buffer.cols = kalman->num_states;
  matrix_transpose(&kalman->F, &kalman->transpose_buffer);
  matrix_multiply(&kalman->F, &kalman->P, &kalman->mult_buffer);
  matrix_multiply(&kalman->mult_buffer, &kalman->transpose_buffer, &kalman->P);
  matrix_add(&kalman->P, &kalman->Q, &kalman->mult_buffer);
  matrix_copy(&kalman->mult_buffer, &kalman->P);
}

void kalman_update(KalmanFilter_t *kalman, Vector_t *updates){
  // 1. Innovazione y = z - (H * x)
  Vector_t vector_buffer = {
    .length = kalman->H.rows,
    .data = (float *)kalman->mult_buffer.data
  };

  matrix_multiply_vector(&kalman->H, &kalman->state, &vector_buffer);
  vector_subtract(updates, &vector_buffer, &kalman->y);
  
  // 2. S = H * P * H^T + R
  kalman->transpose_buffer.rows = kalman->num_states;
  kalman->transpose_buffer.cols = kalman->num_measurements;
  // M = H * P (Matrice temporanea 2x2)
  matrix_multiply(&kalman->H, &kalman->P, &kalman->mult_buffer);
  // H^T
  matrix_transpose(&kalman->H, &kalman->transpose_buffer);
  // M * H^T
  matrix_multiply(&kalman->mult_buffer, &kalman->transpose_buffer, &kalman->S);
  // S = S + R
  kalman->mult_buffer.rows = kalman->S.rows;
  kalman->mult_buffer.cols = kalman->S.cols;
  matrix_add(&kalman->S, &kalman->R, &kalman->mult_buffer);
  matrix_copy(&kalman->mult_buffer, &kalman->S);

  // K = P * H^T * S^-1
  // H^T (N x M)
  kalman->transpose_buffer.rows = kalman->num_states;
  kalman->transpose_buffer.cols = kalman->num_measurements;
  matrix_transpose(&kalman->H, &kalman->transpose_buffer);

  // P * H^T
  matrix_multiply(&kalman->P, &kalman->transpose_buffer, &kalman->K);

  // S^-1
  kalman->transpose_buffer.rows = kalman->num_measurements;
  kalman->transpose_buffer.cols = kalman->num_measurements;
  kalman->mult_buffer.rows = kalman->num_measurements;
  kalman->mult_buffer.cols = kalman->num_measurements;
  matrix_inverse(&kalman->S, &kalman->transpose_buffer, &kalman->mult_buffer);

  // K = (P * H^T) * S^-1
  kalman->mult_buffer.rows = kalman->num_states;
  kalman->mult_buffer.cols = kalman->num_measurements;
  matrix_multiply(&kalman->K, &kalman->transpose_buffer, &kalman->mult_buffer);
  matrix_copy(&kalman->mult_buffer, &kalman->K);

  // x = x + K * y
  vector_buffer.length = kalman->num_states;
  matrix_multiply_vector(&kalman->K, &kalman->y, &vector_buffer);
  vector_add(&kalman->state, &vector_buffer, &kalman->state_buffer);
  vector_copy(&kalman->state_buffer, &kalman->state);

  // P = (I - K * H) * P --> P = P - (K * H * P)
  // H * P (Result: M x N)
  kalman->mult_buffer.rows = kalman->num_measurements;
  kalman->mult_buffer.cols = kalman->num_states;
  matrix_multiply(&kalman->H, &kalman->P, &kalman->mult_buffer);

  // K * (H * P) (Result: N x N)
  kalman->transpose_buffer.rows = kalman->num_states;
  kalman->transpose_buffer.cols = kalman->num_states;
  matrix_multiply(&kalman->K, &kalman->mult_buffer, &kalman->transpose_buffer);

  // P - [K * (H * P)] (Result: N x N)
  kalman->mult_buffer.rows = kalman->num_states;
  kalman->mult_buffer.cols = kalman->num_states;
  matrix_subtract(&kalman->P, &kalman->transpose_buffer, &kalman->mult_buffer);
  matrix_copy(&kalman->mult_buffer, &kalman->P);
}

void kalman_free(KalmanFilter_t *kalman){
  if (!kalman) return;

  // Libera i vettori
  KALMAN_FREE(kalman->state.data);
  KALMAN_FREE(kalman->state_buffer.data);
  KALMAN_FREE(kalman->y.data);

  // Libera le matrici N x N e i buffer temporanei
  KALMAN_FREE(kalman->F.data);
  KALMAN_FREE(kalman->P.data);
  KALMAN_FREE(kalman->Q.data);
  KALMAN_FREE(kalman->transpose_buffer.data);
  KALMAN_FREE(kalman->mult_buffer.data);

  // Libera le matrici M x N, M x M e N x M
  KALMAN_FREE(kalman->H.data);
  KALMAN_FREE(kalman->R.data);
  KALMAN_FREE(kalman->K.data);
  KALMAN_FREE(kalman->S.data);
}