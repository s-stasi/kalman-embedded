#include <stdio.h>
#define KALMAN_FILTER_IMPLEMENTATION
#include "../kalman_filter.h"

int main() {
    /* --- DATI DI TEST (Scenario: Pedale al 20% con rumore) --- */
    float raw_acc1[] = {196.0f, 210.0f, 190.0f, 205.0f, 198.0f};
    float raw_acc2[] = {3946.0f, 3930.0f, 3960.0f, 3940.0f, 3950.0f};
    int steps = 5;

    /* --- CONFIGURAZIONE FILTRI 1D --- */
    KalmanFilter1D_t k1d_acc1;
    KalmanFilter1D_t k1d_acc2;
    kalman_1DInit(&k1d_acc1, 0.1f, 25.0f, 1.0f);
    kalman_1DInit(&k1d_acc2, 0.1f, 25.0f, 1.0f);
    k1d_acc1.state = 196.0f; // Inizializzo al primo valore
    k1d_acc2.state = 3946.0f;

    /* --- CONFIGURAZIONE FILTRO 2D --- */
    KalmanFilter2D_t k2d;
    float dt = 0.01f;
    float F[4] = {1.0f, dt, 0.0f, 1.0f};
    float H[4] = {580.0f, 0.0f, -570.0f, 0.0f}; // Pendenze mappate su 0.0-1.0
    float Q[4] = {0.001f, 0.0f, 0.0f, 0.001f};
    float R[4] = {25.0f, 0.0f, 0.0f, 25.0f};
    float P[4] = {1.0f, 0.0f, 0.0f, 1.0f};
    
    Kalman2DInit(&k2d, F, H, Q, R, P, 80.0f, 4060.0f);
    k2d.state.x[0] = 0.2f; // Inizializzo al 20%

    printf("Test Comparison: 1D vs 2D Sensor Fusion\n");
    printf("------------------------------------------------------------------\n");
    printf("Step | Raw1  | Raw2  | 1D-Acc1 | 1D-Acc2 | 2D-Pos(%%) | 2D-Vel\n");
    printf("------------------------------------------------------------------\n");

    for (int i = 0; i < steps; i++) {
        /* Update 1D */
        kalman_1DPredict(&k1d_acc1);
        kalman_1DUpdate(&k1d_acc1, raw_acc1[i]);
        kalman_1DPredict(&k1d_acc2);
        kalman_1DUpdate(&k1d_acc2, raw_acc2[i]);

        /* Update 2D */
        Kalman2DPredict(&k2d);
        Kalman2DUpdate(&k2d, raw_acc1[i], raw_acc2[i]);

        printf(" %d   | %.0f | %.0f | %7.2f | %7.2f | %8.2f%% | %8.4f\n",
               i, raw_acc1[i], raw_acc2[i], 
               k1d_acc1.state, k1d_acc2.state,
               k2d.state.x[0] * 100.0f, k2d.state.x[1]);
    }

    return 0;
}