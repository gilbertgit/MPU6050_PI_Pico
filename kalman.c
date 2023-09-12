
#include "kalman.h"

void kalman_init(Kalman* kalman) {
  kalman->Q_angle = 0.001;
  kalman->Q_bias = 0.003;
  kalman->R_measure = 0.03;  
  kalman->angle = 0.0;
  kalman->bias = 0.0;
  kalman->P[0][0] = 0.0;
  kalman->P[0][1] = 0.0;    
  kalman->P[1][0] = 0.0;
  kalman->P[1][1] = 0.0;
}

double kalman_getAngle(Kalman* kalman, double newAngle, double newRate, double dt) {

  // Time update
  kalman->rate = newRate - kalman->bias;
  kalman->angle += dt * kalman->rate;  

  kalman->P[0][0] += dt * (dt*kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
  kalman->P[0][1] -= dt * kalman->P[1][1];
  kalman->P[1][0] -= dt * kalman->P[1][1];
  kalman->P[1][1] += kalman->Q_bias * dt;

  // Measurement update
  double S = kalman->P[0][0] + kalman->R_measure;
  double K0 = kalman->P[0][0] / S;
  double K1 = kalman->P[1][0] / S;

  double y = newAngle - kalman->angle;
  kalman->angle += K0 * y;
  kalman->bias += K1 * y;

  kalman->P[0][0] -= K0 * kalman->P[0][0];
  kalman->P[0][1] -= K0 * kalman->P[0][1];
  kalman->P[1][0] -= K1 * kalman->P[0][0];
  kalman->P[1][1] -= K1 * kalman->P[0][1];

  return kalman->angle; 
}

void kalman_setAngle(Kalman* kalman, double newAngle) {
  kalman->angle = newAngle;
}

double kalman_getRate(Kalman* kalman) {
  return kalman->rate;
}

void kalman_setQangle(Kalman* kalman, double newQ_angle) {
  kalman->Q_angle = newQ_angle;
}

void kalman_setQbias(Kalman* kalman, double newQ_bias) {
  kalman->Q_bias = newQ_bias;  
}

void kalman_setRmeasure(Kalman* kalman, double newR_measure) {
  kalman->R_measure = newR_measure;
}

double kalman_getQangle(Kalman* kalman) {
  return kalman->Q_angle;
}

double kalman_getQbias(Kalman* kalman) {
  return kalman->Q_bias;
}

double kalman_getRmeasure(Kalman* kalman) { 
  return kalman->R_measure;
}