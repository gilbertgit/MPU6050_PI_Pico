#ifndef _KALMAN_H_
#define _KALMAN_H_

typedef struct {
  double Q_angle;
  double Q_bias; 
  double R_measure;
  double angle;
  double bias; 
  double rate;
  double P[2][2];
} Kalman;

void kalman_init(Kalman* kalman);

double kalman_getAngle(Kalman* kalman, double newAngle, double newRate, double dt);

void kalman_setAngle(Kalman* kalman, double newAngle);

double kalman_getRate(Kalman* kalman);

void kalman_setQangle(Kalman* kalman, double newQ_angle);

void kalman_setQbias(Kalman* kalman, double newQ_bias);

void kalman_setRmeasure(Kalman* kalman, double newR_measure);

double kalman_getQangle(Kalman* kalman);

double kalman_getQbias(Kalman* kalman);

double kalman_getRmeasure(Kalman* kalman);
#endif // _KALMAN_H_
