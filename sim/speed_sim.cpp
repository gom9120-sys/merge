#include <stdio.h>
#include <stdint.h>
#include <math.h>
unsigned long g_millis = 0;
#include "Wire.h"
SerialClass Serial; WireClass Wire;
#include "../merge.ino"
static unsigned long rs = 7;
static double urand(){ rs = rs*6364136223846793005ULL + 1442695040888963407ULL;
                       return ((rs>>33)&0x7FFFFFFF)/(double)0x7FFFFFFF; }
static double nrand(){ double u1=urand()+1e-12,u2=urand(); return sqrt(-2*log(u1))*cos(2*M_PI*u2); }
static void trueState(double t, double vC, double *v, double *a) {
  if (t < 1.0)      { *v = 0;              *a = 0; }
  else if (t < 2.0) { *v = vC * (t - 1.0); *a = vC; }
  else if (t < 8.0) {  // 순항 중 속도가 변하는 경우도 포함(vC의 +-50%)
    double f = 1.0 + 0.5 * sin((t - 2.0) * 0.7);
    *v = vC * f; *a = vC * 0.5 * 0.7 * cos((t - 2.0) * 0.7);
  }
  else if (t < 9.0) { *v = vC * (9.0 - t); *a = -vC; }  // 감속
  else              { *v = 0;              *a = 0; }
}
// 한 조건 실행: 순항 RMS 오차와 정지 후 잔류 속도를 돌려준다
static void runCase(double vC, double vibMoving, double bias, int seed,
                    double *rms, double *restMax) {
  rs = 7 + seed * 104729;
  double dt = IMU_INTERVAL_MS / 1000.0;
  imuReady = true; filteredPitchDeg = 0; pitchMountOffsetDeg = 0;
  estimatedSpeedMps = 0; accelBiasMps2 = 0; accelVibLpMps2 = 0;
  lastAccelActiveAtMs = 0; g_millis = 0;
  double sumErr2 = 0; int n = 0; double rmax = 0;
  for (int k = 0; k * dt < 11.0; k++) {
    double t = k * dt; g_millis = (unsigned long)(t * 1000.0);
    double vT, aT; trueState(t, vC, &vT, &aT);
    double vib = (vT > 0.01 ? vibMoving : 0.02) * nrand();
    double aMeas = aT + bias + vib;
    az = (int16_t)(-(aMeas / GRAVITY_MPS2) * ACCEL_SENSITIVITY);
    updateSpeedEstimate(dt, g_millis);
    if (t > 2.5 && t < 8.0) { double e = estimatedSpeedMps - vT; sumErr2 += e*e; n++; }
    if (t > 10.0 && estimatedSpeedMps > rmax) rmax = estimatedSpeedMps;
  }
  *rms = sqrt(sumErr2 / n); *restMax = rmax;
}
int main(int argc, char**argv) {
  int verbose = (argc > 1);
  double speeds[] = { 0.2, 0.4, 0.6 };
  double vibs[]   = { 0.30, 0.10 };   // 거친 노면 / 매끈한 실내
  double biases[] = { 0.05, -0.15 };  // 가속도계 바이어스, 피치 오차
  double sumRms = 0, worstRest = 0; int n = 0;
  for (int a = 0; a < 3; a++) for (int b = 0; b < 2; b++) for (int c = 0; c < 2; c++)
    for (int seed = 0; seed < 5; seed++) {
      double rms, rest; runCase(speeds[a], vibs[b], biases[c], seed, &rms, &rest);
      if (verbose && seed == 0)
        printf("  v=%.1f vib=%.2f bias=%+.2f -> RMS %.3f, 정지잔류 %.3f\n",
               speeds[a], vibs[b], biases[c], rms, rest);
      sumRms += rms; if (rest > worstRest) worstRest = rest; n++;
    }
  printf("SPEEDCOST %.3f RMS %.3f REST %.3f NOM %.2f LEAK %.2f BIASTC %.1f "
         "VIBTC %.2f DEAD %.2f HOLD %lu\n",
         sumRms / n + worstRest, sumRms / n, worstRest, SPEED_NOMINAL_MPS,
         SPEED_LEAK_PER_S, SPEED_BIAS_TC_S, SPEED_VIB_TC_S,
         SPEED_MOTION_DEADBAND_MPS2, SPEED_ZERO_HOLD_MS);
  return 0;
}
