// 노면 위험 판정 시뮬레이터. merge.ino의 판정 코드를 그대로 컴파일해서 돌린다.
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
unsigned long g_millis = 0;
#include "Wire.h"
SerialClass Serial; WireClass Wire;
#include "../merge.ino"

// ---------- 난수 (재현 가능하게 고정 시드) ----------
static unsigned long rngState = 1;
static double urand() { rngState = rngState * 6364136223846793005ULL + 1442695040888963407ULL;
                        return ((rngState >> 33) & 0x7FFFFFFF) / (double)0x7FFFFFFF; }
static double nrand() { double u1 = urand() + 1e-12, u2 = urand();
                        return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2); }

// ---------- 노면 프로파일 ----------
// h(x) = 노면 높이(m). +면 지면이 올라온 것(턱), -면 내려간 것(홈).
struct Scenario {
  const char *name;
  double (*h)(double x);
  double lengthM;
  int minRisk;           // 이보다 낮게 보고하면 과소경보(위험)
  int maxRisk;           // 이보다 높게 보고하면 과잉경보(성가심)
  int expectHazardMask;  // 허용 원인 비트, 0이면 원인은 보지 않음
};

static double h_flat(double x)      { (void)x; return 0.0; }
static double h_rough(double x)     { return 0.008 * sin(x * 40.0); }            // 잔요철 +-8mm
static double h_ramp_up(double x)   { return x < 0.3 ? 0.0 : (x - 0.3) * 0.18; } // 10도 경사 시작
static double h_ramp_dn(double x)   { return x < 0.3 ? 0.0 : -(x - 0.3) * 0.18; }
static double h_crack(double x)     { return (x > 0.5 && x < 0.52) ? -0.10 : 0.0; }   // 2cm 홈
static double h_hole_med(double x)  { return (x > 0.5 && x < 0.53) ? -0.10 : 0.0; }   // 3cm 홈
static double h_hole_wide(double x) { return (x > 0.5 && x < 0.58) ? -0.12 : 0.0; }   // 8cm 홈
static double h_curb_up(double x)   { return x > 0.5 ? 0.10 : 0.0; }                  // 10cm 턱
static double h_dropoff(double x)   { return x > 0.5 ? -0.15 : 0.0; }                 // 15cm 단차/계단
static double h_exit_step(double x) { if (x > 0.5 && x < 0.53) return -0.10;
                                      if (x >= 0.53) return 0.10; return 0.0; }       // 홈 뒤 턱

#define M(r) (1 << (r))
static Scenario SCENARIOS[] = {
  { "flat",       h_flat,      1.5, RISK_SAFE,    RISK_SAFE,    0 },
  { "rough",      h_rough,     1.5, RISK_SAFE,    RISK_SAFE,    0 },
  { "ramp_up",    h_ramp_up,   1.5, RISK_SAFE,    RISK_SAFE,    0 },
  { "ramp_down",  h_ramp_dn,   1.5, RISK_SAFE,    RISK_SAFE,    0 },
  { "crack_2cm",  h_crack,     1.5, RISK_SAFE,    RISK_CAUTION, 0 },
  { "hole_3cm",   h_hole_med,  1.5, RISK_CAUTION, RISK_DANGER,  0 },
  { "hole_8cm",   h_hole_wide, 1.5, RISK_DANGER,  RISK_DANGER,  0 },
  { "curb_up",    h_curb_up,   1.5, RISK_DANGER,  RISK_DANGER,  0 },
  { "dropoff",    h_dropoff,   1.5, RISK_DANGER,  RISK_DANGER,  0 },
  { "exit_step",  h_exit_step, 1.5, RISK_DANGER,  RISK_DANGER,  0 },
};
static const int SCENARIO_COUNT = sizeof(SCENARIOS) / sizeof(SCENARIOS[0]);

// 센서 측정 잡음 모델
double NOISE_SIGMA_M = 0.005;   // 빗변 거리 표준편차
double DROPOUT_P     = 0.05;    // 측정 실패 확률
double OUTLIER_P     = 0.02;    // 헛에코 확률
double OUTLIER_MIN_M = 0.05, OUTLIER_MAX_M = 0.10;

// 한 시나리오를 한 속도로 통과시키고 최대 위험도/원인을 돌려준다.
void runScenario(const Scenario &sc, double speed, int *outRisk, int *outHazard) {
  setupTerrainDetectors();
  imuReady = true;
  estimatedSpeedMps = speed;

  double T = (US_PING_INTERVAL_MS * US_COUNT) / 1000.0;  // 센서당 측정 간격
  g_millis = 1000;
  int maxRisk = RISK_SAFE, maxHazard = HAZARD_NONE;

  for (double x = 0.0; x < sc.lengthM; x += speed * T) {
    g_millis += (unsigned long)(T * 1000.0 + 0.5);
    double drop = US_MOUNT_HEIGHT_M[0] - sc.h(x);
    double slant = drop / usSinTilt[0] + nrand() * NOISE_SIGMA_M;

    uint16_t mm;
    double r = urand();
    if (r < DROPOUT_P) mm = 0;
    else {
      if (r < DROPOUT_P + OUTLIER_P)
        slant += OUTLIER_MIN_M + urand() * (OUTLIER_MAX_M - OUTLIER_MIN_M);
      double v = slant * 1000.0;
      mm = (v < 0 || v > 65000) ? 0 : (uint16_t)(v + 0.5);
    }

    detectors[0].risk = RISK_SAFE;
    detectors[0].hazard = HAZARD_NONE;
    runTerrainDetector(0, mm, g_millis);
    if (detectors[0].risk > maxRisk) {
      maxRisk = detectors[0].risk;
      maxHazard = detectors[0].hazard;
    }
  }
  *outRisk = maxRisk;
  *outHazard = maxHazard;
}

int main(int argc, char **argv) {
  int verbose = (argc > 1 && strcmp(argv[1], "-v") == 0);
  double speeds[] = { 0.2, 0.3, 0.4, 0.5, 0.6 };
  int nSpeeds = sizeof(speeds) / sizeof(speeds[0]);
  const int TRIALS = 25;

  int under = 0, over = 0, run = 0;
  for (int si = 0; si < nSpeeds; si++) {
    int su = 0, so = 0;
    for (int i = 0; i < SCENARIO_COUNT; i++) {
      int u = 0, o = 0;
      for (int t = 0; t < TRIALS; t++) {
        rngState = 12345 + t * 7919 + i * 104729;
        int risk, hazard;
        runScenario(SCENARIOS[i], speeds[si], &risk, &hazard);
        if (risk < SCENARIOS[i].minRisk) u++;
        else if (risk > SCENARIOS[i].maxRisk) o++;
      }
      if (verbose)
        printf("  v=%.1f %-11s under=%2d over=%2d /%d\n",
               speeds[si], SCENARIOS[i].name, u, o, TRIALS);
      su += u; so += o; run += TRIALS;
    }
    if (verbose) printf("v=%.1f under=%d over=%d | ", speeds[si], su, so);
    under += su; over += so;
  }
  printf("COST %d UNDER %d OVER %d RUN %d PING %lu SUDDEN %.3f TOLR %.2f "
         "ENTRY %.2f TILT %.0f STALE %.2f KEEPPREV %d EXITFIRST %d\n",
         under * 10 + over, under, over, run, US_PING_INTERVAL_MS,
         SUDDEN_CHANGE_THRESHOLD_M, HOLE_TOLERANCE_RATIO, HOLE_ENTRY_WIDTH_RATIO,
         US_TILT_DEG[0], US_STALE_INTERVAL_S, US_KEEP_PREV_ON_DROPOUT, HOLE_EXIT_CHECK_FIRST);
  return 0;
}
