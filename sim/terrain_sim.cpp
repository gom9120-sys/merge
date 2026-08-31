// 노면 위험 판정 시뮬레이터. merge.ino의 판정 코드를 그대로 컴파일해서 돌린다.
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
unsigned long g_millis = 0;
#include "Wire.h"
SerialClass Serial; WireClass Wire; bool g_serialEcho = false;
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
static double h_hole_5cm(double x)  { return (x > 0.5 && x < 0.55) ? -0.10 : 0.0; }   // 5cm 홈
static double h_hole_wide(double x) { return (x > 0.5 && x < 0.58) ? -0.12 : 0.0; }   // 8cm 홈
static double h_curb_up(double x)   { return x > 0.5 ? 0.10 : 0.0; }                  // 10cm 턱
static double h_dropoff(double x)   { return x > 0.5 ? -0.15 : 0.0; }                 // 15cm 단차/계단
static double h_exit_step(double x) { if (x > 0.5 && x < 0.53) return -0.10;
                                      if (x >= 0.53) return 0.10; return 0.0; }       // 홈 뒤 턱

#define M(r) (1 << (r))
static Scenario SCENARIOS[] = {
  { "flat",       h_flat,      2.0, RISK_SAFE,    RISK_SAFE,    0 },
  { "rough",      h_rough,     2.0, RISK_SAFE,    RISK_SAFE,    0 },
  { "ramp_up",    h_ramp_up,   2.0, RISK_SAFE,    RISK_SAFE,    0 },
  { "ramp_down",  h_ramp_dn,   2.0, RISK_SAFE,    RISK_SAFE,    0 },
  { "crack_2cm",  h_crack,     2.0, RISK_SAFE,    RISK_CAUTION, 0 },
  // 3cm 홈은 safe_gap(4.2cm)보다 좁아 지나가도 되는 홈이다. SAFE가 정답이고
  // CAUTION까지는 봐준다. 반대로 5cm 홈부터는 반드시 DANGER여야 한다.
  { "hole_3cm",   h_hole_med,  2.0, RISK_SAFE,    RISK_CAUTION, 0 },
  { "hole_5cm",   h_hole_5cm,  2.0, RISK_DANGER,  RISK_DANGER,  0 },
  { "hole_8cm",   h_hole_wide, 2.0, RISK_DANGER,  RISK_DANGER,  0 },
  { "curb_up",    h_curb_up,   2.0, RISK_DANGER,  RISK_DANGER,  0 },
  { "dropoff",    h_dropoff,   2.0, RISK_DANGER,  RISK_DANGER,  0 },
  { "exit_step",  h_exit_step, 2.0, RISK_DANGER,  RISK_DANGER,  0 },
};
static const int SCENARIO_COUNT = sizeof(SCENARIOS) / sizeof(SCENARIOS[0]);

// 센서 측정 잡음 모델
double NOISE_SIGMA_M = 0.005;   // 빗변 거리 표준편차
double OUTLIER_P     = 0.02;    // 헛에코 확률
double OUTLIER_MIN_M = 0.05, OUTLIER_MAX_M = 0.10;

// ---------- HC-SR04 빔 모델 ----------
// 초음파는 연필 같은 선이 아니라 원뿔이다. HC-SR04는 유효 빔이 대략 15도
// (반각 7.5도)라, 기울여 달면 노면을 꽤 넓은 띠로 비춘다. 그리고 거리계는
// 그 띠 안에서 '가장 먼저 돌아온 에코', 즉 최단 거리를 값으로 내놓는다.
//   -> 홈이 빔이 비추는 띠보다 좁으면, 홈 주변의 평지가 먼저 반향을 돌려줘서
//      거리값이 아예 변하지 않는다. 즉 빔 폭이 곧 감지 가능한 최소 홈 너비다.
//   -> 띠의 길이는 설치 각도로 정해진다.
//        가까운 끝 = h / tan(각도 + 7.5), 먼 끝 = h / tan(각도 - 7.5)
//      30도면 약 167mm, 45도면 80mm, 60도면 53mm, 90도(수직)면 39mm.
// 이 효과를 빼고는 설치 각도를 고를 수 없어서 모델에 넣었다.
const double BEAM_HALF_ANGLE_DEG = 7.5;

// 스침각(grazing)에서는 노면이 거울처럼 반사해 에코가 통째로 사라진다.
// 수직에 가까울수록 잘 돌아온다. sin(각도)로 실패 확률을 준다.
double DROPOUT_BASE = 0.02, DROPOUT_GRAZE = 0.30;

static double dropoutProb(double tiltDeg) {
  double s = sin(tiltDeg * M_PI / 180.0);
  return DROPOUT_BASE + DROPOUT_GRAZE * (1.0 - s) * (1.0 - s);
}

// 센서가 노면 위 위치 p에 있을 때 실제로 나오는 측정값(mm).
// 빔 원뿔 안에 들어오는 노면 점들 중 최단 거리를 고른다. 0 = 측정 실패.
static uint16_t measureMm(double p, double (*ground)(double), uint8_t idx) {
  double h = US_MOUNT_HEIGHT_M[idx];
  double tilt = US_TILT_DEG[idx];
  double lo = (tilt - BEAM_HALF_ANGLE_DEG) * M_PI / 180.0;
  double hi = (tilt + BEAM_HALF_ANGLE_DEG) * M_PI / 180.0;

  double best = -1.0;
  for (double x = p - 0.05; x < p + 1.2; x += 0.002) {
    double dx = x - p;
    double dz = h - ground(x);          // 센서에서 그 점까지의 수직 낙차
    if (dz <= 0.0) continue;            // 센서보다 높은 노면은 안 본다
    double phi = atan2(dz, dx);         // 내려다보는 각 (90도 = 바로 아래)
    if (phi < lo || phi > hi) continue; // 빔 밖
    double r = sqrt(dx * dx + dz * dz);
    if (best < 0.0 || r < best) best = r;
  }
  if (best < 0.0) return 0;

  double r = best + nrand() * NOISE_SIGMA_M;
  double q = urand();
  if (q < dropoutProb(tilt)) return 0;
  if (q < dropoutProb(tilt) + OUTLIER_P)
    r += OUTLIER_MIN_M + urand() * (OUTLIER_MAX_M - OUTLIER_MIN_M);
  double mm = r * 1000.0;
  return (mm < 0 || mm > 65000) ? 0 : (uint16_t)(mm + 0.5);
}

// 한 시나리오를 한 속도로 통과시키고 최대 위험도/원인을 돌려준다.
void runScenario(const Scenario &sc, double speed, int *outRisk, int *outHazard) {
  setupTerrainDetectors();
  imuReady = true;
  estimatedSpeedMps = speed;

  double T = (US_PING_INTERVAL_MS * US_COUNT) / 1000.0;  // 센서당 측정 간격
  g_millis = 1000;
  int maxRisk = RISK_SAFE, maxHazard = HAZARD_NONE;

  for (double p = 0.0; p < sc.lengthM; p += speed * T) {
    g_millis += (unsigned long)(T * 1000.0 + 0.5);
    uint16_t mm = measureMm(p, sc.h, 0);
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
