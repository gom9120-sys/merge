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
static double h_curb_6cm(double x)  { return x > 0.5 ? 0.06 : 0.0; }                  // 6cm 턱
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
  { "curb_6cm",   h_curb_6cm,  2.0, RISK_DANGER,  RISK_DANGER,  0 },
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
//
// 경사로에서는 유모차가 노면과 함께 기울고 센서도 같이 기운다. 그래서
// 센서 원점과 빔 방향을 바퀴 위치의 국부 기울기만큼 회전시킨다. 이걸 빼면
// 균일한 경사로에서도 측정값이 계속 변해서, 절대 임계값 방식에 불리하게
// (그리고 실제와 다르게) 나온다. 균일 경사에서는 평지와 같은 값이 나와야 맞다.
// 유모차가 놓이는 높이는 '센서 바로 아래 노면'이 아니라 '바퀴가 닿는 면'이
// 정한다. 바퀴는 자기 지름보다 좁은 홈은 다리처럼 건너가므로, 바퀴 반경
// 범위에서 가장 높은 노면이 차체를 받친다(형태학적 팽창). 이걸 빼면 센서가
// 홈에 같이 빠져서 거리 변화가 사라지고, 홈을 아예 감지할 수 없게 된다.
const double WHEEL_RADIUS_M = 0.035;   // 앞바퀴 지름 7cm

static double support(double p, double (*ground)(double)) {
  double best = -1e9;
  for (double x = p - WHEEL_RADIUS_M; x <= p + WHEEL_RADIUS_M; x += 0.002) {
    double g = ground(x);
    if (g > best) best = g;
  }
  return best;
}

// 센서가 바퀴 위치 p에 있을 때 실제로 나오는 빗변 거리(m). -1 = 반향 없음.
// 경사로에서는 차체가 노면과 함께 기울고 센서도 같이 기운다. 균일한 경사
// 에서는 평지와 같은 값이 나와야 맞다.
static double slantRangeM(double p, double (*ground)(double), uint8_t idx) {
  double h = US_MOUNT_HEIGHT_M[idx];
  double tilt = US_TILT_DEG[idx] * M_PI / 180.0;
  double half = BEAM_HALF_ANGLE_DEG * M_PI / 180.0;

  // 차체 기울기는 바퀴가 받치는 면의 기울기다 (양수 = 오르막)
  double slope = atan2(support(p + 0.03, ground) - support(p - 0.03, ground), 0.06);
  // 실제 유모차는 급한 턱에서 45도씩 기울지 않는다. 프레임이 버티는 범위로 제한.
  const double MAX_PITCH = 15.0 * M_PI / 180.0;
  if (slope > MAX_PITCH) slope = MAX_PITCH;
  if (slope < -MAX_PITCH) slope = -MAX_PITCH;

  // 센서 원점: 받침면에서 수직으로 h 만큼
  double ox = p - h * sin(slope);
  double oz = support(p, ground) + h * cos(slope);

  // 빔 중심의 내려다보는 각도도 기울기만큼 줄어든다 (앞이 들리면 덜 숙임)
  double lo = tilt - slope - half;
  double hi = tilt - slope + half;

  double best = -1.0;
  for (double x = ox - 0.30; x < ox + 1.20; x += 0.002) {
    double dx = x - ox;
    double dz = oz - ground(x);
    if (dz <= 0.0) continue;          // 센서보다 높은 노면은 볼 수 없다
    double phi = atan2(dz, dx);
    if (phi < lo || phi > hi) continue;
    double r = sqrt(dx * dx + dz * dz);
    if (best < 0.0 || r < best) best = r;
  }
  return best;
}

static uint16_t measureMm(double p, double (*ground)(double), uint8_t idx) {
  double best = slantRangeM(p, ground, idx);
  if (best < 0.0) return 0;

  double r = best + nrand() * NOISE_SIGMA_M;
  double q = urand();
  double tilt = US_TILT_DEG[idx];
  if (q < dropoutProb(tilt)) return 0;
  if (q < dropoutProb(tilt) + OUTLIER_P)
    r += OUTLIER_MIN_M + urand() * (OUTLIER_MAX_M - OUTLIER_MIN_M);
  double mm = r * 1000.0;
  return (mm < 0 || mm > 65000) ? 0 : (uint16_t)(mm + 0.5);
}


// ============================================================
// 후보 알고리즘: 절대 임계값으로 종류를 가르고 상태별 하위 판정
// ============================================================
// 초음파는 빔 원뿔 안의 최단 거리를 돌려준다. 그래서 평지에서는 빔의 가장
// 아래쪽 광선(내려다보는 각 = 설치각 + 반각)이 만드는 값 하나로 고정된다.
//     d0 = h / sin(설치각 + 반각)
// 이 값보다 길면 노면이 내려간 것(홈), 짧으면 올라온 것(턱)이다. 이전
// 프레임과의 차이(delta)를 볼 필요가 없다.
//
// 편차를 수직 높이로 환산하는 계수 k = sin(설치각 + 반각).
//     노면 높이 변화 = (d - d0) * k     (+ 홈, - 턱)
//
// 종류가 갈린 뒤 상태별 하위 판정으로 들어간다.
//   홈 상태 : 원본 흐름도대로 너비를 속도로 적분해 safe_gap과 비교
//   턱 상태 : 편차의 최대값(peak)을 추적한다. 경사로 진입도 전방을 보는
//             센서에는 턱처럼 잠깐 보이는데, 그 편차는
//                 전방주시거리 * tan(경사각)
//             밖에 안 되므로(70도/16cm에서 10도 경사면 10mm) 실제 턱의
//             수십 mm와 크기로 갈린다. 노면이 복귀했을 때 peak가
//             STEP_DANGER 미만이면 경사로/잔요철로 보고 SAFE.
enum CandState : uint8_t { CS_IDLE, CS_HOLE, CS_STEP };

double CAND_ENTER_M      = 0.020;  // 턱 진입 임계 (노면이 올라옴)
double CAND_HOLE_ENTER_M = 0.040;  // 홈 진입 임계 (노면이 내려감)
                                   // 얕은 굴곡을 홈으로 보지 않으려면 턱보다 높게
double CAND_STEP_DANGER_M= 0.040;  // 턱 높이가 이 이상이면 DANGER
double CAND_EXIT_M       = 0.010;  // 이 아래로 돌아오면 노면 복귀
double CAND_HOLE_DANGER_RATIO = 1.0;   // safe_gap 대비 위험 너비
double CAND_HOLE_CAUTION_RATIO = 0.5;

struct Cand {
  CandState state;
  double d0, k;
  double holeWidth, holeDepth, stepPeak;
  uint16_t recent[3]; uint8_t n;
  int risk, hazard;
};

static void candInit(Cand &c, uint8_t idx) {
  double tilt = US_TILT_DEG[idx], a = BEAM_HALF_ANGLE_DEG;
  c.state = CS_IDLE;
  // 실제 펌웨어에서는 부팅 때 평지에서 몇 프레임 재서 잡는 편이 낫다.
  c.d0 = US_MOUNT_HEIGHT_M[idx] / sin((tilt + a) * M_PI / 180.0);
  c.k = sin((tilt + a) * M_PI / 180.0);
  c.holeWidth = c.holeDepth = c.stepPeak = 0;
  c.n = 0; c.risk = RISK_SAFE; c.hazard = HAZARD_NONE;
}

static void candStep(Cand &c, uint16_t mm, double speed, double interval, uint8_t idx) {
  c.risk = RISK_SAFE; c.hazard = HAZARD_NONE;
  if (mm == 0) { c.n = 0; return; }

  // 현재 코드와 같은 3점 중앙값 (헛에코 제거)
  c.recent[2] = c.recent[1]; c.recent[1] = c.recent[0]; c.recent[0] = mm;
  if (c.n < 3) c.n++;
  uint16_t v = mm;
  if (c.n >= 3) {
    uint16_t a = c.recent[0], b = c.recent[1], d = c.recent[2], t;
    if (a > b) { t = a; a = b; b = t; }
    if (b > d) { t = b; b = d; d = t; }
    if (a > b) { t = a; a = b; b = t; }
    v = b;
  }

  double dev = (v / 1000.0 - c.d0) * c.k;   // + 홈 / - 턱

  switch (c.state) {
    case CS_IDLE:
      if (dev > CAND_HOLE_ENTER_M) {
        c.state = CS_HOLE;
        c.holeDepth = dev;
        c.holeWidth = usBeamFootprintM[idx];   // 빔 띠가 다 들어가야 보인다
      } else if (-dev > CAND_ENTER_M) {
        c.state = CS_STEP;
        c.stepPeak = -dev;
      }
      break;

    case CS_HOLE:
      c.holeWidth += speed * interval;
      if (dev > c.holeDepth) c.holeDepth = dev;
      if (dev < CAND_EXIT_M) {                 // 노면 복귀 -> 지나온 너비로 판정
        if (c.holeWidth >= HOLE_SAFE_GAP_M * CAND_HOLE_DANGER_RATIO) {
          c.risk = RISK_DANGER; c.hazard = HAZARD_HOLE;
        } else if (c.holeWidth >= HOLE_SAFE_GAP_M * CAND_HOLE_CAUTION_RATIO) {
          c.risk = RISK_CAUTION; c.hazard = HAZARD_HOLE;
        }
        c.state = CS_IDLE; c.holeWidth = c.holeDepth = 0;
      } else if (c.holeWidth >= HOLE_SAFE_GAP_M * CAND_HOLE_DANGER_RATIO) {
        c.risk = RISK_DANGER; c.hazard = HAZARD_HOLE;   // 지나는 중에 이미 확정
        c.state = CS_IDLE; c.holeWidth = c.holeDepth = 0;
      } else if (c.holeWidth >= HOLE_SAFE_GAP_M * CAND_HOLE_CAUTION_RATIO) {
        c.risk = RISK_CAUTION; c.hazard = HAZARD_HOLE;
      }
      break;

    case CS_STEP:
      if (-dev > c.stepPeak) c.stepPeak = -dev;
      if (c.stepPeak >= CAND_STEP_DANGER_M) {  // 턱 확정. 즉시 알린다
        c.risk = RISK_DANGER; c.hazard = HAZARD_STEP;
        c.state = CS_IDLE; c.stepPeak = 0;
      } else if (-dev < CAND_EXIT_M) {
        // 노면이 돌아왔고 peak가 작았다 = 경사로 진입이나 잔요철
        c.state = CS_IDLE; c.stepPeak = 0;
      }
      break;
  }
}


// 위험이 바퀴에 닿기 "전에" 얼마나 미리 잡히는지를 잰다.
// p = 바퀴 접지 위치. 모든 위험 지형은 x = 0.5 m에서 시작하므로
//   경보 여유 = 0.5 - (DANGER가 처음 뜬 p)
// 양수면 바퀴가 닿기 전에 예측한 것, 음수면 이미 지난 뒤 알린 것이다.
static bool runWarn(const Scenario &sc, double speed, double *warnM) {
  setupTerrainDetectors();   // 빔 띠/전방주시 계산
  Cand c; candInit(c, 0);
  double T = (US_PING_INTERVAL_MS * US_COUNT) / 1000.0;
  for (double p = 0.0; p < sc.lengthM; p += speed * T) {
    candStep(c, measureMm(p, sc.h, 0), speed, T, 0);
    if (c.risk == RISK_DANGER) { *warnM = 0.5 - p; return true; }
  }
  return false;
}

int main() {
  const char *names[] = { "curb_6cm", "curb_10cm", "dropoff", "hole_8cm" };
  double (*hs[])(double) = { h_curb_6cm, h_curb_up, h_dropoff, h_hole_wide };
  double speeds[] = { 0.3, 0.4, 0.5 };
  const int TRIALS = 40;

  setupTerrainDetectors();
  printf("높이 %.2fm / %.0f도 | 빔띠 %.0fmm 전방주시 %.0fmm\n",
         US_MOUNT_HEIGHT_M[0], US_TILT_DEG[0],
         usBeamFootprintM[0] * 1000.0, usLookAheadM[0] * 1000.0);
  double allSum = 0; int allN = 0, allDet = 0, allRun = 0, allLate = 0;
  for (int i = 0; i < 4; i++) {
    Scenario sc = { names[i], hs[i], 2.0, RISK_DANGER, RISK_DANGER, 0 };
    double sum = 0, worst = 1e9; int det = 0, run = 0, late = 0;
    for (int si = 0; si < 3; si++) for (int t = 0; t < TRIALS; t++) {
      rngState = 12345 + t * 7919 + i * 104729 + si * 31;
      double w; run++;
      if (runWarn(sc, speeds[si], &w)) {
        det++; sum += w; if (w < worst) worst = w; if (w < 0) late++;
      }
    }
    printf("  %-10s 검출 %3d/%3d  평균 여유 %+6.1fmm  최악 %+6.1fmm  늦은 경보 %d\n",
           names[i], det, run, det ? sum / det * 1000.0 : 0.0,
           det ? worst * 1000.0 : 0.0, late);
    allSum += sum; allN += det; allDet += det; allRun += run; allLate += late;
  }
  printf("  >> 합계 검출 %d/%d  평균 여유 %+.1fmm  (0.4m/s에서 %+.0fms)  늦은 경보 %d\n\n",
         allDet, allRun, allSum / allN * 1000.0, allSum / allN / 0.4 * 1000.0, allLate);
  return 0;
}
