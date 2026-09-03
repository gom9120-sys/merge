// ============================================================
// merge_idle.ino - 보관용 이전 버전 (2026-09-03 시점)
//
// 현재 버전(../merge.ino)과 판정 알고리즘은 같다. 절대 기준거리에서
// 벗어난 방향으로 턱/홈을 가르고, 턱은 최대 편차로 경사로와 구분한다.
// 다른 점은 판정 임계값을 어디서 얻느냐 하나다.
//
//   이 파일      : 임계값이 고정 상수다.
//                  STEP_ENTER_M 20mm / STEP_DANGER_M 40mm / HOLE_ENTER_M 40mm
//                  부팅 보정은 기준거리(d0)만 잡는다.
//   ../merge.ino : 부팅 보정에서 잡음까지 재서 임계값을 그 배수로 만들고,
//                  빔 기하에서 나오는 하한(점프 상한, 경사로 겉보기 턱)을
//                  함께 씌운다. 측정 실패 프레임에서 중앙값 버퍼를 비우던
//                  버그도 고쳤다.
//
// 시뮬레이션 비교 (중앙 45도, 평지·요철·경사 400회 / 턱·단차 500회)
//   이 파일      : 오경보 75/400,  턱·단차 미탐  7/500
//   ../merge.ino : 오경보 18/400,  턱·단차 미탐 17/500
//
// 현장에서 부팅 보정이 불안하거나(시동 위치를 평지로 잡기 어려운 경우)
// 임계값을 손으로 못 박고 싶을 때 이 버전을 굽는다.
//
// 아두이노 IDE는 폴더 이름과 같은 .ino를 스케치로 연다. 그래서 merge.ino와
// 한 폴더에 두지 않고 merge_idle/ 아래에 따로 두었다. 두 파일을 같은
// 폴더에 두면 IDE가 둘을 이어붙여서 컴파일이 깨진다.
// ============================================================

#include <Wire.h>

// ============================================================
// merge.ino - 지능형 안전 유모차 통합 펌웨어 (Arduino Mega 2560)
//
// 통합 대상
//   stroller_control.ino : IMU + 홀 + FSR + ZS-X11H 좌/우 모터 제어
//   handle_sensor.ino    : FSR 2개(A0/A1) 손잡이 감지
//   detect.py            : HC-SR04 3개(좌/중/우) 노면 위험(턱/홈/계단) 판정
//
// 동작 우선순위
//  1) IMU 이상 / 손잡이 놓음 / 내리막 -> 즉시 전자제동
//  2) (옵션) 전방 장애물 근접          -> 즉시 전자제동
//  3) 오르막 + 손잡이 잡음             -> 경사에 따라 PWM 자동 보조
//  4) 평지 / 경사 불확실               -> 감속 후 출력 OFF(Coast)
//  5) 안전벨트 홀센서                  -> 경고 표시만, 모터에는 미개입
//  6) 노면 위험(턱/홈/계단)            -> 위험도/위험원인만 앱으로 전송
//
// 모터 드라이버: ZS-X11H V1 (3상 BLDC, 300W 허브모터)
//   제어선 3개  PWM(속도) / DIR(방향, 액티브 로우) / BRAKE(제동, 액티브 하이)
//     전진   : BRAKE=LOW, DIR=전진레벨, PWM=analogWrite(속도)
//     제동   : PWM=LOW,  BRAKE=HIGH
//     코스트 : PWM=LOW,  BRAKE=LOW
//
// 주의: BRAKE가 액티브 하이라서 MCU가 리셋되거나 전원이 빠진 동안에는
// BRAKE 선이 플로팅이 되어 제동이 걸리지 않는다. 각 BRAKE 선의 10k 풀업
// (기판 R3/R4)이 그 페일세이프다.
//
// 주의: 전자제동은 소형 모의실험용이며 실제 유모차의 기계식 안전
// 브레이크를 대신할 수 없다.
//
// ------------------------------------------------------------
// 노면 위험 판정 (detect.py의 DangerDetector 이식)
// ------------------------------------------------------------
// 파이썬 원본은 라즈베리파이가 초음파 3개를 읽고, 아두이노가 시리얼
// (115200)로 보내주는 IMU 속도(speed)를 받아 홈의 너비를 적분했다.
// 여기서는 판정 자체가 아두이노 안으로 들어왔으므로 시리얼로 속도를
// 주고받을 필요가 없다. 속도는 같은 보드의 MPU6050에서 바로 얻는다
// (updateSpeedEstimate 참고). 따라서 원본의 115200 시리얼 링크는 없다.
//
// 센서마다 독립된 DangerDetector 상태를 갖고, 그 센서의 새 측정이
// 끝난 순간(finishPing)에 그 센서의 직전 측정과 비교해 한 프레임을
// 진행시킨다. actual_interval은 그 센서의 직전 측정으로부터의 실제
// 경과 시간이다(라운드로빈이므로 센서당 약 US_PING_INTERVAL_MS * 3).
//
// 판정 흐름(원본 predict()와 동일한 순서)
//   첫 측정 / 측정 실패      -> 값만 저장하고 SAFE
//   in_hole == false
//     |delta| < ramp        -> 경사로. SAFE
//     delta < -sudden       -> 턱(가까워짐). DANGER, STEP
//     delta > +sudden       -> 홈 진입. SAFE, hole_depth=delta,
//                              hole_width = speed * interval
//     그 밖(임계값 사이)     -> SAFE
//   in_hole == true
//     hole_width += speed * interval
//     hole_width >= safe_gap            -> DANGER, STAIR (상태 해제)
//     0.5*safe_gap <= w < safe_gap      -> CAUTION, WIDE_HOLE
//     그 밖
//       -delta > depth + tol            -> DANGER, EXIT_STEP (상태 해제)
//       -delta > depth - tol            -> 원래 높이로 복귀 (상태 해제)
//       그 밖                            -> depth = max(0, depth + delta)
//
// safe_gap = 0.6 * 앞바퀴 지름, tolerance = 0.3 * 앞바퀴 지름 도 원본 그대로다.
// 세 값 모두 실험적으로 맞춰야 하는 값이라 아래 상수로 빼 두었다.
//
// 센서 장착 높이/각도
//   초음파를 노면 쪽으로 비스듬히 달면 측정값은 빗변 거리라서 센서마다
//   기준이 다르다. 그래서 원본의 '거리' 대신 수직 낙차
//       drop = 측정거리 * sin(설치 각도)
//   를 판정에 넣는다. 평지에서 drop은 설치 높이와 같고, delta는 그대로
//   노면의 높이 변화(m)가 되어 원본 임계값(m)을 그대로 쓸 수 있다.
//   높이/각도는 US_MOUNT_HEIGHT_M[], US_TILT_DEG[]에서 센서별로 바꾼다.
//
// ------------------------------------------------------------
// 설치값과 판정 방식의 근거 (시뮬레이션)
// ------------------------------------------------------------
// 이 파일의 판정 코드를 그대로 떼어내 PC에서 돌린 시뮬레이션으로 정했다
// (sim/ 폴더). HC-SR04의 빔을 원뿔로 모델링하고, 바퀴가 자기 지름보다 좁은
// 홈을 다리처럼 건너가는 것과 경사로에서 차체가 함께 기우는 것까지 넣었다.
//
// 검출률만 세면 '바퀴가 이미 지나간 뒤에 뜬 경보'가 성공으로 잡힌다.
// 그래서 경보가 바퀴 도착보다 얼마나 앞서 뜨는지(선행거리)를 지표로 썼다.
// 높이 16cm, 0.3~0.5 m/s, 지형당 120회 기준:
//
//   각도   빔 띠   전방주시   6cm 턱      10cm 턱     15cm 단차   8cm 홈
//    45도   86mm   160mm     +107mm      +74mm       +112mm      늦음
//                            119/120     120/120     120/120     65/120
//    65도   51mm    75mm     -346mm      -265mm      +48mm       +40mm
//                             29/120      24/120     120/120    118/120
//
// 얕은 각도는 턱과 단차를 미리 잡고(0.4 m/s에서 약 0.27초), 급한 각도는
// 빔 띠가 좁아 홈을 잡는다. 한 각도로 둘 다 안 되므로 센서별로 나눴다.
//   중앙 45도 : 턱과 단차. 연석이나 계단은 가로로 길어 하나로 충분하다.
//   좌우 65도 : 홈. 홈은 국소적이라 바퀴가 지나갈 경로에서 봐야 한다.
//
// 판정 방식도 바꿨다. 원본 detect.py는 이전 프레임과의 차이(delta)를 봤는데,
// 초음파가 빔 안의 최단 거리를 돌려준다는 성질을 쓰면 평지 기준거리 하나로
// 종류를 바로 가를 수 있다. 같은 시나리오에서 (높이 12cm, 70도)
//     원본 델타 방식 : 미탐 208 / 과잉  99
//     절대 기준거리  : 미탐  74 / 과잉 173
// 6cm 턱 미탐이 112/125에서 1/125로 줄었다. 턱 판정에는 속도 추정이 전혀
// 필요 없어서, 오차가 가장 컸던 입력이 판정에서 빠진다.
//
// 남는 한계는 홈이다. 5cm 안팎의 홈은 어느 각도에서도 28~32/120에 그친다.
// 빔 띠가 45mm라 5cm 홈은 한 프레임만 보이고, 그 한 프레임은 헛에코와
// 구분할 방법이 없다. 앞바퀴 7cm에는 위험한 크기지만 이 센서의 물리적
// 한계다. 여기서는 홈을 접고 턱과 단차에 맞췄다.
//
// ------------------------------------------------------------
// 시리얼 텔레메트리 포맷 (라즈베리파이/앱 파싱용)
// ------------------------------------------------------------
// 고정 순서 CSV. 파이에서는 split(',') 한 번이면 끝난다.
//
//   T,3,<seq>,<pitch>,<slope>,<fsr1>,<fsr2>,<handle>,<belt>,<mode>,<pwm>,
//     <motor>,<us_l>,<us_c>,<us_r>,<risk>,<hazard>
//   0 1   2      3        4       5      6      7        8      9     10
//     11      12     13     14     15      16
//
//   [0] "T"      텔레메트리 줄 표식 (이 글자로 시작하지 않는 줄은 무시)
//   [1] 포맷 버전 (현재 3). 필드를 늘리면 반드시 올린다.
//   [2] seq      0-255 순환. 줄 유실 감지용
//   [3] pitch    도, 소수 2자리 (+ 오르막 / - 내리막)
//   [4] slope    0=FLAT 1=UP 2=DOWN 3=UNCERTAIN
//   [5] fsr1     A0 원시값 0-1023
//   [6] fsr2     A1 원시값 0-1023
//   [7] handle   0=놓음 1=잡음
//   [8] belt     0=풀림 1=체결
//   [9] mode     0=INITIALIZING 1=SENSOR_FAULT 2=HANDLE_RELEASED
//                3=DOWNHILL_BRAKE 4=UPHILL_ASSIST 5=FLAT 6=UNCERTAIN
//                7=OBSTACLE_BRAKE
//  [10] pwm      현재 출력 0-255
//  [11] motor    0=COAST 1=FORWARD 2=BRAKE
//  [12] us_l     좌 초음파 거리 mm (노면용이라 20-1000mm), 0 = 측정 실패
//  [13] us_c     중앙
//  [14] us_r     우
//  [15] risk     노면 위험도 0=SAFE 1=CAUTION 2=DANGER (세 센서 중 최대)
//  [16] hazard   위험원인 0=NONE 1=STEP(턱) 2=HOLE(홈)
//
// 속도는 홈 너비를 적분하는 데만 쓰고 밖으로 내보내지 않는다.
//
// 위험도/위험원인이 바뀌는 순간에는 500ms 주기를 기다리지 않고 즉시
// 한 줄을 더 내보낸다(앱 경보용). 텔레메트리와 같은 코드값을 쓴다:
//
//   H,<risk>,<hazard>,<sensor>,<dist_mm>,<width_mm>,<depth_mm>
//     sensor   위험을 만든 센서 0=좌 1=중 2=우, 255=없음(위험 해제)
//     dist_mm  그 순간 그 센서의 측정 거리
//     width_mm 판정 당시 누적된 홈의 너비 (턱이면 0)
//     depth_mm 홈이면 홈의 깊이, 턱이면 노면이 올라온 높이
//
// 이벤트/오류는 별도 줄로 나간다:  E,<사유>
//   E,BOOT / E,CAL,<남은초> / E,IMU_READY / E,IMU_FAIL / E,IMU_LOST / E,READY
//   E,US_BEAM,<센서>,<빔길이mm>,<전방주시mm>  센서별 빔 기하 (부팅 시 3줄)
//   E,US_BASE,<l>,<c>,<r>  평지에서 실측한 기준거리 mm (보정 완료 시 1회)
//
// HUMAN_READABLE_LOG를 1로 바꾸면 '#'로 시작하는 사람이 읽는 줄이 하나 더
// 나간다(기본 0, 현장 확인용). 아래 파서는 어차피 무시한다.
//
// 파이 파서 예시:
//   SLOPE  = ("FLAT","UP","DOWN","UNCERTAIN")
//   MOTOR  = ("COAST","FORWARD","BRAKE")
//   RISK   = ("SAFE","CAUTION","DANGER")
//   HAZARD = ("NONE","STEP","HOLE")
//   def parse_tel(line):
//       p = line.strip().split(',')
//       if len(p) != 17 or p[0] != 'T' or p[1] != '3':
//           return None
//       return {"seq": int(p[2]),   "pitch": float(p[3]),
//               "slope": SLOPE[int(p[4])],
//               "fsr1": int(p[5]),  "fsr2": int(p[6]),
//               "handle": int(p[7]),"belt": int(p[8]),
//               "mode": int(p[9]),  "pwm": int(p[10]),
//               "motor": MOTOR[int(p[11])],
//               "us_l": int(p[12]), "us_c": int(p[13]), "us_r": int(p[14]),
//               "risk": RISK[int(p[15])], "hazard": HAZARD[int(p[16])]}
// ============================================================

// 0으로 바꾸면 센서 판단과 Serial 출력만 하고 모터에는 출력하지 않는다.
#define MOTOR_OUTPUT_ENABLED 1

// 1 = 양쪽 FSR을 모두 잡아야 '손잡이 잡음'. 0 = 한쪽만 잡아도 인정.
#define HANDLE_REQUIRE_BOTH 0

// 1로 바꾸면 전방 장애물이 US_OBSTACLE_BRAKE_MM보다 가까울 때 자동 제동.
// 기본 0 = 초음파는 측정과 텔레메트리 전송만 한다.
// 주의: 이 세 센서는 노면을 보도록 아래로 기울여 달았다. 평지에서 나오는
// 값이 이미 US_MOUNT_HEIGHT_M / sin(US_TILT_DEG) (기본 설정에서 161mm)라
// 임계값을 그보다 확실히 낮게(기본 80mm, 평지 반사값의 절반) 잡지 않으면
// 평지에서 계속 제동이 걸린다. 앞을 보는 장애물 감지가 필요하면 수평 센서를 따로 다는 편이 맞다.
// 실차 시험 전에 임계값을 반드시 현장에서 확인하고 켤 것.
#define US_OBSTACLE_BRAKE_ENABLED 0

// 원본 detect.py는 홈 상태에서 너비 판정을 먼저 하고, 너비가 CAUTION 구간
// (0.5*safe_gap 이상)에 들어가면 그 프레임에서는 복귀 판정을 아예 건너뛴다.
// 그래서 홈이 끝나 원래 높이로 돌아와도 in_hole이 풀리지 않고, 다음 프레임에
// 너비가 safe_gap을 넘으면서 반드시 STAIR(DANGER)가 된다. 시뮬레이션에서
// 3cm 홈이 상수를 어떻게 잡아도 전부 계단으로 보고된 원인이 이것이라,
// CAUTION은 DANGER 직전 한 프레임짜리 상태로만 존재한다.
//   1 = 복귀/턱 판정을 너비 판정보다 먼저 해서, 홈이 끝나면 그 자리에서
//       지나온 너비로 SAFE/CAUTION/DANGER를 매기고 홈 상태를 닫는다.
//   0 = 원본 detect.py와 완전히 같은 순서(위 성질을 그대로 감수).
#define HOLE_EXIT_CHECK_FIRST 1

// 초음파는 노면 상태에 따라 에코가 통째로 빠지는 프레임이 섞인다.
//   1 = 직전 유효 측정값을 들고 있다가 다음 유효 측정과 비교하고, 놓친
//       시간만큼 홈 너비도 이어서 적분한다. 실패 한 번에 판정이 두 프레임
//       먹통이 되지 않는다.
//   0 = 원본 detect.py처럼 이전 값을 버린다(실패 다음 프레임도 판정 없음).
#define US_KEEP_PREV_ON_DROPOUT 1

// 초음파는 다중 반사 때문에 가끔 한 프레임만 크게 튀는 값(헛에코)을 낸다.
// 시뮬레이션에서 과잉경보의 사실상 전부가 이 한 프레임짜리 스파이크였다
// (헛에코를 0으로 두면 과잉경보 450건 -> 0건). 판정에 넣기 전에 최근 유효
// 측정 3개의 중앙값을 쓰면 고립된 스파이크가 그대로 걸러진다.
//   1 = 3점 중앙값 필터 사용. 진짜 단차는 한 프레임(약 45ms) 늦게 잡힌다.
//   0 = 원본처럼 측정값을 그대로 판정에 넣는다.
// 텔레메트리로 나가는 us_l/us_c/us_r과 장애물 제동은 원래 측정값을 쓴다.
#define US_MEDIAN_FILTER 1

// 1로 두면 위험 판정이 바뀔 때마다 사람이 읽을 수 있는 줄을 하나 더 낸다.
// 시리얼 모니터에서 눈으로 확인하는 용도라 평소에는 0으로 꺼 둔다.
// '#'로 시작해서 파이/앱 파서는 어차피 무시하지만, 9600 baud에서 한 줄에
// 약 60ms가 들기 때문에 굳이 흘려보낼 이유가 없다. 현장에서 눈으로
// 확인해야 할 때만 1로 바꿔 굽는다.
//   # 12.345s DANGER HOLE  C dist=250mm width=45mm depth=150mm
//   # 13.900s SAFE   CLEAR
#define HUMAN_READABLE_LOG 0

// 노면 위험 판정은 모터에 개입하지 않는다. 위험도/위험원인은 앱으로만 간다.
// 1 = IMU 가속도 적분으로 속도를 추정. 0 = SPEED_FIXED_MPS 고정값 사용.
// 홈의 너비는 속도 * 시간으로 쌓이므로, 실험 초반에 속도 추정이 불안하면
// 0으로 두고 밀고 다니는 평균 속도를 SPEED_FIXED_MPS에 넣는 편이 낫다.
#define SPEED_FROM_IMU 1

// 텔레메트리는 아두이노 -> 파이/앱 단방향이다. 속도는 이 보드의 IMU에서
// 직접 얻으므로 파이가 아두이노로 보내주는 값은 없다.
// 파이의 app_3_12.py도 같은 값이어야 한다(현재 9600).
const unsigned long SERIAL_BAUD = 9600;

// ===================== 핀 연결 =====================
// ZS-X11H V1 제어 3선 (모터당 PWM / DIR / BRAKE)
const uint8_t LEFT_MOTOR_PWM_PIN = 5;      // 왼쪽 ZS-X11H PWM
const uint8_t LEFT_MOTOR_DIR_PIN = 6;      // 왼쪽 ZS-X11H DIR
const uint8_t LEFT_MOTOR_BRAKE_PIN = 9;    // 왼쪽 ZS-X11H BRAKE
const uint8_t RIGHT_MOTOR_PWM_PIN = 7;     // 오른쪽 ZS-X11H PWM
const uint8_t RIGHT_MOTOR_DIR_PIN = 8;     // 오른쪽 ZS-X11H DIR
const uint8_t RIGHT_MOTOR_BRAKE_PIN = 10;  // 오른쪽 ZS-X11H BRAKE
const uint8_t BELT_HALL_PIN = 28;          // SEN080603 S/OUT
const uint8_t HANDLE_FSR1_PIN = A0;        // 손잡이 FSR 1 분압 중간점
const uint8_t HANDLE_FSR2_PIN = A1;        // 손잡이 FSR 2 분압 중간점

// HC-SR04 3개. 센서 VCC/GND는 기판 +5V 레일에서, TRIG/ECHO는 메가에 직결.
// Mega는 5V 로직이라 ECHO에 전압분배가 필요 없다.
const uint8_t US_COUNT = 3;
const uint8_t US_TRIG_PINS[US_COUNT] = { 22, 24, 26 };  // 좌, 중, 우
const uint8_t US_ECHO_PINS[US_COUNT] = { 23, 25, 27 };

// ===================== IMU 설정 =====================
const uint8_t MPU_ADDR = 0x68;
const float ACCEL_SENSITIVITY = 16384.0;  // +/-2g
const float GYRO_SENSITIVITY = 131.0;     // +/-250 deg/s
const float FILTER_TIME_CONSTANT_S = 0.35;

// 현재 프로젝트에서 확인한 세로 90도 장착 기준:
// 전후축=-센서 Z, 좌우축=센서 Y, 수직축=센서 X, Pitch gyro=센서 Y
// 유모차 앞부분을 들었는데 DOWN으로 나오면 true로 바꾼다.
const bool INVERT_PITCH_DIRECTION = false;

// ===================== 센서 판단값 =====================
const float FLAT_THRESHOLD_DEG = 5.0;
const float SLOPE_THRESHOLD_DEG = 8.0;
const uint8_t REQUIRED_SLOPE_COUNT = 3;

// 실제 손잡이에서 텔레메트리의 fsr1/fsr2 값을 본 뒤 조정한다.
// handle_sensor.ino는 임계값 3에 반전 논리를 썼는데, 그건 분압 배선이
// 다른 시험용 값이다. 여기서는 stroller_control.ino 쪽 기준을 따른다.
const int FSR_GRIP_ON_THRESHOLD = 250;
const int FSR_GRIP_OFF_THRESHOLD = 150;
const unsigned long BELT_DEBOUNCE_MS = 50;

// ===================== 초음파 측정값 =====================
const unsigned long US_PING_INTERVAL_MS = 15;    // 센서 간 간격 -> 센서당 45ms
const unsigned long US_RISE_TIMEOUT_US = 3000;   // 에코 상승 대기 한계
const unsigned long US_MAX_ECHO_US = 6000;       // 약 1 m 왕복
const uint16_t US_MIN_VALID_MM = 20;             // HC-SR04 최소 측정 거리
const uint16_t US_MAX_VALID_MM = 1000;           // 노면용으로 좁힌 상한
const uint16_t US_OBSTACLE_BRAKE_MM = 80;        // 자동 제동 임계(옵션)

// ===================== 초음파 장착 형상 =====================
// 센서별로 자유롭게 바꾼다. 좌 / 중 / 우 순서.
//   US_MOUNT_HEIGHT_M : 노면에서 센서까지의 높이 (m)
//   US_TILT_DEG       : 수평면 기준 아래로 숙인 각도 (도). 90 = 정확히 아래
// 각도는 빔이 노면에 그리는 띠의 길이를 좌우한다(아래 US_BEAM_HALF_ANGLE_DEG).
// 부팅 때 E,US_BEAM,<빔길이mm>,<전방주시mm> 로 실제 값을 찍어 준다.
// 세 개를 서로 다르게 달아도 되고, 값만 여기서 고치면 된다.
// 각도가 0에 가까우면(수평) 노면을 보지 않는 셈이라 노면 판정이 무의미하다.
// 그래서 sin은 US_MIN_TILT_SIN 아래로 내려가지 않게 막는다.
float US_MOUNT_HEIGHT_M[US_COUNT] = { 0.16, 0.16, 0.16 };
// 좌우는 급하게(빔 띠 51mm, 바퀴 경로의 홈까지), 중앙은 얕게(빔 띠 86mm,
// 대신 전방 16cm를 봐서 턱/단차를 0.27초 미리 잡는다). 턱과 단차는 가로로
// 길게 이어져 있어 중앙 하나로 충분하고, 홈은 국소적이라 바퀴 경로인
// 좌우에서 봐야 한다.
float US_TILT_DEG[US_COUNT] = { 65.0, 45.0, 65.0 };
const float US_MIN_TILT_SIN = 0.0175;  // 약 1도

// HC-SR04의 유효 빔 반각. 초음파는 선이 아니라 원뿔이라 노면을 띠로 비추고,
// 거리계는 그 띠 안에서 가장 먼저 돌아온 에코(= 최단 거리)를 값으로 낸다.
// 띠의 길이 = h/tan(각도-반각) - h/tan(각도+반각) 이고, 이 길이가
//   - 감지할 수 있는 최소 홈 너비이며 (띠가 통째로 홈 안에 들어가야 보인다)
//   - 홈의 겉보기 너비를 그만큼 짧게 만든다.
// 그래서 홈에 진입할 때 이 길이를 미리 더해 준다(usBeamFootprintM).
const float US_BEAM_HALF_ANGLE_DEG = 7.5;

// 노면으로 인정할 측정값의 범위. 평지 기대값(height / sin(tilt))에 대한 비율.
// 이 밖의 값은 노면이 아니라고 보고 '측정 실패'로 처리한다(원본의 None).
const float US_GROUND_MIN_RATIO = 0.15;
const float US_GROUND_MAX_RATIO = 3.00;

// ===================== 노면 위험 판정값 =====================
// 초음파는 빔 원뿔 안에서 '가장 먼저 돌아온 에코', 즉 최단 거리를 값으로
// 낸다. 그래서 평지에서는 빔의 가장 아래쪽 광선(내려다보는 각 = 설치각 +
// 빔반각)이 만드는 값 하나로 고정된다.
//     d0 = 설치높이 / sin(설치각 + 빔반각)
// 이 기준보다 길면 노면이 내려간 것(홈), 짧으면 올라온 것(턱)이다. 원본
// detect.py처럼 이전 프레임과의 차이를 볼 필요가 없다. 거리 편차를 노면
// 높이 변화로 바꾸는 계수는 k = sin(설치각 + 빔반각)이고,
//     노면 높이 변화 = (측정거리 - d0) * k        (+ 홈 / - 턱)
//
// 종류가 갈리면 상태별 하위 판정으로 들어간다.
//   홈 상태 : 빔 띠가 통째로 들어가야 홈이 보이기 시작하는데, 이 띠가
//             safe_gap(4.2cm)보다 길다(45도에서 86mm, 65도에서 51mm).
//             즉 보이는 홈은 이미 바퀴가 빠지는 크기라 바로 DANGER다.
//             너비 적분은 앱에 참고값으로 실어 보내는 용도로만 남겼다.
//   턱 상태 : 편차의 최대값을 추적한다. 경사로에 진입할 때도 전방을 보는
//             센서에는 노면이 올라오는 것처럼 잠깐 보이는데, 그 크기는
//                 전방주시거리 * tan(경사각)
//             뿐이라(45도/16cm에서 10도 경사면 28mm) 실제 턱의 수십 mm와
//             크기로 갈린다. 최대 편차가 STEP_DANGER_M에 못 미친 채 노면이
//             돌아오면 경사로나 잔요철로 보고 조용히 끝낸다.
const float STEP_ENTER_M = 0.020;    // 이만큼 올라오면 턱 판정 시작
const float STEP_DANGER_M = 0.040;   // 턱 높이가 이 이상이면 DANGER
const float HOLE_ENTER_M = 0.040;    // 이만큼 내려가면 홈으로 보고 DANGER
const float TERRAIN_EXIT_M = 0.010;  // 이 아래로 돌아오면 노면 복귀

const float WHEEL_WIDTH_M = 0.07;    // 앞바퀴 지름 (m)

// 안전하게 지날 수 있는 홈의 너비. 원본 detect.py와 같은 비율이다.
// 지금 설치(16cm, 45/65도)에서는 빔 띠가 이미 이 값보다 길어서 판정에
// 직접 쓰이지는 않고, 앱으로 보내는 너비 값의 해석 기준으로만 남는다.
const float HOLE_SAFE_GAP_RATIO = 0.6;
const float HOLE_SAFE_GAP_M = HOLE_SAFE_GAP_RATIO * WHEEL_WIDTH_M;

// 부팅 직후 이만큼의 유효 측정을 모아 평지 기준거리 d0를 실측으로 잡는다.
// 타이어 공기압이나 장착 오차로 실제 높이가 조금 달라도 흡수된다. 평균이
// 계산값과 너무 다르면(위험 지형 위에서 켠 경우) 계산값을 그대로 쓴다.
const uint8_t US_BASELINE_SAMPLES = 20;
const float US_BASELINE_MIN_RATIO = 0.7;
const float US_BASELINE_MAX_RATIO = 1.4;

// 측정 주기
//   센서 하나가 다시 측정될 때까지의 시간은 US_PING_INTERVAL_MS * US_COUNT,
//   기본값에서 45ms다. 판정이 절대 기준거리 방식으로 바뀌면서 한 프레임만
//   있어도 종류가 갈리므로, 예전처럼 속도에 따라 판정이 깨지는 한계는 없다.
//   다만 위험이 보이는 구간을 몇 프레임 안에 지나가느냐는 여전히 중요하다.
//   0.4 m/s에서 한 프레임에 18mm를 이동한다.
//
//   US_MAX_ECHO_US(6ms)는 측정 간격보다 짧게 잡아 두었다. 앞 센서의 늦은
//   에코가 다음 센서의 청취 구간에 들어와도 범위 밖으로 버려지고, 잘못된
//   거리 대신 '측정 실패'가 되어 판정이 안전한 쪽으로 넘어간다.

// 한 센서의 측정이 이 시간 이상 끊기면 연속성이 깨진 것으로 보고
// 그 센서의 판정 상태를 초기화한다.
const float US_STALE_INTERVAL_S = 0.25;
const float US_MIN_INTERVAL_S = 0.01;

// 위험도는 한 프레임짜리 판정이라 그대로 두면 500ms 텔레메트리에서 놓친다.
// 마지막 위험 판정을 이 시간만큼 유지해서 앱이 확실히 받게 한다.
const unsigned long RISK_HOLD_MS = 1500;

// ===================== 속도 추정 (IMU) =====================
// 파이썬에서는 시리얼로 받던 speed를 여기서는 같은 보드의 IMU로 만든다.
// 전후 가속도에서 중력 성분을 빼고 적분하되, 적분 드리프트를 누설
// (SPEED_LEAK_PER_S)로 계속 갉아 준다. 정밀한 속도계가 아니라 홈 너비를
// 쌓는 용도의 근사값이다. 실측하며 아래 상수를 맞춘다.
// 가속도만으로는 등속 구간의 속도를 유지할 수 없다(가속도가 0이라 정보가
// 없다). 그래서 세 가지를 겹쳐 쓴다.
//   1) 느린 저역통과로 가속도 바이어스와 피치 오차를 빼낸다.
//   2) 빠른 저역통과와의 차이(진동)로 '움직이는 중'인지 판단한다.
//      절대값이 아니라 진동을 보므로 경사로에 세워 둬도 오판하지 않는다.
//   3) 적분값은 실측 평균 밀기 속도(SPEED_NOMINAL_MPS)로 수렴시킨다.
//
// 시뮬레이션 결과를 그대로 적자면, 수렴 속도(SPEED_LEAK_PER_S)를 올릴수록
// 오차가 줄었다. 즉 MPU6050 하나로는 적분이 보태 주는 정보가 사실상 없고,
// 이 추정기는 실질적으로 '이동 중이면 SPEED_NOMINAL_MPS, 서 있으면 0'에
// 가깝게 동작한다(순항 RMS 오차 약 0.15 m/s, 그 대부분이 실제 속도와
// SPEED_NOMINAL_MPS의 차이다). 정지 판정은 잘 맞아서 서 있을 때 홈 너비가
// 헛되이 쌓이지는 않는다.
//   -> 그래서 SPEED_NOMINAL_MPS를 실제로 밀어 보고 잰 평균 속도로 맞추는
//      것이 이 파일에서 가장 중요한 실측 작업이다. 홈 너비는 속도 x 시간
//      이라 속도 오차가 그대로 너비 오차가 된다.
//   -> 바퀴에 엔코더나 홀센서를 달 수 있으면 그쪽이 훨씬 정확하다.
const float GRAVITY_MPS2 = 9.80665;
const float SPEED_NOMINAL_MPS = 0.40;             // 실측 평균 밀기 속도
const float SPEED_BIAS_TC_S = 1.0;                // 바이어스 추정 시정수
const float SPEED_VIB_TC_S = 0.15;                // 진동 성분 추출 시정수
const float SPEED_MOTION_DEADBAND_MPS2 = 0.10;    // 이보다 큰 진동이면 이동 중
const float SPEED_LEAK_PER_S = 2.0;               // 평균 속도로 수렴하는 속도
const unsigned long SPEED_ZERO_HOLD_MS = 400;     // 진동이 끊기면 정지로 간주
const float SPEED_MAX_MPS = 3.0;                  // 추정 상한
const float SPEED_FIXED_MPS = 0.40;               // SPEED_FROM_IMU 0일 때 사용
const float SPEED_FALLBACK_MPS = 0.40;            // IMU 실패 시 가정 속도

// ===================== 모터 시험값 =====================
// PWM_MIN_DRIVE는 구간별 시험에서 찾은 확실히 회전하는 최소값으로 수정한다.
const uint8_t PWM_MIN_DRIVE = 120;
const uint8_t PWM_MAX_ASSIST = 200;
const float PWM_START_SLOPE_DEG = 8.0;
const float PWM_MAX_SLOPE_DEG = 15.0;
const uint8_t PWM_RISE_STEP = 2;
const uint8_t PWM_FALL_STEP = 6;

// ZS-X11H의 DIR은 액티브 로우다. 전진에 해당하는 레벨을 여기서 정한다.
// 좌/우 중 반대로 도는 쪽만 LOW/HIGH를 바꾼다.
const uint8_t LEFT_FORWARD_DIR_LEVEL = LOW;
const uint8_t RIGHT_FORWARD_DIR_LEVEL = LOW;

// 출력 모드 전환 시 드라이버가 상태를 정리할 시간(us).
const unsigned int MOTOR_TRANSITION_BLANK_US = 100;

// ===================== 실행 주기 =====================
const unsigned long IMU_INTERVAL_MS = 20;
const unsigned long INPUT_INTERVAL_MS = 20;
const unsigned long SLOPE_INTERVAL_MS = 100;
const unsigned long CONTROL_INTERVAL_MS = 20;
const unsigned long TELEMETRY_INTERVAL_MS = 500;
const uint8_t INITIAL_SETUP_SECONDS = 5;

// 텔레메트리 코드값은 파이 파서와 맞물려 있다. 순서를 바꾸지 말 것.
enum SlopeState : uint8_t {
  SLOPE_FLAT = 0,
  SLOPE_UP = 1,
  SLOPE_DOWN = 2,
  SLOPE_UNCERTAIN = 3
};

enum ControlMode : uint8_t {
  MODE_INITIALIZING = 0,
  MODE_SENSOR_FAULT = 1,
  MODE_HANDLE_RELEASED = 2,
  MODE_DOWNHILL_BRAKE = 3,
  MODE_UPHILL_ASSIST = 4,
  MODE_FLAT = 5,
  MODE_UNCERTAIN = 6,
  MODE_OBSTACLE_BRAKE = 7
};

enum MotorOutput : uint8_t {
  MOTOR_COAST = 0,
  MOTOR_FORWARD = 1,
  MOTOR_BRAKE = 2
};

enum UsPhase : uint8_t {
  US_IDLE,
  US_WAIT_RISE,
  US_WAIT_FALL
};

// detect.py의 Risk / Hazard IntEnum과 값이 같다.
enum RiskLevel : uint8_t {
  RISK_SAFE = 0,
  RISK_CAUTION = 1,
  RISK_DANGER = 2
};

// 원본 detect.py는 STEP / WIDE_HOLE / EXIT_STEP / STAIR 네 가지였는데,
// 앱에서 쓸 구분은 결국 '노면이 올라왔는가(턱)'와 '내려갔는가(홈)' 둘이라
// 두 가지로 합쳤다.
//   STEP <- 턱, 홈에서 빠져나오며 원래 높이보다 높아진 턱
//   HOLE <- 지나갈 수 없는 넓은 홈, 계단/단차
enum HazardCause : uint8_t {
  HAZARD_NONE = 0,
  HAZARD_STEP = 1,  // 턱: 노면이 갑자기 올라옴
  HAZARD_HOLE = 2   // 홈: 노면이 갑자기 내려가고 안전 너비를 넘음
};

// 센서 하나의 노면 판정 상태.
enum TerrainState : uint8_t {
  TERRAIN_IDLE = 0,  // 평지
  TERRAIN_HOLE = 1,  // 홈을 지나는 중
  TERRAIN_STEP = 2   // 노면이 올라옴 (턱인지 경사로인지 판별 중)
};

struct DangerDetector {
  TerrainState state;
  float holeWidthM;        // 홈 상태에서 누적한 너비 (참고값)
  float holeDepthM;        // 홈의 최대 깊이
  float stepPeakM;         // 턱 상태에서 본 최대 높이
  unsigned long lastSampleAtMs;
  uint16_t recentMm[3];    // 중앙값 필터용 최근 유효 측정
  uint8_t recentCount;
  float pendingIntervalS;  // 측정 실패로 건너뛴 시간 (다음 유효 프레임에 합산)
  float baselineSumM;      // 부팅 직후 기준거리 실측용
  uint8_t baselineCount;
  RiskLevel risk;          // 마지막 판정 (RISK_HOLD_MS 동안 유지)
  HazardCause hazard;
  unsigned long riskAtMs;
  // 판정이 난 그 프레임의 값. 상태를 지우기 전에 잡아 둔다.
  // hazard가 STEP이면 riskDepthM은 홈 깊이가 아니라 턱 높이다.
  float riskWidthM;
  float riskDepthM;
};

// ===================== IMU 값 =====================
int16_t ax = 0, ay = 0, az = 0;
int16_t gx = 0, gy = 0, gz = 0;
float gyroYOffsetDps = 0.0;
float pitchMountOffsetDeg = 0.0;
float accelPitchDeg = 0.0;
float filteredPitchDeg = 0.0;
float pitchRateDps = 0.0;
bool imuReady = false;
uint8_t consecutiveImuFailures = 0;

// ===================== 속도 추정 상태 =====================
float linearAccelMps2 = 0.0;
float accelBiasMps2 = 0.0;
float accelVibLpMps2 = 0.0;
float estimatedSpeedMps = 0.0;
unsigned long lastAccelActiveAtMs = 0;

// ===================== 입력/제어 상태 =====================
int hallRaw = HIGH;
bool beltFastened = false;
bool lastRawBeltFastened = false;
unsigned long hallRawChangedAt = 0;

int fsr1Value = 0;
int fsr2Value = 0;
bool grip1 = false;
bool grip2 = false;
bool handleHeld = false;

SlopeState slopeState = SLOPE_FLAT;
SlopeState previousSlopeCandidate = SLOPE_FLAT;
uint8_t slopeStableCount = 0;
ControlMode controlMode = MODE_INITIALIZING;
MotorOutput motorOutput = MOTOR_COAST;
uint8_t targetPWM = 0;
uint8_t currentPWM = 0;
uint8_t telemetrySeq = 0;

// ===================== 초음파 상태 =====================
UsPhase usPhase = US_IDLE;
uint8_t usIndex = 0;
unsigned long usPingStartedAtMs = 0;
unsigned long usTrigAtUs = 0;
unsigned long usEchoStartUs = 0;
uint16_t usDistanceMm[US_COUNT] = { 0, 0, 0 };

// 설치 각도/높이에서 미리 계산해 두는 값.
float usSinTilt[US_COUNT];
float usExpectedFlatM[US_COUNT];
float usBeamFootprintM[US_COUNT];  // 빔이 노면에 그리는 띠의 길이
float usLookAheadM[US_COUNT];      // 센서 바로 아래에서 빔 중심까지의 거리
float usBaselineM[US_COUNT];       // 평지에서 나오는 기준 거리 d0
float usHeightGainM[US_COUNT];     // 거리 편차 -> 노면 높이 변화 계수 k

DangerDetector detectors[US_COUNT];

// 세 센서를 합친 결과. 앱으로 나가는 값이다.
RiskLevel overallRisk = RISK_SAFE;
HazardCause overallHazard = HAZARD_NONE;
uint8_t overallRiskSensor = 255;
RiskLevel reportedRisk = RISK_SAFE;
HazardCause reportedHazard = HAZARD_NONE;

unsigned long lastImuAt = 0;
unsigned long lastInputAt = 0;
unsigned long lastSlopeAt = 0;
unsigned long lastControlAt = 0;
unsigned long lastTelemetryAt = 0;

bool baselineReported = false;

bool writeRegister(uint8_t reg, uint8_t value);
bool readRegisters(uint8_t startReg, uint8_t *data, uint8_t length);
bool initializeIMU();
bool readIMU();
bool calibrateIMU();
void calculateIMUValues();
void updateIMU(unsigned long now);
void updateSpeedEstimate(float dt, unsigned long now);
float currentSpeedMps();
void updateInputs(unsigned long now);
void updateUltrasonic(unsigned long now);
void finishPing();
uint16_t pulseToMm(unsigned long pulseUs);
bool obstacleTooClose();
void setupTerrainDetectors();
void resetDetector(uint8_t index);
float terrainDeviationM(uint8_t index, uint16_t distanceMm);
void runTerrainDetector(uint8_t index, uint16_t distanceMm, unsigned long now);
void updateOverallRisk(unsigned long now);
void outputHazardLine();
void outputHumanLine();
SlopeState classifySlope(float pitchDeg);
void updateConfirmedSlope(SlopeState candidate);
void updateControlMode();
uint8_t calculateSlopePWM(float pitchDeg);
void updatePwmRamp();
void commandMotor(MotorOutput requestedOutput, uint8_t pwm);
void writeOneMotor(uint8_t pwmPin, uint8_t dirPin, uint8_t brakePin,
                   uint8_t forwardDirLevel, MotorOutput output, uint8_t pwm);
void outputTelemetry();

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_BRAKE_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_BRAKE_PIN, OUTPUT);
  pinMode(BELT_HALL_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  for (uint8_t i = 0; i < US_COUNT; i++) {
    pinMode(US_TRIG_PINS[i], OUTPUT);
    digitalWrite(US_TRIG_PINS[i], LOW);
    pinMode(US_ECHO_PINS[i], INPUT);
  }

  setupTerrainDetectors();

  // 부팅/보정 중에는 모터 전자제동.
  // ZS-X11H는 BRAKE가 액티브 하이이므로 PWM을 먼저 끄고 BRAKE를 올린다.
  digitalWrite(LEFT_MOTOR_PWM_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_PWM_PIN, LOW);
  digitalWrite(LEFT_MOTOR_DIR_PIN, LEFT_FORWARD_DIR_LEVEL);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, RIGHT_FORWARD_DIR_LEVEL);
  digitalWrite(LEFT_MOTOR_BRAKE_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_BRAKE_PIN, HIGH);
  motorOutput = MOTOR_BRAKE;
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin();
  Wire.setClock(100000);

  Serial.println(F("E,BOOT"));
  for (uint8_t second = 0; second < INITIAL_SETUP_SECONDS; second++) {
    Serial.print(F("E,CAL,"));
    Serial.println(INITIAL_SETUP_SECONDS - second);
    delay(1000);
  }

  if (initializeIMU() && calibrateIMU()) {
    imuReady = true;
    calculateIMUValues();
    filteredPitchDeg = accelPitchDeg;
    Serial.println(F("E,IMU_READY"));
  } else {
    imuReady = false;
    Serial.println(F("E,IMU_FAIL"));
  }

  // 센서별 빔 기하: 노면에 그리는 띠 길이와 전방 주시 거리 (mm)
  for (uint8_t i = 0; i < US_COUNT; i++) {
    Serial.print(F("E,US_BEAM,"));
    Serial.print(i);
    Serial.print(',');
    Serial.print((int)(usBeamFootprintM[i] * 1000.0));
    Serial.print(',');
    Serial.println((int)(usLookAheadM[i] * 1000.0));
  }

  unsigned long now = millis();
  updateInputs(now);
  controlMode = imuReady ? MODE_FLAT : MODE_SENSOR_FAULT;
  lastImuAt = now;
  lastInputAt = now;
  lastSlopeAt = now;
  lastControlAt = now;
  lastTelemetryAt = now;
  lastAccelActiveAtMs = now;
  usPingStartedAtMs = now;
  for (uint8_t i = 0; i < US_COUNT; i++) {
    detectors[i].lastSampleAtMs = now;
    detectors[i].riskAtMs = now;
  }
  Serial.println(F("E,READY"));
}

void loop() {
  unsigned long now = millis();

  if (imuReady && now - lastImuAt >= IMU_INTERVAL_MS) {
    updateIMU(now);
  }

  if (now - lastInputAt >= INPUT_INTERVAL_MS) {
    lastInputAt = now;
    updateInputs(now);
  }

  // 초음파는 매 loop마다 상태를 진행시킨다. 블로킹은 트리거 10us뿐.
  // 한 센서의 측정이 끝나면 그 자리에서 노면 위험 판정 한 프레임이 돈다.
  updateUltrasonic(now);

  // 평지 기준거리 보정이 끝나면 실측값을 한 번 알린다.
  if (!baselineReported) {
    bool done = true;
    for (uint8_t i = 0; i < US_COUNT; i++) {
      if (detectors[i].baselineCount < US_BASELINE_SAMPLES) done = false;
    }
    if (done && usPhase == US_IDLE) {
      baselineReported = true;
      Serial.print(F("E,US_BASE,"));
      Serial.print((int)(usBaselineM[0] * 1000.0));
      Serial.print(',');
      Serial.print((int)(usBaselineM[1] * 1000.0));
      Serial.print(',');
      Serial.println((int)(usBaselineM[2] * 1000.0));
    }
  }

  // 위험 판정 유지시간이 지났는지 확인하고 세 센서를 합친다.
  updateOverallRisk(now);

  // 위험도나 위험원인이 바뀌면 텔레메트리 주기를 기다리지 않고 바로 알린다.
  if ((overallRisk != reportedRisk || overallHazard != reportedHazard)
      && usPhase == US_IDLE) {
    outputHazardLine();
  }

  if (imuReady && now - lastSlopeAt >= SLOPE_INTERVAL_MS) {
    lastSlopeAt = now;
    updateConfirmedSlope(classifySlope(filteredPitchDeg));
  }

  if (now - lastControlAt >= CONTROL_INTERVAL_MS) {
    lastControlAt = now;
    updateControlMode();
    updatePwmRamp();

    bool immediateBrake = controlMode == MODE_SENSOR_FAULT
                          || controlMode == MODE_HANDLE_RELEASED
                          || controlMode == MODE_DOWNHILL_BRAKE
                          || controlMode == MODE_OBSTACLE_BRAKE;

    if (immediateBrake) {
      currentPWM = 0;
      commandMotor(MOTOR_BRAKE, 0);
    } else if (currentPWM > 0) {
      commandMotor(MOTOR_FORWARD, currentPWM);
    } else {
      commandMotor(MOTOR_COAST, 0);
    }

    // 내장 LED: 안전벨트 자석 감지 시 켜짐. 모터 제어에는 영향 없음.
    digitalWrite(LED_BUILTIN, beltFastened ? HIGH : LOW);
  }

  // 초음파 측정이 진행 중이 아닐 때만 보낸다. 에코 타이밍과 겹치지 않게 막는다.
  if (now - lastTelemetryAt >= TELEMETRY_INTERVAL_MS && usPhase == US_IDLE) {
    lastTelemetryAt = now;
    outputTelemetry();
  }
}

// ===================== IMU =====================
bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t startReg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t received = Wire.requestFrom(MPU_ADDR, length);
  if (received != length) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (uint8_t i = 0; i < length; i++) data[i] = Wire.read();
  return true;
}

bool initializeIMU() {
  if (!writeRegister(0x6B, 0x80)) return false;  // reset
  delay(100);
  if (!writeRegister(0x6B, 0x01)) return false;  // wake, gyro clock
  if (!writeRegister(0x1A, 0x03)) return false;  // DLPF
  if (!writeRegister(0x1B, 0x00)) return false;  // gyro +/-250 dps
  if (!writeRegister(0x1C, 0x00)) return false;  // accel +/-2g
  delay(100);
  return true;
}

bool readIMU() {
  uint8_t data[14];
  if (!readRegisters(0x3B, data, 14)) return false;

  ax = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  ay = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  az = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  gx = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
  gy = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
  gz = (int16_t)(((uint16_t)data[12] << 8) | data[13]);
  return true;
}

bool calibrateIMU() {
  const int CALIBRATION_SAMPLES = 200;
  long sumGy = 0;
  float sumPitch = 0.0;
  int validSamples = 0;
  int attempts = 0;

  while (validSamples < CALIBRATION_SAMPLES && attempts < 400) {
    attempts++;
    if (!readIMU()) {
      delay(5);
      continue;
    }

    float forwardG = -az / ACCEL_SENSITIVITY;
    float sideG = ay / ACCEL_SENSITIVITY;
    float verticalG = ax / ACCEL_SENSITIVITY;
    float rawPitch = atan2(forwardG, sqrt(sideG * sideG + verticalG * verticalG))
                     * 180.0 / PI;

    sumGy += gy;
    sumPitch += rawPitch;
    validSamples++;
    delay(5);
  }

  if (validSamples < CALIBRATION_SAMPLES) return false;
  gyroYOffsetDps = (sumGy / (float)validSamples) / GYRO_SENSITIVITY;
  pitchMountOffsetDeg = sumPitch / validSamples;
  return true;
}

void calculateIMUValues() {
  float forwardG = -az / ACCEL_SENSITIVITY;
  float sideG = ay / ACCEL_SENSITIVITY;
  float verticalG = ax / ACCEL_SENSITIVITY;

  accelPitchDeg = atan2(forwardG, sqrt(sideG * sideG + verticalG * verticalG))
                  * 180.0 / PI - pitchMountOffsetDeg;
  pitchRateDps = (gy / GYRO_SENSITIVITY) - gyroYOffsetDps;

  if (INVERT_PITCH_DIRECTION) {
    accelPitchDeg = -accelPitchDeg;
    pitchRateDps = -pitchRateDps;
  }
}

void updateIMU(unsigned long now) {
  float dt = (now - lastImuAt) / 1000.0;
  lastImuAt = now;
  if (dt < 0.001) dt = 0.001;
  if (dt > 0.100) dt = 0.100;

  if (!readIMU()) {
    if (consecutiveImuFailures < 255) consecutiveImuFailures++;
    if (consecutiveImuFailures >= 3) {
      imuReady = false;
      Serial.println(F("E,IMU_LOST"));
    }
    return;
  }

  consecutiveImuFailures = 0;
  calculateIMUValues();
  float alpha = FILTER_TIME_CONSTANT_S / (FILTER_TIME_CONSTANT_S + dt);
  filteredPitchDeg = alpha * (filteredPitchDeg + pitchRateDps * dt)
                     + (1.0 - alpha) * accelPitchDeg;

  updateSpeedEstimate(dt, now);
}

// 전후 가속도에서 중력 성분을 빼고 적분해 전진 속도를 추정한다.
// 파이썬 원본이 시리얼로 받던 speed를 대신하는 값이다.
void updateSpeedEstimate(float dt, unsigned long now) {
  float forwardG = -az / ACCEL_SENSITIVITY;

  // filteredPitchDeg는 장착 오프셋을 뺀(그리고 필요하면 부호를 뒤집은) 값이라
  // 중력 성분을 구하려면 센서가 실제로 보는 기울기로 되돌려야 한다.
  float mountedPitchDeg =
      (INVERT_PITCH_DIRECTION ? -filteredPitchDeg : filteredPitchDeg)
      + pitchMountOffsetDeg;
  float gravityForwardG = sin(mountedPitchDeg * PI / 180.0);

  float rawAccel = (forwardG - gravityForwardG) * GRAVITY_MPS2;
  if (INVERT_PITCH_DIRECTION) rawAccel = -rawAccel;

  // 느린 저역통과로 남은 바이어스(가속도계 오프셋 + 피치 오차)를 추정해 뺀다.
  float beta = dt / (SPEED_BIAS_TC_S + dt);
  accelBiasMps2 += beta * (rawAccel - accelBiasMps2);
  linearAccelMps2 = rawAccel - accelBiasMps2;

  // 이동 여부는 '고주파 진동'으로만 판단한다. 바이어스 제거용 저역통과는
  // 시정수가 길어서 감속 직후 한동안 잔차가 남는데, 그걸 이동으로 오판하지
  // 않도록 빠른 저역통과를 따로 두고 그 차이(진동)만 본다.
  float gamma = dt / (SPEED_VIB_TC_S + dt);
  accelVibLpMps2 += gamma * (rawAccel - accelVibLpMps2);
  float vibration = rawAccel - accelVibLpMps2;
  if (fabs(vibration) > SPEED_MOTION_DEADBAND_MPS2) {
    lastAccelActiveAtMs = now;
  }
  if (now - lastAccelActiveAtMs >= SPEED_ZERO_HOLD_MS) {
    estimatedSpeedMps = 0.0;  // 진동이 없다 = 서 있다
    return;
  }

  estimatedSpeedMps += linearAccelMps2 * dt;
  // 등속 구간에는 가속도에 정보가 없으므로 실측 평균 속도로 수렴시킨다.
  estimatedSpeedMps +=
      (SPEED_NOMINAL_MPS - estimatedSpeedMps) * SPEED_LEAK_PER_S * dt;

  // 유모차는 앞으로만 민다고 보고 음수는 0으로 잘라낸다.
  if (estimatedSpeedMps < 0.0) estimatedSpeedMps = 0.0;
  if (estimatedSpeedMps > SPEED_MAX_MPS) estimatedSpeedMps = SPEED_MAX_MPS;
}

float currentSpeedMps() {
#if SPEED_FROM_IMU
  // IMU가 죽었으면 홈 너비를 과소평가하지 않도록 보수적으로 가정한다.
  return imuReady ? estimatedSpeedMps : SPEED_FALLBACK_MPS;
#else
  return SPEED_FIXED_MPS;
#endif
}

// ===================== Hall + FSR =====================
void updateInputs(unsigned long now) {
  hallRaw = digitalRead(BELT_HALL_PIN);
  bool rawFastened = (hallRaw == LOW);  // 자석 감지 시 LOW

  if (rawFastened != lastRawBeltFastened) {
    lastRawBeltFastened = rawFastened;
    hallRawChangedAt = now;
  }
  if (rawFastened != beltFastened
      && now - hallRawChangedAt >= BELT_DEBOUNCE_MS) {
    beltFastened = rawFastened;
  }

  // FSR 2개를 각각 히스테리시스로 판정한다.
  // handle_sensor.ino의 10회 평균 + delay(2)는 20ms를 통째로 잡아먹어서
  // 제어 주기를 깨므로 가져오지 않았다.
  fsr1Value = analogRead(HANDLE_FSR1_PIN);
  fsr2Value = analogRead(HANDLE_FSR2_PIN);

  if (!grip1 && fsr1Value >= FSR_GRIP_ON_THRESHOLD) grip1 = true;
  if (grip1 && fsr1Value <= FSR_GRIP_OFF_THRESHOLD) grip1 = false;
  if (!grip2 && fsr2Value >= FSR_GRIP_ON_THRESHOLD) grip2 = true;
  if (grip2 && fsr2Value <= FSR_GRIP_OFF_THRESHOLD) grip2 = false;

#if HANDLE_REQUIRE_BOTH
  handleHeld = grip1 && grip2;
#else
  handleHeld = grip1 || grip2;
#endif
}

// ===================== 초음파 =====================
// 한 번에 한 센서만 쏘고, 에코를 폴링으로 기다린다. pulseIn을 쓰면 센서당
// 최대 30ms를 블로킹해서 3개면 90ms - 20ms 제어 주기가 무너진다.
void updateUltrasonic(unsigned long now) {
  switch (usPhase) {
    case US_IDLE:
      if (now - usPingStartedAtMs < US_PING_INTERVAL_MS) return;
      usPingStartedAtMs = now;
      digitalWrite(US_TRIG_PINS[usIndex], LOW);
      delayMicroseconds(2);
      digitalWrite(US_TRIG_PINS[usIndex], HIGH);
      delayMicroseconds(10);
      digitalWrite(US_TRIG_PINS[usIndex], LOW);
      usTrigAtUs = micros();
      usPhase = US_WAIT_RISE;
      break;

    case US_WAIT_RISE:
      if (digitalRead(US_ECHO_PINS[usIndex]) == HIGH) {
        usEchoStartUs = micros();
        usPhase = US_WAIT_FALL;
      } else if (micros() - usTrigAtUs > US_RISE_TIMEOUT_US) {
        usDistanceMm[usIndex] = 0;  // 센서 응답 없음
        finishPing();
      }
      break;

    case US_WAIT_FALL:
      if (digitalRead(US_ECHO_PINS[usIndex]) == LOW) {
        usDistanceMm[usIndex] = pulseToMm(micros() - usEchoStartUs);
        finishPing();
      } else if (micros() - usEchoStartUs > US_MAX_ECHO_US) {
        usDistanceMm[usIndex] = 0;  // 측정 범위 밖
        finishPing();
      }
      break;
  }
}

void finishPing() {
  // 이 센서의 한 프레임이 완성됐으니 그 자리에서 노면 위험 판정을 돌린다.
  // 파이썬의 for prev, curr in zip(...) 한 바퀴에 해당한다.
  runTerrainDetector(usIndex, usDistanceMm[usIndex], millis());

  usIndex++;
  if (usIndex >= US_COUNT) usIndex = 0;
  usPhase = US_IDLE;
}

uint16_t pulseToMm(unsigned long pulseUs) {
  if (pulseUs == 0 || pulseUs > US_MAX_ECHO_US) return 0;
  // 음속 343 m/s = 0.343 mm/us, 왕복이므로 절반.
  unsigned long mm = (pulseUs * 343UL) / 2000UL;
  if (mm < US_MIN_VALID_MM || mm > US_MAX_VALID_MM) return 0;
  return (uint16_t)mm;
}

bool obstacleTooClose() {
  for (uint8_t i = 0; i < US_COUNT; i++) {
    // 0은 '측정 실패'라서 장애물로 보지 않는다.
    if (usDistanceMm[i] > 0 && usDistanceMm[i] <= US_OBSTACLE_BRAKE_MM) {
      return true;
    }
  }
  return false;
}

// ===================== 노면 위험 판정 (detect.py 이식) =====================
void setupTerrainDetectors() {
  for (uint8_t i = 0; i < US_COUNT; i++) {
    float s = sin(US_TILT_DEG[i] * PI / 180.0);
    if (s < US_MIN_TILT_SIN) s = US_MIN_TILT_SIN;  // 수평 장착 방어
    usSinTilt[i] = s;
    usExpectedFlatM[i] = US_MOUNT_HEIGHT_M[i] / s;  // 평지에서 나와야 할 거리

    // 빔이 노면에 그리는 띠. 각도가 얕아 빔 위쪽이 지평선을 향하면
    // (각도 <= 반각) 노면을 제대로 못 보는 장착이라 크게 잡아 둔다.
    float nearDeg = US_TILT_DEG[i] + US_BEAM_HALF_ANGLE_DEG;
    float farDeg = US_TILT_DEG[i] - US_BEAM_HALF_ANGLE_DEG;
    float nearM = US_MOUNT_HEIGHT_M[i] / tan(nearDeg * PI / 180.0);
    if (farDeg < 1.0) {
      usBeamFootprintM[i] = 10.0;
    } else {
      usBeamFootprintM[i] = US_MOUNT_HEIGHT_M[i] / tan(farDeg * PI / 180.0) - nearM;
    }
    usLookAheadM[i] = US_MOUNT_HEIGHT_M[i] / tan(US_TILT_DEG[i] * PI / 180.0);

    // 평지 기준거리와, 거리 편차를 노면 높이로 바꾸는 계수.
    // 빔의 가장 아래쪽 광선이 최단 거리를 만든다.
    float nearSin = sin(nearDeg * PI / 180.0);
    if (nearSin < US_MIN_TILT_SIN) nearSin = US_MIN_TILT_SIN;
    usBaselineM[i] = US_MOUNT_HEIGHT_M[i] / nearSin;
    usHeightGainM[i] = nearSin;

    resetDetector(i);
  }
}

// 판정 상태를 모두 버린다. 기준거리 실측값도 다시 잡는다.
void resetDetector(uint8_t index) {
  DangerDetector &d = detectors[index];
  d.state = TERRAIN_IDLE;
  d.holeWidthM = 0.0;
  d.holeDepthM = 0.0;
  d.stepPeakM = 0.0;
  d.pendingIntervalS = 0.0;
  d.recentCount = 0;
  d.recentMm[0] = d.recentMm[1] = d.recentMm[2] = 0;
  d.baselineSumM = 0.0;
  d.baselineCount = 0;
  d.lastSampleAtMs = millis();
  d.risk = RISK_SAFE;
  d.hazard = HAZARD_NONE;
  d.riskAtMs = d.lastSampleAtMs;
  d.riskWidthM = 0.0;
  d.riskDepthM = 0.0;
}

// 측정 거리(mm) -> 평지 기준 대비 노면 높이 변화(m).
//   + 노면이 내려감 (홈) / - 노면이 올라옴 (턱)
// 노면으로 볼 수 없는 값이면 NAN을 돌려준다(원본의 None에 해당).
float terrainDeviationM(uint8_t index, uint16_t distanceMm) {
  if (distanceMm == 0) return NAN;

  float distanceM = distanceMm / 1000.0;
  if (distanceM < usExpectedFlatM[index] * US_GROUND_MIN_RATIO
      || distanceM > usExpectedFlatM[index] * US_GROUND_MAX_RATIO) {
    return NAN;
  }
  return (distanceM - usBaselineM[index]) * usHeightGainM[index];
}

// 세 값의 중앙값. 한 프레임짜리 헛에코를 걸러낸다.
static uint16_t medianOf3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

// 센서 한 개분의 노면 판정. 절대 기준거리에서 벗어난 방향으로 종류를 먼저
// 가르고(위 판정값 주석 참고), 상태별 하위 판정으로 확정한다.
void runTerrainDetector(uint8_t index, uint16_t distanceMm, unsigned long now) {
  DangerDetector &d = detectors[index];

  float interval = (now - d.lastSampleAtMs) / 1000.0 + d.pendingIntervalS;
  d.lastSampleAtMs = now;
  d.pendingIntervalS = 0.0;

  // 측정이 오래 끊겼으면 이어서 볼 근거가 없다. 상태를 버린다.
  if (interval > US_STALE_INTERVAL_S) {
    d.state = TERRAIN_IDLE;
    d.holeWidthM = 0.0;
    d.holeDepthM = 0.0;
    d.stepPeakM = 0.0;
    d.recentCount = 0;
    interval = US_STALE_INTERVAL_S;
  }
  if (interval < US_MIN_INTERVAL_S) interval = US_MIN_INTERVAL_S;

#if US_MEDIAN_FILTER
  // 유효 측정만 밀어 넣고, 3개가 모이면 중앙값을 판정에 쓴다.
  if (distanceMm != 0) {
    d.recentMm[2] = d.recentMm[1];
    d.recentMm[1] = d.recentMm[0];
    d.recentMm[0] = distanceMm;
    if (d.recentCount < 3) d.recentCount++;
    if (d.recentCount >= 3) {
      distanceMm = medianOf3(d.recentMm[0], d.recentMm[1], d.recentMm[2]);
    }
  }
#endif

  float dev = terrainDeviationM(index, distanceMm);
  if (isnan(dev)) {
    // 측정 실패. 이번 프레임은 판정하지 않는다.
#if US_KEEP_PREV_ON_DROPOUT
    d.pendingIntervalS = interval;  // 놓친 시간은 다음 유효 프레임에 넘긴다
#else
    d.state = TERRAIN_IDLE;
#endif
    d.recentCount = 0;              // 끊긴 뒤의 중앙값은 믿을 수 없다
    return;
  }

  // 부팅 직후에는 평지를 달린다고 보고 기준거리를 실측으로 잡는다.
  if (d.baselineCount < US_BASELINE_SAMPLES) {
    d.baselineSumM += distanceMm / 1000.0;
    d.baselineCount++;
    if (d.baselineCount >= US_BASELINE_SAMPLES) {
      float measured = d.baselineSumM / US_BASELINE_SAMPLES;
      float computed = US_MOUNT_HEIGHT_M[index] / usHeightGainM[index];
      // 위험 지형 위에서 켠 경우를 걸러낸다. 너무 다르면 계산값을 쓴다.
      if (measured > computed * US_BASELINE_MIN_RATIO
          && measured < computed * US_BASELINE_MAX_RATIO) {
        usBaselineM[index] = measured;
      }
    }
    return;  // 보정이 끝날 때까지는 판정하지 않는다
  }

  RiskLevel risk = RISK_SAFE;
  HazardCause hazard = HAZARD_NONE;
  float eventWidthM = 0.0;
  float eventDepthM = 0.0;

  switch (d.state) {
    case TERRAIN_IDLE:
      if (dev > HOLE_ENTER_M) {
        // 빔 띠가 safe_gap보다 길어서, 보이기 시작한 홈은 이미 바퀴가
        // 빠지는 크기다. 바로 알린다.
        d.state = TERRAIN_HOLE;
        d.holeDepthM = dev;
        d.holeWidthM = usBeamFootprintM[index];
        risk = RISK_DANGER;
        hazard = HAZARD_HOLE;
        eventWidthM = d.holeWidthM;
        eventDepthM = d.holeDepthM;
      } else if (-dev > STEP_ENTER_M) {
        // 턱일 수도, 경사로 진입일 수도 있다. 크기를 보고 정한다.
        d.state = TERRAIN_STEP;
        d.stepPeakM = -dev;
      }
      break;

    case TERRAIN_HOLE:
      d.holeWidthM += currentSpeedMps() * interval;   // 앱에 보낼 참고값
      if (dev > d.holeDepthM) d.holeDepthM = dev;
      if (dev < TERRAIN_EXIT_M) {                     // 노면 복귀
        d.state = TERRAIN_IDLE;
        d.holeWidthM = 0.0;
        d.holeDepthM = 0.0;
      }
      break;

    case TERRAIN_STEP:
      if (-dev > d.stepPeakM) d.stepPeakM = -dev;
      if (d.stepPeakM >= STEP_DANGER_M) {
        risk = RISK_DANGER;
        hazard = HAZARD_STEP;
        eventDepthM = d.stepPeakM;                    // 턱 높이
        d.state = TERRAIN_IDLE;
        d.stepPeakM = 0.0;
      } else if (-dev < TERRAIN_EXIT_M) {
        // 노면이 돌아왔고 크기가 작았다 = 경사로 진입이나 잔요철
        d.state = TERRAIN_IDLE;
        d.stepPeakM = 0.0;
      }
      break;
  }

  // 한 프레임짜리 판정이라 그대로 두면 텔레메트리에서 놓친다.
  // 더 높은 위험이면 갱신하고, 같은 위험이면 유지시간만 늘린다.
  if (risk >= d.risk) {
    d.risk = risk;
    d.hazard = hazard;
    d.riskAtMs = now;
    d.riskWidthM = eventWidthM;
    d.riskDepthM = eventDepthM;
  }
}

// 세 센서 중 가장 높은 위험도를 앱으로 보낼 값으로 삼는다.
// 원본의 max(max_risk, risk, key=lambda x: x[0].value)에 해당한다.
void updateOverallRisk(unsigned long now) {
  RiskLevel best = RISK_SAFE;
  HazardCause bestHazard = HAZARD_NONE;
  uint8_t bestSensor = 255;

  for (uint8_t i = 0; i < US_COUNT; i++) {
    // 유지시간이 지난 판정은 내린다.
    if (detectors[i].risk != RISK_SAFE
        && now - detectors[i].riskAtMs >= RISK_HOLD_MS) {
      detectors[i].risk = RISK_SAFE;
      detectors[i].hazard = HAZARD_NONE;
    }
    if (detectors[i].risk > best) {
      best = detectors[i].risk;
      bestHazard = detectors[i].hazard;
      bestSensor = i;
    }
  }

  overallRisk = best;
  overallHazard = bestHazard;
  overallRiskSensor = bestSensor;
}

// 위험도/위험원인이 바뀐 순간 앱으로 바로 나가는 한 줄.
void outputHazardLine() {
  uint8_t s = overallRiskSensor;
  uint16_t distanceMm = (s < US_COUNT) ? usDistanceMm[s] : 0;
  // 판정 당시의 값이다. STAIR/EXIT_STEP은 판정과 동시에 홈 상태를 지우므로
  // 현재 상태를 읽으면 항상 0이 나간다.
  uint16_t widthMm = (s < US_COUNT) ? (uint16_t)(detectors[s].riskWidthM * 1000.0) : 0;
  uint16_t depthMm = (s < US_COUNT) ? (uint16_t)(detectors[s].riskDepthM * 1000.0) : 0;

  Serial.print(F("H,"));
  Serial.print((uint8_t)overallRisk);
  Serial.print(',');
  Serial.print((uint8_t)overallHazard);
  Serial.print(',');
  Serial.print(s);
  Serial.print(',');
  Serial.print(distanceMm);
  Serial.print(',');
  Serial.print(widthMm);
  Serial.print(',');
  Serial.println(depthMm);

#if HUMAN_READABLE_LOG
  outputHumanLine();
#endif

  reportedRisk = overallRisk;
  reportedHazard = overallHazard;
}

#if HUMAN_READABLE_LOG
// 시리얼 모니터에서 눈으로 확인하는 줄. '#'로 시작하므로 파서는 무시한다.
void outputHumanLine() {
  static const char *RISK_NAME[] = { "SAFE  ", "CAUTION", "DANGER" };
  static const char *HAZARD_NAME[] = { "CLEAR", "STEP ", "HOLE " };
  static const char SENSOR_NAME[] = { 'L', 'C', 'R' };

  Serial.print(F("# "));
  Serial.print(millis() / 1000.0, 3);
  Serial.print(F("s "));
  Serial.print(RISK_NAME[overallRisk]);
  Serial.print(' ');
  Serial.print(HAZARD_NAME[overallHazard]);

  uint8_t s = overallRiskSensor;
  if (s >= US_COUNT) {
    Serial.println();
    return;
  }

  Serial.print(' ');
  Serial.print(SENSOR_NAME[s]);
  Serial.print(F(" dist="));
  Serial.print(usDistanceMm[s]);
  if (overallHazard == HAZARD_HOLE) {
    Serial.print(F("mm width="));
    Serial.print((int)(detectors[s].riskWidthM * 1000.0));
    Serial.print(F("mm depth="));
    Serial.print((int)(detectors[s].riskDepthM * 1000.0));
    Serial.println(F("mm"));
  } else {
    Serial.print(F("mm height="));
    Serial.print((int)(detectors[s].riskDepthM * 1000.0));
    Serial.println(F("mm"));
  }
}
#endif

// ===================== 판단 + 제어 =====================
SlopeState classifySlope(float pitchDeg) {
  if (pitchDeg >= SLOPE_THRESHOLD_DEG) return SLOPE_UP;
  if (pitchDeg <= -SLOPE_THRESHOLD_DEG) return SLOPE_DOWN;
  if (fabs(pitchDeg) <= FLAT_THRESHOLD_DEG) return SLOPE_FLAT;
  return SLOPE_UNCERTAIN;
}

void updateConfirmedSlope(SlopeState candidate) {
  if (candidate == previousSlopeCandidate) {
    if (slopeStableCount < 255) slopeStableCount++;
  } else {
    previousSlopeCandidate = candidate;
    slopeStableCount = 1;
  }

  if (slopeStableCount >= REQUIRED_SLOPE_COUNT) {
    slopeState = candidate;
    slopeStableCount = REQUIRED_SLOPE_COUNT;
  }
}

void updateControlMode() {
  targetPWM = 0;

  if (!imuReady) {
    controlMode = MODE_SENSOR_FAULT;
  } else if (!handleHeld) {
    controlMode = MODE_HANDLE_RELEASED;
#if US_OBSTACLE_BRAKE_ENABLED
  } else if (obstacleTooClose()) {
    controlMode = MODE_OBSTACLE_BRAKE;
#endif
  } else if (slopeState == SLOPE_DOWN) {
    controlMode = MODE_DOWNHILL_BRAKE;
  } else if (slopeState == SLOPE_UP) {
    controlMode = MODE_UPHILL_ASSIST;
    targetPWM = calculateSlopePWM(filteredPitchDeg);
  } else if (slopeState == SLOPE_FLAT) {
    controlMode = MODE_FLAT;
  } else {
    controlMode = MODE_UNCERTAIN;
  }
}

uint8_t calculateSlopePWM(float pitchDeg) {
  if (pitchDeg <= PWM_START_SLOPE_DEG) return PWM_MIN_DRIVE;
  if (pitchDeg >= PWM_MAX_SLOPE_DEG) return PWM_MAX_ASSIST;

  float ratio = (pitchDeg - PWM_START_SLOPE_DEG)
                / (PWM_MAX_SLOPE_DEG - PWM_START_SLOPE_DEG);
  return (uint8_t)(PWM_MIN_DRIVE
                   + ratio * (PWM_MAX_ASSIST - PWM_MIN_DRIVE));
}

void updatePwmRamp() {
  if (currentPWM < targetPWM) {
    int nextPWM = currentPWM + PWM_RISE_STEP;
    currentPWM = nextPWM > targetPWM ? targetPWM : (uint8_t)nextPWM;
  } else if (currentPWM > targetPWM) {
    int nextPWM = currentPWM - PWM_FALL_STEP;
    currentPWM = nextPWM < targetPWM ? targetPWM : (uint8_t)nextPWM;
  }
}

void commandMotor(MotorOutput requestedOutput, uint8_t pwm) {
#if !MOTOR_OUTPUT_ENABLED
  requestedOutput = MOTOR_COAST;
  pwm = 0;
#endif

  // 출력 모드가 바뀔 때 PWM을 먼저 0으로 떨어뜨려 전환 충돌을 줄인다.
  // BRAKE는 여기서 건드리지 않는다. 제동 중에 한순간이라도 풀리면 안 된다.
  if (requestedOutput != motorOutput) {
    digitalWrite(LEFT_MOTOR_PWM_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_PWM_PIN, LOW);
    delayMicroseconds(MOTOR_TRANSITION_BLANK_US);
    motorOutput = requestedOutput;
  }

  writeOneMotor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, LEFT_MOTOR_BRAKE_PIN,
                LEFT_FORWARD_DIR_LEVEL, requestedOutput, pwm);
  writeOneMotor(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN, RIGHT_MOTOR_BRAKE_PIN,
                RIGHT_FORWARD_DIR_LEVEL, requestedOutput, pwm);
}

void writeOneMotor(uint8_t pwmPin, uint8_t dirPin, uint8_t brakePin,
                   uint8_t forwardDirLevel, MotorOutput output, uint8_t pwm) {
  if (output == MOTOR_FORWARD) {
    // 제동을 먼저 풀고 방향을 정한 뒤 속도를 준다.
    digitalWrite(brakePin, LOW);
    digitalWrite(dirPin, forwardDirLevel);
    analogWrite(pwmPin, pwm);
  } else if (output == MOTOR_BRAKE) {
    // 속도를 먼저 끊고 제동을 건다.
    // digitalWrite가 analogWrite로 붙어 있던 PWM 타이머를 떼어낸다.
    digitalWrite(pwmPin, LOW);
    digitalWrite(brakePin, HIGH);
  } else {
    // 코스트: 출력도 제동도 없이 자유 회전.
    digitalWrite(pwmPin, LOW);
    digitalWrite(brakePin, LOW);
  }
}

// ===================== 텔레메트리 =====================
// 고정 순서 CSV 한 줄. 파일 상단의 포맷 설명과 반드시 함께 고칠 것.
// 최악의 경우 약 64바이트라 9600 baud에서 마지막 몇 바이트가 송신 버퍼를
// 기다릴 수 있지만, 그래도 1-2ms 수준이라 제어 주기에는 영향이 없다.
void outputTelemetry() {
  Serial.print(F("T,3,"));
  Serial.print(telemetrySeq);
  Serial.print(',');
  Serial.print(filteredPitchDeg, 2);
  Serial.print(',');
  Serial.print((uint8_t)slopeState);
  Serial.print(',');
  Serial.print(fsr1Value);
  Serial.print(',');
  Serial.print(fsr2Value);
  Serial.print(',');
  Serial.print(handleHeld ? 1 : 0);
  Serial.print(',');
  Serial.print(beltFastened ? 1 : 0);
  Serial.print(',');
  Serial.print((uint8_t)controlMode);
  Serial.print(',');
  Serial.print(currentPWM);
  Serial.print(',');
  Serial.print((uint8_t)motorOutput);
  Serial.print(',');
  Serial.print(usDistanceMm[0]);
  Serial.print(',');
  Serial.print(usDistanceMm[1]);
  Serial.print(',');
  Serial.print(usDistanceMm[2]);
  Serial.print(',');
  Serial.print((uint8_t)overallRisk);
  Serial.print(',');
  Serial.println((uint8_t)overallHazard);

  telemetrySeq++;

  // 텔레메트리로 나간 값이 곧 앱이 아는 최신 상태다.
  reportedRisk = overallRisk;
  reportedHazard = overallHazard;
}
