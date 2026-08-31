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
// 상수 튜닝 근거 (시뮬레이션)
// ------------------------------------------------------------
// 이 파일의 판정 코드를 그대로 떼어내 PC에서 돌린 시뮬레이션으로 정했다
// (sim/ 폴더). 노면 11종(평지, 잔요철, 오르막, 내리막, 2cm 홈, 3cm 홈,
// 5cm 홈, 8cm 홈, 10cm 턱, 15cm 단차, 홈+턱)에 측정 잡음, 스침각에 따른
// 측정 실패, 헛에코를 섞고 속도 0.2~0.6 m/s에서 조건당 25회, 총 1375회.
// 판정 결과는 두 가지로 나눠 셌다.
//   과소경보 = 위험한 노면을 낮게 본 것 (실제로 위험한 오류)
//   과잉경보 = 안전한 노면을 높게 본 것 (성가신 오류)
//
// 가장 크게 작용한 것은 초음파의 빔 폭이다. HC-SR04는 빔이 15도라 노면을
// 띠로 비추고, 그 띠보다 좁은 홈은 주변 평지가 먼저 에코를 돌려줘서 아예
// 보이지 않는다. 띠 길이는 설치 각도로 정해지므로 각도가 곧 성능이다.
// 높이 15cm에서 각도별 결과(측정 간격 15ms 기준):
//
//   각도   빔 띠   전방주시   과소   과잉
//    30도  167mm   260mm      444     1     <- 홈을 거의 못 본다
//    45도   80mm   150mm      261   195
//    60도   53mm    87mm      206   260
//    70도   45mm    55mm      125    91
//    80도   41mm    26mm      117    92     <- 채택
//    90도   39mm     0mm      115    90     <- 전방주시가 0
//
// 그래서 기본값을 80도로 잡았다. 90도가 근소하게 낫지만 전방 주시가 0이라
// 위험을 바로 아래에서야 알게 된다. 80도는 26mm 앞을 보면서 띠 길이는
// 90도와 2mm 차이다.
//
// 남은 오차의 성격 (80도, 측정 간격 15ms 기준)
//   턱 10cm / 단차 15cm / 홈 탈출 턱 : 모든 속도에서 100% 검출
//   8cm 홈 : 0.5 m/s까지 검출, 0.6 m/s에서 놓치기 시작
//   5cm 홈 : 대부분 놓친다. 빔 띠가 41mm라 5cm 홈은 한 프레임만 보이고,
//            그 한 프레임은 헛에코와 구분할 방법이 없다.
//   평지 오경보 : 전부 헛에코 한 프레임 때문이다(헛에코를 0으로 두면
//            과잉경보가 0이 된다). 그래서 3점 중앙값 필터를 넣었고
//            과잉경보가 450건에서 92건으로 줄었다.
//
// 즉 이 구성으로 확실히 잡는 것은 턱과 단차이고, 홈은 5cm 근처가 한계다.
// 더 좁은 홈까지 보려면 빔이 좁은 센서로 바꾸거나 센서를 더 낮게 달아야
// 한다(띠 길이는 높이에 비례한다). 다만 센서보다 높은 턱은 노면으로 보이지
// 않으므로, 높이는 감지하려는 가장 높은 턱보다 확실히 높아야 한다.
//
// 그 밖에 크게 작용한 것 세 가지
//   1. 측정 간격 60ms -> 15ms. 센서당 180ms에서는 홈이 샘플 사이로 빠진다.
//   2. HOLE_EXIT_CHECK_FIRST. 원본 순서에서는 홈 너비가 CAUTION 구간에
//      들어가는 순간부터 복귀 판정을 건너뛰어, 홈이 끝나도 반드시 넓은
//      홈(DANGER)으로 올라갔다. 상수로는 고칠 수 없는 문제였다.
//   3. 홈 진입 시 빔 띠 길이를 미리 더하기. 홈은 띠가 통째로 들어가야
//      보이기 시작하므로, 그냥 세면 겉보기 너비가 띠 길이만큼 짧게 나와
//      5~8cm 홈이 안전한 홈으로 과소평가됐다.
//
// 속도 추정도 같은 방식으로 밀기 프로파일(정지-가속-순항-감속-정지)에
// 진동과 가속도 바이어스를 넣고 맞췄다. 자세한 결과는 속도 추정 상수
// 주석에 적어 두었다.
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
//   E,US_BEAM,<빔길이mm>,<전방주시mm>  설치 각도에서 나오는 빔 기하
//   E,US_VMAX,<cm/s>  현재 측정 주기에서 홈 너비 판정이 가능한 한계 속도
//
// 그리고 HUMAN_READABLE_LOG가 1이면 사람이 읽는 줄이 하나 더 나간다.
// '#'로 시작하므로 아래 파서는 그대로 무시한다.
//   # 12.345s DANGER HOLE  C dist=250mm width=45mm depth=150mm
//   # 13.900s SAFE   CLEAR
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
// 값이 이미 US_MOUNT_HEIGHT_M / sin(US_TILT_DEG) (기본 설정에서 300mm)라
// 임계값을 그보다 확실히 낮게(기본 150mm) 잡지 않으면 평지에서 계속 제동이
// 걸린다. 앞을 보는 장애물 감지가 필요하면 수평 센서를 따로 다는 편이 맞다.
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
// 시리얼 모니터에서 눈으로 확인하는 용도다. '#'로 시작해서 파이/앱 파서는
// 그대로 무시한다. 9600 baud에서 한 줄에 약 60ms가 드는데 판정이 바뀔
// 때만 나가므로 제어 주기에는 영향이 없다. 실차 배포 때는 0으로 끈다.
//   # 12.345s DANGER HOLE  C dist=250mm width=45mm depth=150mm
//   # 13.900s SAFE   CLEAR
#define HUMAN_READABLE_LOG 1

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
const uint16_t US_OBSTACLE_BRAKE_MM = 150;       // 자동 제동 임계(옵션)

// ===================== 초음파 장착 형상 =====================
// 센서별로 자유롭게 바꾼다. 좌 / 중 / 우 순서.
//   US_MOUNT_HEIGHT_M : 노면에서 센서까지의 높이 (m)
//   US_TILT_DEG       : 수평면 기준 아래로 숙인 각도 (도). 90 = 정확히 아래
// 각도는 빔이 노면에 그리는 띠의 길이를 좌우한다(아래 US_BEAM_HALF_ANGLE_DEG).
// 부팅 때 E,US_BEAM,<빔길이mm>,<전방주시mm> 로 실제 값을 찍어 준다.
// 세 개를 서로 다르게 달아도 되고, 값만 여기서 고치면 된다.
// 각도가 0에 가까우면(수평) 노면을 보지 않는 셈이라 노면 판정이 무의미하다.
// 그래서 sin은 US_MIN_TILT_SIN 아래로 내려가지 않게 막는다.
float US_MOUNT_HEIGHT_M[US_COUNT] = { 0.15, 0.15, 0.15 };
float US_TILT_DEG[US_COUNT] = { 80.0, 80.0, 80.0 };
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

// ===================== 노면 위험 판정값 (detect.py) =====================
// 원본 detect.py의 DangerDetector 생성 인자와 같은 값이다.
const float SUDDEN_CHANGE_THRESHOLD_M = 0.06;  // 턱/홈으로 볼 급변 임계 (m)
const float RAMP_CHANGE_THRESHOLD_M = 0.03;    // 경사로로 볼 완만한 변화 (m)
const float WHEEL_WIDTH_M = 0.07;              // 앞바퀴 지름 (m)

// 원본과 동일한 유도값. 원본 주석대로 실험적으로 맞춰야 하는 계수라
// 비율을 따로 빼 두었다.
const float HOLE_SAFE_GAP_RATIO = 0.6;   // 안전하게 지날 수 있는 홈 너비 / 바퀴 지름
const float HOLE_TOLERANCE_RATIO = 0.6;  // 원래 높이 복귀 허용 오차 / 바퀴 지름
const float HOLE_SAFE_GAP_M = HOLE_SAFE_GAP_RATIO * WHEEL_WIDTH_M;
const float HOLE_TOLERANCE_M = HOLE_TOLERANCE_RATIO * WHEEL_WIDTH_M;

// 홈 진입 프레임에서 이미 지나왔다고 볼 너비의 비율.
// 원본은 speed * interval, 즉 한 프레임을 통째로 센다(1.0). 실제로 홈의
// 시작 지점은 그 프레임 어딘가라 평균적으로는 절반이 맞고, 이 한 프레임의
// 과대평가가 좁은 홈을 계단으로 올려버리는 주된 원인이었다.
const float HOLE_ENTRY_WIDTH_RATIO = 0.5;

// 측정 주기와 판정 가능한 속도 (빔 폭과 함께 성능을 정하는 두 축 중 하나)
//   hole_width는 프레임마다 speed * interval씩 쌓이고, 여기서 interval은
//   그 센서가 다시 측정될 때까지의 시간, 즉 US_PING_INTERVAL_MS * US_COUNT다.
//   그래서 한 프레임에 쌓이는 너비(= 공간 분해능) q = speed * interval이고,
//   q가 0.5 * safe_gap보다 커지면 지나갈 수 있는 홈과 위험한 홈을 구분할 수 없다.
//       v_max = 0.5 * safe_gap / (US_PING_INTERVAL_MS * US_COUNT)
//   잡음/드롭아웃을 넣은 시뮬레이션에서 측정 간격별로 판정이 깨지는 속도가
//   이 식과 거의 그대로 나왔다(바퀴 0.07m, safe_gap 0.042m 기준).
//       15ms (센서당  45ms) : 0.6 m/s 까지 정상, 0.8 m/s에서 깨짐
//       20ms (센서당  60ms) : 0.4 m/s 까지
//       25ms (센서당  75ms) : 0.3 m/s 까지
//       30ms (센서당  90ms) : 0.3 m/s 까지
//       60ms (센서당 180ms) : 0.2 m/s 에서도 깨짐 (기존 값)
//   그래서 15ms로 내렸다. 10ms면 0.8 m/s까지 올라가지만 HC-SR04의 잔향과
//   센서 간 크로스토크 여유가 줄어든다. 실측에서 거리값이 튀면 20~25ms로
//   올리고, 대신 미는 속도를 위 표만큼 낮춰야 한다.
//   부팅 때 E,US_VMAX,<cm/s> 로 현재 설정의 한계 속도를 찍어 준다.
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

// detect.py의 DangerDetector 인스턴스 하나에 해당한다(센서당 하나).
struct DangerDetector {
  bool hasPrev;            // prev_distances is None 여부
  float prevDropM;         // 직전 프레임의 수직 낙차 (m)
  bool inHole;             // in_hole
  float holeWidthM;        // hole_width
  float holeDepthM;        // hole_depth
  unsigned long lastSampleAtMs;
  uint16_t recentMm[3];    // 중앙값 필터용 최근 유효 측정
  uint8_t recentCount;
  float pendingIntervalS;  // 측정 실패로 건너뛴 시간 (다음 유효 프레임에 합산)
  RiskLevel risk;          // 마지막 판정 (RISK_HOLD_MS 동안 유지)
  HazardCause hazard;
  unsigned long riskAtMs;
  // 판정이 난 그 프레임의 홈 너비/깊이. STAIR나 EXIT_STEP은 판정과 동시에
  // 홈 상태를 지우기 때문에, 앱으로 보낼 값은 지우기 전에 따로 잡아 둔다.
  // hazard가 STEP / EXIT_STEP이면 riskDepthM은 홈 깊이가 아니라 턱 높이다.
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
float groundDropM(uint8_t index, uint16_t distanceMm);
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

  // 설치 각도에서 나오는 빔 띠 길이와 전방 주시 거리 (mm)
  Serial.print(F("E,US_BEAM,"));
  Serial.print((int)(usBeamFootprintM[0] * 1000.0));
  Serial.print(',');
  Serial.println((int)(usLookAheadM[0] * 1000.0));

  // 현재 측정 주기에서 홈/계단을 구분할 수 있는 한계 속도를 알려 준다.
  // v_max = 0.5 * safe_gap / (센서당 측정 간격)
  Serial.print(F("E,US_VMAX,"));
  Serial.println((int)(0.5 * HOLE_SAFE_GAP_M
                       / (US_PING_INTERVAL_MS * US_COUNT / 1000.0) * 100.0));

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

    resetDetector(i);
  }
}

// DangerDetector.__init__ 에 해당. 홈 상태와 이전 프레임을 모두 버린다.
void resetDetector(uint8_t index) {
  DangerDetector &d = detectors[index];
  d.hasPrev = false;
  d.prevDropM = 0.0;
  d.inHole = false;
  d.holeWidthM = 0.0;
  d.holeDepthM = 0.0;
  d.pendingIntervalS = 0.0;
  d.recentCount = 0;
  d.recentMm[0] = d.recentMm[1] = d.recentMm[2] = 0;
  d.lastSampleAtMs = millis();
  d.risk = RISK_SAFE;
  d.hazard = HAZARD_NONE;
  d.riskAtMs = d.lastSampleAtMs;
  d.riskWidthM = 0.0;
  d.riskDepthM = 0.0;
}

// 빗변 거리(mm) -> 센서 아래 노면까지의 수직 낙차(m).
// 노면으로 볼 수 없는 값이면 음수를 돌려준다(원본의 None에 해당).
float groundDropM(uint8_t index, uint16_t distanceMm) {
  if (distanceMm == 0) return -1.0;

  float distanceM = distanceMm / 1000.0;
  if (distanceM < usExpectedFlatM[index] * US_GROUND_MIN_RATIO
      || distanceM > usExpectedFlatM[index] * US_GROUND_MAX_RATIO) {
    return -1.0;
  }
  return distanceM * usSinTilt[index];
}

// 세 값의 중앙값. 한 프레임짜리 헛에코를 걸러낸다.
static uint16_t medianOf3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

// 원본 predict()의 센서 한 개분. 흐름도의 분기 순서를 그대로 따른다.
void runTerrainDetector(uint8_t index, uint16_t distanceMm, unsigned long now) {
  DangerDetector &d = detectors[index];

  float interval = (now - d.lastSampleAtMs) / 1000.0 + d.pendingIntervalS;
  d.lastSampleAtMs = now;
  d.pendingIntervalS = 0.0;

  // 측정이 오래 끊겼으면 홈을 이어서 적분할 근거가 없다. 상태를 버린다.
  if (interval > US_STALE_INTERVAL_S) {
    d.hasPrev = false;
    d.inHole = false;
    d.holeWidthM = 0.0;
    d.holeDepthM = 0.0;
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

  float drop = groundDropM(index, distanceMm);
  if (drop < 0.0) {
    // 측정 실패(원본의 None). 이번 프레임은 판정하지 않는다.
#if US_KEEP_PREV_ON_DROPOUT
    d.pendingIntervalS = interval;  // 놓친 시간은 다음 유효 프레임에 넘긴다
#else
    d.hasPrev = false;              // 원본: 다음 프레임도 비교 대상이 없다
#endif
    d.recentCount = 0;              // 끊긴 뒤의 중앙값은 믿을 수 없다
    return;
  }

  if (!d.hasPrev) {  // prev_distances is None (첫 측정)
    d.prevDropM = drop;
    d.hasPrev = true;
    return;
  }

  float delta = drop - d.prevDropM;
  d.prevDropM = drop;

  float speed = currentSpeedMps();
  RiskLevel risk = RISK_SAFE;
  HazardCause hazard = HAZARD_NONE;
  float eventWidthM = 0.0;   // 이번 프레임 판정에 쓰인 홈 너비
  float eventDepthM = 0.0;   // 홈 깊이, 턱이면 턱 높이

  if (!d.inHole) {
    // ---------- 홈이 감지되지 않은 상태 ----------
    if (fabs(delta) < RAMP_CHANGE_THRESHOLD_M) {
      // 1. 경사로
      risk = RISK_SAFE;
      hazard = HAZARD_NONE;
    } else if (delta < -SUDDEN_CHANGE_THRESHOLD_M) {
      // 2. 턱 (노면이 갑자기 가까워짐)
      risk = RISK_DANGER;
      hazard = HAZARD_STEP;
      eventDepthM = -delta;  // 턱 높이
    } else if (delta > SUDDEN_CHANGE_THRESHOLD_M) {
      // 3. 홈 (노면이 갑자기 멀어짐) - 여기서부터 너비를 적분한다.
      risk = RISK_SAFE;
      hazard = HAZARD_NONE;
      d.inHole = true;
      d.holeDepthM = delta;
      // 홈은 빔의 띠가 통째로 안에 들어가야 보이기 시작한다. 그 시점에는
      // 이미 띠 길이만큼 홈을 지나온 뒤이므로 그만큼을 더하고 시작한다.
      d.holeWidthM = usBeamFootprintM[index]
                     + speed * interval * HOLE_ENTRY_WIDTH_RATIO;
      eventWidthM = d.holeWidthM;
      eventDepthM = d.holeDepthM;
    } else {
      // 임계값 사이
      risk = RISK_SAFE;
      hazard = HAZARD_NONE;
    }
  } else {
    // ---------- 홈이 감지된 상태 ----------
    risk = RISK_SAFE;
    hazard = HAZARD_NONE;
    d.holeWidthM += speed * interval;
    // 상태를 지우는 분기가 있으므로 판정에 쓰인 값을 먼저 남겨 둔다.
    eventWidthM = d.holeWidthM;
    eventDepthM = d.holeDepthM;

    // 노면이 원래 높이 근처(또는 그 위)로 돌아왔는가.
    bool recovered = false;
#if HOLE_EXIT_CHECK_FIRST
    recovered = (-delta > d.holeDepthM - HOLE_TOLERANCE_M);
#endif

    if (recovered) {
      // 홈이 여기서 끝났다. 지나온 너비로 위험도를 매기고 상태를 닫는다.
      if (-delta > d.holeDepthM + HOLE_TOLERANCE_M) {
        // 원래 높이보다 더 높아짐 = 턱
        risk = RISK_DANGER;
        hazard = HAZARD_STEP;
        eventDepthM = -delta - d.holeDepthM;  // 원래 높이 위로 올라온 만큼
      } else if (d.holeWidthM >= HOLE_SAFE_GAP_M) {
        risk = RISK_DANGER;
        hazard = HAZARD_HOLE;
      } else if (d.holeWidthM >= 0.5 * HOLE_SAFE_GAP_M) {
        risk = RISK_CAUTION;
        hazard = HAZARD_HOLE;
      }
      d.inHole = false;
      d.holeDepthM = 0.0;
      d.holeWidthM = 0.0;
    } else if (d.holeWidthM >= HOLE_SAFE_GAP_M) {
      // 홈이 아니라 계단처럼 아예 단차. 경보를 띄우고 홈 상태에서 벗어난다.
      risk = RISK_DANGER;
      hazard = HAZARD_HOLE;
      d.inHole = false;
      d.holeDepthM = 0.0;
      d.holeWidthM = 0.0;
    } else if (d.holeWidthM >= 0.5 * HOLE_SAFE_GAP_M) {
      // 아직 지나갈 수는 있지만 바퀴가 빠질 수 있는 너비.
      risk = RISK_CAUTION;
      hazard = HAZARD_HOLE;
    } else {
      if (-delta > d.holeDepthM + HOLE_TOLERANCE_M) {
        // 홈에서 원래 높이보다 더 높아짐 = 턱
        risk = RISK_DANGER;
        hazard = HAZARD_STEP;
        eventDepthM = -delta - d.holeDepthM;
        d.inHole = false;
        d.holeDepthM = 0.0;
        d.holeWidthM = 0.0;
      } else if (-delta > d.holeDepthM - HOLE_TOLERANCE_M) {
        // 원래 높이 근처로 복귀
        d.inHole = false;
        d.holeDepthM = 0.0;
        d.holeWidthM = 0.0;
      } else {
        // 홈의 깊이가 약간 달라짐. 음수는 노이즈로 보고 잘라낸다.
        d.holeDepthM = d.holeDepthM + delta;
        if (d.holeDepthM < 0.0) d.holeDepthM = 0.0;
      }
    }
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
