# 시리얼 출력 규격 (merge.ino → 라즈베리파이 → 앱)

앱을 만들 때 필요한 값만 정리한 문서다. 코드의 원본 정의는
`merge.ino` 파일 상단 주석에 있고, 둘이 어긋나면 코드가 맞다.

## 링크

| 항목 | 값 |
|---|---|
| 포트 | 아두이노 Mega USB 시리얼 |
| 속도 | **9600** 8N1 (`SERIAL_BAUD`) |
| 방향 | 아두이노 → 파이 **단방향**. 파이가 보내는 값은 없다 |
| 줄 끝 | `\r\n` (아두이노 `println` 기본). 파싱 전에 `strip()` 할 것 |
| 인코딩 | ASCII만 |

줄의 **첫 글자로 종류를 구분**한다. `T` / `H` / `E` 세 가지이며,
그 밖의 줄(부팅 시 잡음, 향후 추가분)은 **무시**하도록 짜야 한다.

## 1. `T` — 주기 상태 (500ms마다)

전체 상태 스냅샷. 앱의 상시 표시용.

```
T,3,17,-1.24,0,512,498,1,1,5,0,0,301,300,299,0,0
```

| # | 이름 | 값 |
|---|---|---|
| 0 | `"T"` | 줄 표식 |
| 1 | version | **3**. 이 값이 다르면 파싱하지 말 것 |
| 2 | seq | 0–255 순환. 줄 유실 감지용 |
| 3 | pitch | 도, 소수 2자리. `+` 오르막 / `-` 내리막 |
| 4 | slope | 0 FLAT / 1 UP / 2 DOWN / 3 UNCERTAIN |
| 5 | fsr1 | 손잡이 압력센서 1 원시값 0–1023 |
| 6 | fsr2 | 손잡이 압력센서 2 원시값 0–1023 |
| 7 | handle | 0 놓음 / 1 잡음 |
| 8 | belt | 0 안전벨트 풀림 / 1 체결 |
| 9 | mode | 0 INITIALIZING / 1 SENSOR_FAULT / 2 HANDLE_RELEASED / 3 DOWNHILL_BRAKE / 4 UPHILL_ASSIST / 5 FLAT / 6 UNCERTAIN / 7 OBSTACLE_BRAKE |
| 10 | pwm | 모터 출력 0–255 |
| 11 | motor | 0 COAST / 1 FORWARD / 2 BRAKE |
| 12 | us_l | 좌 초음파 거리 mm |
| 13 | us_c | 중앙 |
| 14 | us_r | 우 |
| 15 | **risk** | 0 SAFE / 1 CAUTION / 2 DANGER |
| 16 | **hazard** | 0 NONE / 1 STEP(턱) / 2 HOLE(홈) |

- 필드 수는 항상 **17개**다. 다르면 깨진 줄이므로 버린다.
- `us_*`는 20–1000mm, **0은 측정 실패**(에코 없음/범위 밖)다. 거리 0m이
  아니므로 그대로 표시하면 안 된다. 노면을 보는 센서라 평지에서는
  설치 높이/각도로 정해지는 값이 나온다. 기본 설정(높이 16cm, 좌우 65도
  중앙 45도)에서 좌우 약 168mm, 중앙 약 201mm다. 부팅 직후 평지에서 실측해
  잡은 값이 `E,US_CAL`로 나오므로 그 값을 기준으로 보면 된다.
- `risk`/`hazard`는 세 센서 중 **가장 높은 위험도**와 그 원인이다.
- 속도는 내부에서 홈 너비를 적분하는 데만 쓰고 내보내지 않는다.

## 2. `H` — 위험 알림 (바뀌는 순간 즉시)

`risk`나 `hazard`가 바뀌는 순간에만 나간다. 평소에는 한 줄도 나오지
않는다. 앱 푸시/경보의 트리거로 쓰면 된다.

```
H,2,2,1,305,67,150      위험 발생
H,0,0,255,0,0,0         위험 해제
```

| # | 이름 | 값 |
|---|---|---|
| 0 | `"H"` | 줄 표식 |
| 1 | risk | 0 SAFE / 1 CAUTION / 2 DANGER |
| 2 | hazard | 0 NONE / 1 STEP(턱) / 2 HOLE(홈) |
| 3 | sensor | 0 좌 / 1 중 / 2 우, **255 = 없음(위험 해제)** |
| 4 | dist_mm | 그 순간 그 센서의 측정 거리 |
| 5 | width_mm | 홈의 너비. 턱이면 0 |
| 6 | depth_mm | 홈이면 깊이, 턱이면 노면이 올라온 높이 |

- 필드 수는 항상 **7개**.
- `risk=0, sensor=255`가 곧 **경보 해제** 신호다. 앱은 이 줄을 받으면
  화면의 경보를 지운다.
- 위험 판정은 **최소 1.5초 유지**된다(`RISK_HOLD_MS`). 즉 경보가 뜨면
  최소 1.5초는 유지되고, 그 뒤에도 위험이 없으면 해제 줄이 나온다.
- 판정 자체는 센서당 45ms마다 도는데 `T`는 500ms 주기라, 짧은 위험이
  `T`에서 누락될 수 있다. **경보는 `H`로 받고, 화면 상태는 `T`로 유지**하는
  구성이 맞다.

## 3. `E` — 이벤트 / 오류

부팅 순서는 항상 아래와 같다.

```
E,BOOT                  전원 인가
E,CAL,5 / 4 / 3 / 2 / 1 IMU 보정 카운트다운 (1초 간격, 이 5초간 움직이면 안 됨)
E,IMU_READY             또는 E,IMU_FAIL
E,US_BEAM,0,51,75       센서0(좌) 빔 띠 51mm, 전방 주시 75mm
E,US_BEAM,1,86,160      센서1(중) 빔 띠 86mm, 전방 주시 160mm
E,US_BEAM,2,51,75       센서2(우)
E,READY                 이후부터 T / H 줄이 나오기 시작
```

운행 중에 나올 수 있는 줄은 하나다.

```
E,IMU_LOST              IMU 통신 3회 연속 실패. 이후 mode=1(SENSOR_FAULT),
                        모터는 즉시 제동에 들어간다
```

부팅 직후 평지를 얼마간 굴리면 기준거리 보정이 끝나고 한 번 더 나온다.
이 줄이 나오기 전까지는 노면 판정이 돌지 않는다(risk는 계속 0).

```
E,US_CAL,0,168,3,10,30,24     센서0(좌)
E,US_CAL,1,202,2,10,34,56     센서1(중)
E,US_CAL,2,168,3,10,30,24     센서2(우)
```

필드는 `센서, 기준거리, 잡음, 턱시작, 턱확정, 홈시작` (모두 mm)이다.
판정 임계값을 고정값으로 박지 않고 이 보정 구간에서 잰 잡음과 빔 기하로
만들기 때문에, 실제로 어떤 값이 쓰이는지 이 줄로 확인할 수 있다.

`E,BOOT`을 다시 받으면 아두이노가 리셋된 것이므로 앱 상태를 초기화한다.

## 4. `#` — 사람이 읽는 줄 (기본 꺼짐)

`merge.ino`의 `HUMAN_READABLE_LOG`를 1로 바꿔 구우면 나온다. 배포
펌웨어에서는 꺼져 있다. 파서는 `#`로 시작하는 줄을 무시하면 된다.

## 파이 파서

```python
import serial

SLOPE  = ("FLAT", "UP", "DOWN", "UNCERTAIN")
MOTOR  = ("COAST", "FORWARD", "BRAKE")
MODE   = ("INITIALIZING", "SENSOR_FAULT", "HANDLE_RELEASED", "DOWNHILL_BRAKE",
          "UPHILL_ASSIST", "FLAT", "UNCERTAIN", "OBSTACLE_BRAKE")
RISK   = ("SAFE", "CAUTION", "DANGER")
HAZARD = ("NONE", "STEP", "HOLE")
SENSOR = {0: "LEFT", 1: "CENTER", 2: "RIGHT", 255: "NONE"}

ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)

def parse(line):
    p = line.strip().split(',')
    if not p or not p[0]:
        return None

    if p[0] == 'T' and len(p) == 17 and p[1] == '3':
        return {"type": "state",
                "seq": int(p[2]), "pitch": float(p[3]),
                "slope": SLOPE[int(p[4])],
                "fsr1": int(p[5]), "fsr2": int(p[6]),
                "handle": int(p[7]) == 1, "belt": int(p[8]) == 1,
                "mode": MODE[int(p[9])], "pwm": int(p[10]),
                "motor": MOTOR[int(p[11])],
                # 0 = 측정 실패이므로 None으로 바꿔 둔다
                "us": [int(v) or None for v in p[12:15]],
                "risk": RISK[int(p[15])], "hazard": HAZARD[int(p[16])]}

    if p[0] == 'H' and len(p) == 7:
        return {"type": "alert",
                "risk": RISK[int(p[1])], "hazard": HAZARD[int(p[2])],
                "sensor": SENSOR.get(int(p[3]), "NONE"),
                "distance_mm": int(p[4]),
                "width_mm": int(p[5]), "depth_mm": int(p[6])}

    if p[0] == 'E':
        return {"type": "event", "name": p[1], "args": p[2:]}

    return None   # '#' 줄, 깨진 줄, 미래에 늘어날 줄

while True:
    msg = parse(ser.readline().decode(errors="ignore"))
    if msg is None:
        continue
    if msg["type"] == "alert":
        push_to_app(msg)       # 경보 발생/해제
    elif msg["type"] == "state":
        update_app_state(msg)  # 상시 화면 갱신
```

## 앱 쪽 구현 메모

- **경보 표시/해제는 `H`만 보면 된다.** `risk == "SAFE"`면 해제.
- 화면의 위험도 표시는 `T`의 `risk`/`hazard`로 유지한다. `H`를 놓쳐도
  500ms 안에 `T`로 복구된다.
- `T`가 2초 이상 끊기면 아두이노 연결 이상으로 처리한다(정상이면 500ms마다 온다).
- `seq`가 1씩 늘지 않으면 줄이 유실된 것이다. 경보를 놓쳤을 수 있으니
  다음 `T`의 `risk`로 화면을 맞춘다.
- `mode`가 `SENSOR_FAULT` / `HANDLE_RELEASED` / `DOWNHILL_BRAKE` /
  `OBSTACLE_BRAKE`면 모터가 제동 상태다. 노면 위험(`risk`)은 모터에
  개입하지 않으므로 둘은 별개로 표시한다.
- `belt`(안전벨트)는 경고 표시용이고 모터에는 개입하지 않는다.

## 사용자에게 보여줄 문구 예시

| risk | hazard | 문구 |
|---|---|---|
| DANGER | STEP | "앞에 턱이 있습니다. 속도를 줄이세요" |
| DANGER | HOLE | "앞에 홈/단차가 있습니다. 우회하세요" |
| CAUTION | HOLE | "바퀴가 빠질 수 있는 홈이 있습니다" |
| SAFE | NONE | 경보 없음 |

`sensor`(LEFT/CENTER/RIGHT)로 어느 쪽인지 함께 표시하면 좋다.

## 검출 한계 (앱 문구를 정할 때 참고)

시뮬레이션 기준으로 **턱 10cm와 단차 15cm는 모든 속도에서 검출**되지만,
**홈은 사실상 잡히지 않는다.** 5cm 안팎의 홈은 시뮬레이션에서 120회 중
30회 정도만 잡히고 그나마 대부분 바퀴가 지난 뒤다. 초음파 빔이 노면에
45~86mm 띠를 그려서 그보다 좁은 홈은 보이지 않기 때문이고, 이 센서의
물리적 한계다. 그래서 `hazard=HOLE`은 나오면 참고하되, 홈이 없다는
보장으로 쓰면 안 된다.

턱과 단차는 중앙 센서(45도)가 바퀴 도착보다 평균 100mm 앞서 잡는다
(0.4 m/s에서 약 0.27초). 그 이상의 여유가 필요하면 센서를 바퀴보다
앞에 다는 수밖에 없다. 그래서 "홈이 없다"가 아니라 "감지된 위험이
없다"는 뜻으로 표시해야 한다. 밀기 속도가 `E,US_VMAX`로 나온 값
(기본 설정 46cm/s)을 넘으면 홈 너비 판정이 부정확해진다.
