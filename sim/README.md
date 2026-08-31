# 노면 판정 / 속도 추정 시뮬레이터

`merge.ino`의 판정 코드를 **수정 없이 그대로** PC에서 컴파일해서 돌린다.
`Wire.h`는 아두이노 API를 대신하는 최소 스텁이고, `millis()`만 테스트가
직접 굴릴 수 있게 되어 있다. 상수를 바꿔 가며 재튜닝할 때 쓴다.

```sh
cd sim
g++ -O1 -std=gnu++11 -I. -o terrain_sim terrain_sim.cpp && ./terrain_sim -v
g++ -O1 -std=gnu++11 -I. -o speed_sim   speed_sim.cpp   && ./speed_sim -v
```

## terrain_sim

노면 프로파일 10종(평지, 잔요철, 오르막, 내리막, 2cm 홈, 3cm 홈, 8cm 홈,
10cm 턱, 15cm 단차, 홈+턱)을 만들고, 센서 장착 높이/각도로 빗변 거리를
역산해서 잡음(표준편차 5mm) · 측정 실패 5% · 헛에코 2%를 섞어 넣는다.
속도 0.2~0.6 m/s에서 조건당 25회씩 총 1250회를 돌려 두 가지를 센다.

- `UNDER` 과소경보: 위험한 노면을 낮게 봤다 (실제로 위험한 오류, 가중치 10)
- `OVER`  과잉경보: 안전한 노면을 높게 봤다 (성가신 오류, 가중치 1)

현재 `merge.ino` 상수 기준 결과는 `UNDER 1 OVER 7 / 1250`이다.

`SCENARIOS[]`의 `minRisk` / `maxRisk`가 판정 기준이므로, 실제 유모차에서
"이 정도 홈은 지나가도 된다"가 달라지면 이 표부터 고치고 다시 돌린다.

## speed_sim

정지 → 가속 → 순항(속도 변동 포함) → 감속 → 정지 프로파일에 진동과
가속도 바이어스를 섞어 `updateSpeedEstimate()`를 돌린다. 순항 구간 RMS
오차와 정지 후 잔류 속도를 출력한다. 바퀴 엔코더 없이 IMU만 쓰면 순항
RMS 오차가 0.15 m/s 아래로 내려가지 않고, 그 대부분이 실제 속도와
`SPEED_NOMINAL_MPS`의 차이다.

## 상수를 바꿔 가며 탐색하기

`merge.ino`를 직접 고치지 않고 sed로 갈아 끼우면 그리드 탐색이 된다.

```sh
for ping in 10 15 20 25; do
  sed "s/^const unsigned long US_PING_INTERVAL_MS = .*/const unsigned long US_PING_INTERVAL_MS = ${ping};/" \
      ../merge.ino > /tmp/merge_variant.ino
  # terrain_sim.cpp의 include 경로만 바꿔 컴파일해서 비교
done
```

## serial_demo

평지를 지나다 15cm 단차를 만나는 상황을 만들어서, 실제로 시리얼에 나가는
줄을 그대로 찍어 준다. 앱/라즈베리 파서를 만들 때 기대 출력 확인용이다.

```sh
g++ -O1 -std=gnu++11 -I. -o serial_demo serial_demo.cpp && ./serial_demo
```

```
[x=0.53m] H,1,2,1,600,27,150     <- CAUTION / WIDE_HOLE, 중앙 센서
[x=0.55m] H,2,4,1,600,45,150     <- DANGER / STAIR, 너비 45mm 깊이 150mm
          T,2,0,-1.24,0,512,498,1,1,5,0,0,600,600,600,40,2,4
          H,0,0,255,0,0,0        <- 1.5초 뒤 위험 해제
```
