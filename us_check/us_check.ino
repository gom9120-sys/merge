// ============================================================
// us_check.ino - HC-SR04 배선 확인용 최소 스케치
//
// merge.ino와 무관하게 초음파 3개만 돌린다. 배선/전원 문제인지
// 펌웨어 문제인지 가르는 용도다. 핀 번호는 merge.ino와 같다.
//
// 시리얼 모니터 9600으로 열면 센서마다 한 줄씩 나온다.
//   L trig=22 echo=23  pulse=1180us  dist=202mm
//   C trig=24 echo=25  NO ECHO   <- 트리거는 나갔는데 에코가 안 올라옴
//   R trig=26 echo=27  pulse=8000us  dist=1372mm
//
// NO ECHO 가 나오면 순서대로 확인한다.
//   1. 모듈 VCC가 5V인지 (3.3V로는 동작하지 않는다)
//   2. GND가 아두이노 GND와 같이 묶여 있는지 (별도 전원이면 필수)
//   3. TRIG/ECHO를 서로 바꿔 꽂지 않았는지
//   4. 핀 번호가 아래 배열과 맞는지
// ============================================================

const uint8_t COUNT = 3;
const uint8_t TRIG[COUNT] = { 22, 24, 26 };   // 좌, 중, 우
const uint8_t ECHO[COUNT] = { 23, 25, 27 };
const char NAMES[COUNT] = { 'L', 'C', 'R' };

void setup() {
  Serial.begin(9600);
  for (uint8_t i = 0; i < COUNT; i++) {
    pinMode(TRIG[i], OUTPUT);
    digitalWrite(TRIG[i], LOW);
    pinMode(ECHO[i], INPUT);
  }
  delay(500);
  Serial.println(F("us_check start"));
}

void loop() {
  for (uint8_t i = 0; i < COUNT; i++) {
    digitalWrite(TRIG[i], LOW);
    delayMicroseconds(4);
    digitalWrite(TRIG[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG[i], LOW);

    // 넉넉한 한계로 잰다 (30ms = 약 5m). merge.ino보다 훨씬 관대하게 잡아서
    // '값이 이상한 것'과 '아예 안 오는 것'을 구분한다.
    unsigned long pulse = pulseIn(ECHO[i], HIGH, 30000UL);

    Serial.print(NAMES[i]);
    Serial.print(F(" trig="));
    Serial.print(TRIG[i]);
    Serial.print(F(" echo="));
    Serial.print(ECHO[i]);
    if (pulse == 0) {
      Serial.println(F("  NO ECHO"));
    } else {
      Serial.print(F("  pulse="));
      Serial.print(pulse);
      Serial.print(F("us  dist="));
      Serial.print((pulse * 343UL) / 2000UL);
      Serial.println(F("mm"));
    }
    delay(60);
  }
  Serial.println();
  delay(500);
}
