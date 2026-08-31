// 실제로 시리얼에 나가는 줄을 그대로 찍어 본다.
#include <stdio.h>
#include <stdint.h>
#include <math.h>
unsigned long g_millis = 0;
#include "Wire.h"
SerialClass Serial; WireClass Wire; bool g_serialEcho = false;
#include "../merge.ino"

// 노면: 0.5m 지점에서 15cm 아래로 떨어지는 단차(계단)
static double h(double x) { return x > 0.50 ? -0.15 : 0.0; }

int main() {
  setupTerrainDetectors();
  imuReady = true; estimatedSpeedMps = 0.4;
  filteredPitchDeg = -1.24; fsr1Value = 512; fsr2Value = 498;
  handleHeld = true; beltFastened = true; controlMode = MODE_FLAT;
  double T = (US_PING_INTERVAL_MS * US_COUNT) / 1000.0;
  g_millis = 1000;

  printf("--- 평지를 지나다가 0.5m 지점에서 15cm 단차를 만나는 경우 ---\n");
  g_serialEcho = true;
  for (double x = 0.30; x < 0.75; x += estimatedSpeedMps * T) {
    g_millis += (unsigned long)(T * 1000.0 + 0.5);
    for (uint8_t i = 0; i < US_COUNT; i++) {
      double drop = US_MOUNT_HEIGHT_M[i] - h(i == 1 ? x : x - 0.02);
      usDistanceMm[i] = (uint16_t)(drop / usSinTilt[i] * 1000.0 + 0.5);
      runTerrainDetector(i, usDistanceMm[i], g_millis);
    }
    updateOverallRisk(g_millis);
    if (overallRisk != reportedRisk || overallHazard != reportedHazard) {
      printf("  [x=%.2fm] ", x); outputHazardLine();
    }
  }
  printf("--- 같은 순간의 주기 텔레메트리 한 줄 ---\n  ");
  outputTelemetry();
  printf("--- 1.5초 뒤 위험이 풀릴 때 ---\n");
  g_millis += RISK_HOLD_MS + 100;
  updateOverallRisk(g_millis);
  if (overallRisk != reportedRisk || overallHazard != reportedHazard) {
    printf("  "); outputHazardLine();
  }
  printf("  "); outputTelemetry();
  return 0;
}
