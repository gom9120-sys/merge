#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#define PI 3.1415926535897932384626433832795
#define LOW 0
#define HIGH 1
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define LED_BUILTIN 13
#define A0 54
#define A1 55
#define DEC 10
#define F(x) (x)
typedef bool boolean;
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 0; }
inline void analogWrite(uint8_t, int) {}
inline int analogRead(uint8_t) { return 0; }
extern unsigned long g_millis; inline unsigned long millis() { return g_millis; }
inline unsigned long micros() { return 0; }
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}
// g_serialEcho 를 true 로 두면 Serial 출력이 stdout 으로 그대로 나온다.
extern bool g_serialEcho;
struct SerialClass {
  void begin(unsigned long) {}
  void print(const char* v) { if (g_serialEcho) fputs(v, stdout); }
  void print(char v) { if (g_serialEcho) fputc(v, stdout); }
  void print(int v, int = DEC) { if (g_serialEcho) printf("%d", v); }
  void print(unsigned int v, int = DEC) { if (g_serialEcho) printf("%u", v); }
  void print(long v, int = DEC) { if (g_serialEcho) printf("%ld", v); }
  void print(unsigned long v, int = DEC) { if (g_serialEcho) printf("%lu", v); }
  void print(unsigned char v, int = DEC) { if (g_serialEcho) printf("%u", v); }
  void print(double v, int d = 2) { if (g_serialEcho) printf("%.*f", d, v); }
  void println() { if (g_serialEcho) fputc('\n', stdout); }
  void println(const char* v) { print(v); println(); }
  void println(int v, int b = DEC) { print(v, b); println(); }
  void println(unsigned int v, int b = DEC) { print(v, b); println(); }
  void println(unsigned long v, int b = DEC) { print(v, b); println(); }
  void println(unsigned char v, int b = DEC) { print(v, b); println(); }
  void println(double v, int d = 2) { print(v, d); println(); }
};
extern SerialClass Serial;
struct WireClass {
  void begin() {}
  void setClock(unsigned long) {}
  void beginTransmission(uint8_t) {}
  void write(uint8_t) {}
  uint8_t endTransmission(bool = true) { return 0; }
  uint8_t requestFrom(uint8_t, uint8_t) { return 0; }
  int available() { return 0; }
  int read() { return 0; }
};
extern WireClass Wire;
