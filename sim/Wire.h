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
struct SerialClass {
  void begin(unsigned long) {}
  void print(const char*) {}
  void print(int, int = DEC) {}
  void print(unsigned int, int = DEC) {}
  void print(long, int = DEC) {}
  void print(unsigned long, int = DEC) {}
  void print(unsigned char, int = DEC) {}
  void print(char) {}
  void print(double, int = 2) {}
  void println(const char*) {}
  void println(int, int = DEC) {}
  void println(unsigned int, int = DEC) {}
  void println(unsigned long, int = DEC) {}
  void println(unsigned char, int = DEC) {}
  void println(double, int = 2) {}
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
