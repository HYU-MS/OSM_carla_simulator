#include "Cytron_Shield3AMotor.h"

Shield3AMotor motor(SIGNED_MAGNITUDE);

signed int motor1Speed, motor2Speed;

const int always = A1;
const int detect = A2;
const int rotate = A3;
const int restore = A4;
unsigned long startTime1 = 0;
unsigned long startTime2 = 0;
bool runLoop1 = false;
bool runLoop2 = false;

void setup() {
  Serial.begin(115200);
  pinMode(always, OUTPUT);
  pinMode(detect, OUTPUT);
  pinMode(rotate, OUTPUT);
  pinMode(restore, OUTPUT);
  digitalWrite(always, LOW);
  digitalWrite(detect, LOW);
  digitalWrite(rotate, LOW);
  digitalWrite(restore, LOW);
}

void loop() {
  if (runLoop1 && millis() - startTime1 < 3000) {  // 3초 동안만 실행
    motor.control(0, 100);
    return;
  } else {
    runLoop1 = false;
  }
  if (runLoop2 && millis() - startTime2 < 3000) {  // 3초 동안만 실행
    motor.control(100, 0);
    return;
  } else {
    runLoop2 = false;
  }


  if (Serial.available() > 0) {  // 데이터가 도착하면
    char received = Serial.read();  // 데이터 읽기

    switch (received) {
      case '1':
        // 상황 1에 대한 동작
        digitalWrite(always, HIGH);  // LED 켜기
        digitalWrite(detect, LOW);
        digitalWrite(rotate, LOW);
        digitalWrite(restore, LOW);
        motor.control(0, 0);
        break;
      case '2':
        // 상황 2에 대한 동작
        digitalWrite(always, LOW);
        digitalWrite(detect, HIGH);  // LED 켜기
        digitalWrite(rotate, LOW);
        digitalWrite(restore, LOW);
        break;
      case '3':
        // 상황 3에 대한 동작
        digitalWrite(always, LOW);
        digitalWrite(detect, LOW);
        digitalWrite(rotate, HIGH);
        digitalWrite(restore, LOW);  // LED 켜기
        motor.control(0, -100);
        break;
      case '4':
        digitalWrite(always, LOW);
        digitalWrite(detect, LOW);
        digitalWrite(rotate, LOW);
        digitalWrite(restore, HIGH);  // LED 켜기
        motor.control(0, 100);
        break;
      case '5':
        // 상황 3에 대한 동작
        digitalWrite(always, LOW);
        digitalWrite(detect, LOW);
        digitalWrite(rotate, HIGH);
        digitalWrite(restore, LOW);  // LED 켜기
        motor.control(-100, 0);
        break;
      case '6':
        digitalWrite(always, LOW);
        digitalWrite(detect, LOW);
        digitalWrite(rotate, LOW);
        digitalWrite(restore, HIGH);  // LED 켜기
        motor.control(100, 0);
        break;
      case '7':
        runLoop1 = true;
        startTime1 = millis();
      case '8':
        runLoop2 = true;
        startTime2 = millis();
      default:
        digitalWrite(always, HIGH);  // LED 끄기
        digitalWrite(detect, LOW);
        digitalWrite(rotate, LOW);
        digitalWrite(restore, LOW);
        motor.control(0, 0);
        break;
    }
  }
}