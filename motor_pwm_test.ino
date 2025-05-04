// 왼쪽 모터 드라이버 핀 설정 (L298N)
const int L_ENA = 9;  // 왼쪽 모터 Enable/PWM 핀
const int L_IN1 = 10; // 왼쪽 모터 방향 제어 1
const int L_IN2 = 11; // 왼쪽 모터 방향 제어 2

// 오른쪽 모터 드라이버 핀 설정 (L298N)
const int R_ENB = 8;  // 오른쪽 모터 Enable/PWM 핀
const int R_IN3 = 6;  // 오른쪽 모터 방향 제어 1
const int R_IN4 = 7;  // 오른쪽 모터 방향 제어 2

void setup() {
  // 왼쪽 모터 핀 설정
  pinMode(L_ENA, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);

  // 오른쪽 모터 핀 설정
  pinMode(R_ENB, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);

  // 모터의 방향 설정 (전진)
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);

  digitalWrite(R_IN3, HIGH);
  digitalWrite(R_IN4, LOW);
}

void loop() {
  // 왼쪽 모터와 오른쪽 모터의 PWM 값을 0부터 255까지 50씩 증가시키면서 제어
  for (int pwmValue = 0; pwmValue <= 255; pwmValue += 50) {
    analogWrite(L_ENA, pwmValue); // 왼쪽 모터 PWM 제어
    analogWrite(R_ENB, pwmValue); // 오른쪽 모터 PWM 제어
    delay(1000); // 1초 대기
  }

  // PWM 값이 255에 도달하면 다시 0부터 시작
  for (int pwmValue = 255; pwmValue >= 0; pwmValue -= 50) {
    analogWrite(L_ENA, pwmValue); // 왼쪽 모터 PWM 제어
    analogWrite(R_ENB, pwmValue); // 오른쪽 모터 PWM 제어
    delay(1000); // 1초 대기
  }
}
