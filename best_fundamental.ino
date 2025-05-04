/*
 * 2륜 구동 로봇 PID 위치 제어
 * - Arduino Mega 2560 + L298N 모터 드라이버
 * - 17PPR 증분형 엔코더 (2채널)
 * - 감속비 1/21 
 * - 시리얼 명령어: 'forward', 'backward'
 */

// 모터 드라이버 핀 설정 (L298N)
// 왼쪽 모터
const int L_ENA = 9;    // 왼쪽 모터 Enable/PWM 핀
const int L_IN1 = 10;   // 왼쪽 모터 방향 제어 1
const int L_IN2 = 11;   // 왼쪽 모터 방향 제어 2

// 오른쪽 모터
const int R_ENB = 8;    // 오른쪽 모터 Enable/PWM 핀
const int R_IN3 = 6;    // 오른쪽 모터 방향 제어 1
const int R_IN4 = 7;    // 오른쪽 모터 방향 제어 2

// 엔코더 핀 설정 (인터럽트 핀)
const int LEFT_ENCODER_A = 2;   // 왼쪽 모터 엔코더 A상 (인터럽트 0)
const int LEFT_ENCODER_B = 3;   // 왼쪽 모터 엔코더 B상 (인터럽트 1)
const int RIGHT_ENCODER_A = 18; // 오른쪽 모터 엔코더 A상 (인터럽트 5)
const int RIGHT_ENCODER_B = 19; // 오른쪽 모터 엔코더 B상 (인터럽트 4)

// 엔코더 카운트 변수
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// 목표 위치
volatile long leftTargetPosition = 0;
volatile long rightTargetPosition = 0;

// PID 제어 관련 변수
unsigned long prevTime = 0;
const unsigned long CONTROL_INTERVAL = 20; // 20ms 제어 주기

// PID 상수
float Kp = 0.09;    // 비례 게인
float Ki = 0;    // 적분 게인
float Kd = 0;   // 미분 게인

// PID 오차 변수
long leftPrevError = 0;
long rightPrevError = 0;
long leftIntegral = 0;
long rightIntegral = 0;

// 모터 제어 변수
int leftPWM = 0;
int rightPWM = 0;

// 로봇 이동 거리 설정 (엔코더 카운트 기준)
const long MOVE_DISTANCE = 17 * 21 * 10; // 17PPR * 21(감속비) * 10(바퀴 회전 수)
boolean isMoving = false;
boolean isForward = true;

// 시리얼 통신 변수
String inputString = "";
boolean stringComplete = false;

void setup() {
  // 시리얼 통신 초기화
  Serial.begin(115200);
  inputString.reserve(200);
  
  // 모터 드라이버 핀 설정
  pinMode(L_ENA, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_ENB, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);
  
  // 모터 정지
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, LOW);
  analogWrite(L_ENA, 0);
  analogWrite(R_ENB, 0);
  
  // 엔코더 핀 설정
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  
  // 인터럽트 설정 (4개 핀 모두 rising과 falling에 대해 인터럽트 설정)
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), leftEncoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), rightEncoderB_ISR, CHANGE);
  
  // 초기 설정 및 안내 메시지
  Serial.println("2륜 구동 로봇 PID 위치 제어 시스템");
  Serial.println("명령어: 'forward' - 앞으로 이동, 'backward' - 뒤로 이동");
  Serial.println();
  /*
  // PWM 테스트
  Serial.println("모터 테스트 시작 (0.5초간 PWM 70으로 구동)");
  
  // 왼쪽 모터 전진 방향 테스트
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_ENA, 70);
  
  // 오른쪽 모터 전진 방향 테스트
  digitalWrite(R_IN3, HIGH);
  digitalWrite(R_IN4, LOW);
  analogWrite(R_ENB, 70);
  
  delay(500); // 0.5초 동안 구동
  
  // 모터 정지
  analogWrite(L_ENA, 0);
  analogWrite(R_ENB, 0);
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, LOW);
  
  Serial.println("모터 테스트 완료");
  Serial.println();
  delay(3000);
  */
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  Serial.println(leftEncoderCount);
  Serial.println(rightEncoderCount);

  
  // 타이머 초기화
  prevTime = millis();
}

void loop() {
  // 시리얼 명령어 처리
  if (stringComplete) {
    if (inputString.indexOf("forward") >= 0) {
      startMoving(true);
    }
    else if (inputString.indexOf("backward") >= 0) {
      startMoving(false);
    }
    
    // 문자열 초기화
    inputString = "";
    stringComplete = false;
  }
  
  // PID 제어 (20ms 주기로 실행)
  unsigned long currentTime = millis();
  if (currentTime - prevTime >= CONTROL_INTERVAL) {
    if (isMoving) {
      // PID 제어 실행
      updatePID();
      
      // 목표 위치에 도달했는지 확인
      if (abs(leftTargetPosition - leftEncoderCount) < 10 && 
          abs(rightTargetPosition - rightEncoderCount) < 10) {
        stopMotors();
        isMoving = false;
        Serial.println("목표 위치에 도달했습니다.");
      }
    }
    
    // Serial Plotter 데이터 출력 (엔코더 카운트 및 목표값)
    Serial.print("leftEncoderCount:");
    Serial.print(leftEncoderCount);
    Serial.print(",leftTargetPosition:");
    Serial.print(leftTargetPosition);
    Serial.print(",rightEncoderCount:");
    Serial.print(rightEncoderCount);
    Serial.print(",rightTargetPosition:");
    Serial.println(rightTargetPosition);
    
    // Serial Monitor에 PWM 값 출력
    if (isMoving) {
      Serial.print("왼쪽 PWM: ");
      Serial.print(leftPWM);
      Serial.print(", 오른쪽 PWM: ");
      Serial.println(rightPWM);
    }
    
    prevTime = currentTime;
  }
}

// 시리얼 이벤트 핸들러
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

// 로봇 이동 시작 함수
void startMoving(boolean forward) {
  // 이동 방향 설정
  isForward = forward;
  
  // 현재 위치를 기준으로 목표 위치 설정
  if (forward) {
    leftTargetPosition = leftEncoderCount + MOVE_DISTANCE;
    rightTargetPosition = rightEncoderCount + MOVE_DISTANCE;
    Serial.println("앞으로 이동합니다.");
  } else {
    leftTargetPosition = leftEncoderCount - MOVE_DISTANCE;
    rightTargetPosition = rightEncoderCount - MOVE_DISTANCE;
    Serial.println("뒤로 이동합니다.");
  }
  
  // PID 제어 변수 초기화
  leftPrevError = 0;
  rightPrevError = 0;
  leftIntegral = 0;
  rightIntegral = 0;
  
  // 이동 상태 설정
  isMoving = true;
  
  // 모터 방향 설정
  if (forward) {
    // 전진 방향
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN3, HIGH);
    digitalWrite(R_IN4, LOW);
  } else {
    // 후진 방향
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, HIGH);
  }
}

// PID 제어 업데이트 함수
void updatePID() {
  // 왼쪽 모터 PID 제어
  // 현재 오차 계산
  long leftError = leftTargetPosition - leftEncoderCount;
  
  // 적분 항 계산 (구분구적법과 유사하게 사각형을 계속합치는 거니까 사실상 적분임)
  leftIntegral += leftError * CONTROL_INTERVAL;
  leftIntegral = constrain(leftIntegral, -100, 100);
  
  // 미분 항 계산 (매우 짧은 시간에 해당하는 error의 기울기니까 사실상 미분값임)
  long leftDerivative = (leftError - leftPrevError) / CONTROL_INTERVAL;
  
  // PID 출력 계산
  float leftOutput = Kp * leftError + Ki * leftIntegral + Kd * leftDerivative;
  
  // PWM 출력값 계산 (절대값)
  leftPWM = constrain((int)leftOutput, 0, 70);
  
  // 방향에 따른 PWM 적용
  analogWrite(L_ENA, leftPWM);
  
  // 오른쪽 모터 PID 제어
  // 현재 오차 계산
  long rightError = rightTargetPosition - rightEncoderCount;
  
  // 적분 항 계산
  rightIntegral += rightError * CONTROL_INTERVAL;
  rightIntegral = constrain(rightIntegral, -100, 100);
  
  // 미분 항 계산
  long rightDerivative = (rightError - rightPrevError) / CONTROL_INTERVAL;
  
  // PID 출력 계산
  float rightOutput = Kp * rightError + Ki * rightIntegral + Kd * rightDerivative;
  
  // PWM 출력값 계산 (절대값)
  rightPWM = constrain((int)rightOutput, 0, 70);
  
  // 방향에 따른 PWM 적용
  analogWrite(R_ENB, rightPWM);
  
  // 이전 오차 저장
  leftPrevError = leftError;
  rightPrevError = rightError;
}

// 모터 정지 함수
void stopMotors() {
  analogWrite(L_ENA, 0);
  analogWrite(R_ENB, 0);
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, LOW);
}

// 왼쪽 엔코더 A상 인터럽트 서비스 루틴
void leftEncoderA_ISR() {
  // A상과 B상의 현재 상태 읽기
  bool A_val = digitalRead(LEFT_ENCODER_A);
  bool B_val = digitalRead(LEFT_ENCODER_B);
  
  // 엔코더 카운트 업데이트 (A와 B의 위상 차이를 이용한 방향 감지)
  // A상이 rising edge일 때
  if (A_val == HIGH) {
    // B가 LOW면 시계 방향 (증가)
    if (B_val == LOW) {
      leftEncoderCount++;
    } 
    // B가 HIGH면 반시계 방향 (감소)
    else {
      leftEncoderCount--;
    }
  }
  // A상이 falling edge일 때
  else {
    // B가 HIGH면 시계 방향 (증가)
    if (B_val == HIGH) {
      leftEncoderCount++;
    } 
    // B가 LOW면 반시계 방향 (감소)
    else {
      leftEncoderCount--;
    }
  }
}

// 왼쪽 엔코더 B상 인터럽트 서비스 루틴
void leftEncoderB_ISR() {
  // A상과 B상의 현재 상태 읽기
  bool A_val = digitalRead(LEFT_ENCODER_A);
  bool B_val = digitalRead(LEFT_ENCODER_B);
  
  // 엔코더 카운트 업데이트 (A와 B의 위상 차이를 이용한 방향 감지)
  // B상이 rising edge일 때
  if (B_val == HIGH) {
    // A가 HIGH면 시계 방향 (증가)
    if (A_val == HIGH) {
      leftEncoderCount++;
    } 
    // A가 LOW면 반시계 방향 (감소)
    else {
      leftEncoderCount--;
    }
  }
  // B상이 falling edge일 때
  else {
    // A가 LOW면 시계 방향 (증가)
    if (A_val == LOW) {
      leftEncoderCount++;
    } 
    // A가 HIGH면 반시계 방향 (감소)
    else {
      leftEncoderCount--;
    }
  }
}

// 오른쪽 엔코더 A상 인터럽트 서비스 루틴
void rightEncoderA_ISR() {
  // A상과 B상의 현재 상태 읽기
  bool A_val = digitalRead(RIGHT_ENCODER_A);
  bool B_val = digitalRead(RIGHT_ENCODER_B);
  
  // 엔코더 카운트 업데이트 (A와 B의 위상 차이를 이용한 방향 감지)
  // A상이 rising edge일 때
  if (A_val == HIGH) {
    // B가 LOW면 시계 방향 (증가)
    if (B_val == LOW) {
      rightEncoderCount++;
    } 
    // B가 HIGH면 반시계 방향 (감소)
    else {
      rightEncoderCount--;
    }
  }
  // A상이 falling edge일 때
  else {
    // B가 HIGH면 시계 방향 (증가)
    if (B_val == HIGH) {
      rightEncoderCount++;
    } 
    // B가 LOW면 반시계 방향 (감소)
    else {
      rightEncoderCount--;
    }
  }
}

// 오른쪽 엔코더 B상 인터럽트 서비스 루틴
void rightEncoderB_ISR() {
  // A상과 B상의 현재 상태 읽기
  bool A_val = digitalRead(RIGHT_ENCODER_A);
  bool B_val = digitalRead(RIGHT_ENCODER_B);
  
  // 엔코더 카운트 업데이트 (A와 B의 위상 차이를 이용한 방향 감지)
  // B상이 rising edge일 때
  if (B_val == HIGH) {
    // A가 HIGH면 시계 방향 (증가)
    if (A_val == HIGH) {
      rightEncoderCount++;
    } 
    // A가 LOW면 반시계 방향 (감소)
    else {
      rightEncoderCount--;
    }
  }
  // B상이 falling edge일 때
  else {
    // A가 LOW면 시계 방향 (증가)
    if (A_val == LOW) {
      rightEncoderCount++;
    } 
    // A가 HIGH면 반시계 방향 (감소)
    else {
      rightEncoderCount--;
    }
  }
}