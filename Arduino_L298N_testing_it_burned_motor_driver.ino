/*
 * 2륜구동 로봇의 PID 위치제어 코드
 * - Arduino Mega 2560 + L298N 모터 드라이버 + 2채널 증분형 엔코더(95PPR, 감속비 1/200)
 * - 'forward' 명령으로 전진, 'backward' 명령으로 후진
 * - PID 제어로 정확한 위치 제어
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

// 목표 위치 (엔코더 카운트)
long leftTargetPosition = 0;
long rightTargetPosition = 0;

// PID 제어 관련 변수
// PID 상수 (튜닝 필요)
float Kp = 0.1;   // 비례 상수
float Ki = 0.05;  // 적분 상수
float Kd = 0;   // 미분 상수

// PID 계산을 위한 변수
long leftPrevError = 0;   // 이전 오차
long rightPrevError = 0;  // 이전 오차
float leftIntegral = 0;   // 적분값
float rightIntegral = 0;  // 적분값

// 제어 주기 (ms)
const unsigned long CONTROL_INTERVAL = 10; // 10ms = 0.01초
unsigned long lastControlTime = 0;

// 엔코더 관련 상수 계산
const int ENCODER_PPR = 17;           // 엔코더 1회전당 펄스 수 (Pulse Per Revolution)
const float GEAR_RATIO = 21;       // 감속비 (1:200)
const float PULSES_PER_REV = ENCODER_PPR * 4 * GEAR_RATIO; // 4체배 모드 (A상, B상의 상승/하강 엣지) (모터의 1회전당 발생하는 펄스)

// 이동 거리 설정 (mm 단위)
const float WHEEL_DIAMETER = 72.0;   // 바퀴 지름 (mm), 실제 값으로 변경 필요
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER; // 바퀴 둘레 (mm)
const float DISTANCE_TO_MOVE = 1000.0;  // 이동할 거리 (mm)
//edge 1개당 0.1583 mm 이동

// 로봇이 움직이는 중인지 확인하는 플래그
bool isMoving = false;

// 명령어 수신 버퍼
String command = "";

void setup() {
  // 시리얼 통신 초기화
  Serial.begin(115200);
  Serial.println("2-Wheel Robot with PID Position Control");
  Serial.println("Send 'forward' or 'backward' to move the robot");
  
  // 모터 제어 핀 설정
  pinMode(L_ENA, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_ENB, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);
  
  // 엔코더 핀 설정
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  
  // 모터 정지
  stopMotors();
  
  // 인터럽트 설정 - 각 엔코더 핀의 상승/하강 엣지에서 인터럽트 발생
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), leftEncoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), rightEncoderB_ISR, CHANGE);
  
  // PWM 테스트
  Serial.println("Motor PWM Test (PWM: 70)");
  
  // 왼쪽 모터 테스트
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_ENA, 70);
  
  // 오른쪽 모터 테스트
  digitalWrite(R_IN3, HIGH);
  digitalWrite(R_IN4, LOW);
  analogWrite(R_ENB, 70);
  
  delay(500);  // 0.5초 동안 테스트
  stopMotors();
  delay(3000);
  Serial.println("Motor test complete. Ready for commands.");
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  Serial.println(leftEncoderCount);
  Serial.println(rightEncoderCount);
}

void loop() {
  // 시리얼 명령 읽기
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '\n') {
      processCommand();
      command = ""; // 명령어 버퍼 초기화
    } else {
      command += receivedChar;
    }
  }
  
  // PID 제어 로직 실행 (제어 주기마다)
  unsigned long currentTime = millis();
  if (isMoving && (currentTime - lastControlTime >= CONTROL_INTERVAL)) {
    lastControlTime = currentTime;
    updatePID();
    
    // 시리얼 플로터용 데이터 출력 (엔코더 카운트와 목표 위치)
    Serial.print("leftEncoderCount:");
    Serial.print(leftEncoderCount);
    Serial.print(",leftTargetPosition:");
    Serial.print(leftTargetPosition);
    Serial.print(",rightEncoderCount:");
    Serial.print(rightEncoderCount);
    Serial.print(",rightTargetPosition:");
    Serial.println(rightTargetPosition);
    
    // 목표 위치에 도달했는지 확인
    if (abs(leftTargetPosition - leftEncoderCount) < 100 && 
        abs(rightTargetPosition - rightEncoderCount) < 100) {
      stopMotors();

      //도착시 두 모터의 encodercount를 평균값으로 맞춰주기
      int a = (abs(leftEncoderCount)+abs(rightEncoderCount))/2;
      if (leftEncoderCount>=0) {
      leftEncoderCount = a;
      } else {
      leftEncoderCount = -a;
      }
      if (rightEncoderCount>=0) {
      rightEncoderCount = a;
      } else {
        rightEncoderCount = -a;
      }

      isMoving = false;
      Serial.println("Target position reached!");
    }
  }
}

// 명령어 처리 함수
void processCommand() {
  command.trim();  // 공백 제거
  
  if (command.equalsIgnoreCase("forward")) {
    moveForward();
  } 
  else if (command.equalsIgnoreCase("backward")) {
    moveBackward();
  }
  else {
    Serial.println("Unknown command. Use 'forward' or 'backward'.");
  }
}

// 전진 명령 처리
void moveForward() {
  // 현재 위치 기준으로 목표 위치 설정 (거리를 엔코더 카운트로 변환해서 pulses에 저장)
  long pulses = distanceToPulses(DISTANCE_TO_MOVE);
  Serial.println(pulses);
  leftTargetPosition = leftEncoderCount + pulses;
  rightTargetPosition = rightEncoderCount - pulses;
  
  // PID 제어 변수 초기화
  leftPrevError = 0;
  rightPrevError = 0;
  leftIntegral = 0;
  rightIntegral = 0;
  
  isMoving = true;
  Serial.println("Moving forward...");
}

// 후진 명령 처리
void moveBackward() {
  // 현재 위치 기준으로 목표 위치 설정 (거리를 엔코더 카운트로 변환)
  long pulses = distanceToPulses(DISTANCE_TO_MOVE);
  leftTargetPosition = leftEncoderCount - pulses;
  rightTargetPosition = rightEncoderCount - pulses;
  
  // PID 제어 변수 초기화
  leftPrevError = 0;
  rightPrevError = 0;
  leftIntegral = 0;
  rightIntegral = 0;
  
  isMoving = true;
  Serial.println("Moving backward...");
}

// 거리(mm)를 엔코더 펄스로 변환
long distanceToPulses(float distance_mm) {
  return (long)((distance_mm / WHEEL_CIRCUMFERENCE) * PULSES_PER_REV);
}

// PID 제어 업데이트
void updatePID() {
  // 왼쪽 모터 PID 계산
  long leftError = leftTargetPosition - leftEncoderCount;
  leftIntegral += leftError * CONTROL_INTERVAL;
  leftIntegral = constrain(leftIntegral, -100, 100);  // 적분값 제한
  long leftDerivative = (leftError - leftPrevError) / CONTROL_INTERVAL;
  float leftOutput = Kp * leftError + Ki * leftIntegral + Kd * leftDerivative;
  leftPrevError = leftError;
  
  // 오른쪽 모터 PID 계산
  long rightError = rightTargetPosition - rightEncoderCount;
  rightIntegral += rightError * CONTROL_INTERVAL;
  rightIntegral = constrain(rightIntegral, -100, 100);  // 적분값 제한
  long rightDerivative = (rightError - rightPrevError) / CONTROL_INTERVAL;
  float rightOutput = Kp * rightError + Ki * rightIntegral + Kd * rightDerivative;
  rightPrevError = rightError;
  
  // PWM 출력값 계산 (0~255 범위로 제한)
  int leftPWM = constrain(abs(leftOutput), 0, 100);
  int rightPWM = constrain(abs(rightOutput), 0, 100);
  
  // 모터 방향 설정 및 PWM 출력
  if (leftOutput > 0) {
    // 정방향
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
  } else {
    // 역방향
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
  }
  
  if (rightOutput < 0) {
    // 정방향
    digitalWrite(R_IN3, HIGH);
    digitalWrite(R_IN4, LOW);
  } else {
    // 역방향
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, HIGH);
  }
  
  // PWM 출력 (최소값 설정으로 모터 데드존 방지)
  //leftPWM = max(leftPWM, 30);  
  //rightPWM = max(rightPWM, 30);  
  
  analogWrite(L_ENA, leftPWM);
  analogWrite(R_ENB, rightPWM);
  
  // Serial Monitor에 PWM 값 출력
  Serial.print("Left PWM: ");
  Serial.print(leftPWM);
  Serial.print(", Right PWM: ");
  Serial.println(rightPWM);
}

// 모터 정지
void stopMotors() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, LOW);
  analogWrite(L_ENA, 0);
  analogWrite(R_ENB, 0);
}

// 엔코더 인터럽트 서비스 루틴 (ISR)
// 왼쪽 모터 엔코더 A상 ISR
void leftEncoderA_ISR() {
  // A상이 HIGH일 때
  if (digitalRead(LEFT_ENCODER_A) == HIGH) {
    // B상이 HIGH면 역방향, LOW면 정방향
    if (digitalRead(LEFT_ENCODER_B) == HIGH) {
      leftEncoderCount--;
    } else {
      leftEncoderCount++;
    }
  } 
  // A상이 LOW일 때
  else {
    // B상이 HIGH면 정방향, LOW면 역방향
    if (digitalRead(LEFT_ENCODER_B) == HIGH) {
      leftEncoderCount++;
    } else {
      leftEncoderCount--;
    }
  }
}

// 왼쪽 모터 엔코더 B상 ISR
void leftEncoderB_ISR() {
  // B상이 HIGH일 때
  if (digitalRead(LEFT_ENCODER_B) == HIGH) {
    // A상이 HIGH면 정방향, LOW면 역방향
    if (digitalRead(LEFT_ENCODER_A) == HIGH) {
      leftEncoderCount++;
    } else {
      leftEncoderCount--;
    }
  } 
  // B상이 LOW일 때
  else {
    // A상이 HIGH면 역방향, LOW면 정방향
    if (digitalRead(LEFT_ENCODER_A) == HIGH) {
      leftEncoderCount--;
    } else {
      leftEncoderCount++;
    }
  }
}

// 오른쪽 모터 엔코더 A상 ISR
void rightEncoderA_ISR() {
  // A상이 HIGH일 때
  if (digitalRead(RIGHT_ENCODER_A) == HIGH) {
    // B상이 HIGH면 역방향, LOW면 정방향
    if (digitalRead(RIGHT_ENCODER_B) == HIGH) {
      rightEncoderCount--;
    } else {
      rightEncoderCount++;
    }
  } 
  // A상이 LOW일 때
  else {
    // B상이 HIGH면 정방향, LOW면 역방향
    if (digitalRead(RIGHT_ENCODER_B) == HIGH) {
      rightEncoderCount++;
    } else {
      rightEncoderCount--;
    }
  }
}

// 오른쪽 모터 엔코더 B상 ISR
void rightEncoderB_ISR() {
  // B상이 HIGH일 때
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) {
    // A상이 HIGH면 정방향, LOW면 역방향
    if (digitalRead(RIGHT_ENCODER_A) == HIGH) {
      rightEncoderCount++;
    } else {
      rightEncoderCount--;
    }
  } 
  // B상이 LOW일 때
  else {
    // A상이 HIGH면 역방향, LOW면 정방향
    if (digitalRead(RIGHT_ENCODER_A) == HIGH) {
      rightEncoderCount--;
    } else {
      rightEncoderCount++;
    }
  }
}