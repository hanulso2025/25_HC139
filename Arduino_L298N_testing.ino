/**
 * 2륜구동 로봇 위치 제어 코드
 * 사용 하드웨어:
 * - Arduino Mega 2560
 * - L298N 모터 드라이버
 * - 2개의 엔코더 모터 (95PPR, 감속비 1/200)
 */

// 모터 드라이버 핀 설정 (L298N)
// 왼쪽 모터
const int L_ENA = 9;   // 왼쪽 모터 Enable/PWM 핀
const int L_IN1 = 10;  // 왼쪽 모터 방향 제어 1
const int L_IN2 = 11;  // 왼쪽 모터 방향 제어 2

// 오른쪽 모터
const int R_ENB = 8;   // 오른쪽 모터 Enable/PWM 핀
const int R_IN3 = 6;   // 오른쪽 모터 방향 제어 1
const int R_IN4 = 7;   // 오른쪽 모터 방향 제어 2

// 엔코더 핀 설정 (인터럽트 핀)
const int LEFT_ENCODER_A = 2;   // 왼쪽 모터 엔코더 A상 (인터럽트 0)
const int LEFT_ENCODER_B = 3;   // 왼쪽 모터 엔코더 B상 (인터럽트 1)
const int RIGHT_ENCODER_A = 18; // 오른쪽 모터 엔코더 A상 (인터럽트 5)
const int RIGHT_ENCODER_B = 19; // 오른쪽 모터 엔코더 B상 (인터럽트 4)

// 엔코더 카운터 변수
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// 이동 방향 상태
enum Direction {
  STOPPED,
  FORWARD,
  BACKWARD
};

// 로봇 상태 변수
Direction currentDirection = STOPPED;
boolean isMoving = false;

// 목표 위치 변수
long leftTargetPosition = 0;
long rightTargetPosition = 0;

// PID 제어 변수
const float Kp = 0.05;  // 비례 게인
const float Ki = 0;  // 적분 게인
const float Kd = 0;  // 미분 게인

// PID 제어를 위한 변수
long leftPrevError = 0;
long rightPrevError = 0;
float leftIntegral = 0;
float rightIntegral = 0;

// 제어 관련 상수
const int MAX_PWM = 100;         // 최대 PWM 값
const int MIN_PWM = 0;         // 최소 PWM 값 
const int POSITION_TOLERANCE = 100; // 위치 허용 오차 (엔코더 카운트)

// 이동 거리 상수
const float WHEEL_DIAMETER = 7.2;                  // 바퀴 지름 (cm)
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI; // 바퀴 둘레 (cm)
const int ENCODER_PPR = 17;                        // 엔코더 PPR (1회전당 펄스 수)
const int GEAR_RATIO = 21;                        // 감속비 (1/200)
const int PULSES_PER_REVOLUTION = ENCODER_PPR * 4; // 4체배 엔코더 (A,B상 rising, falling edge 모두 카운트)
const float CM_PER_PULSE = WHEEL_CIRCUMFERENCE / (PULSES_PER_REVOLUTION * GEAR_RATIO); // (바퀴 둘레)/(바퀴 1회전에 발생되는 엣지수) = 엣지 하나당 움직이는 거리(cm)

// 시간 관련 변수
unsigned long previousMillis = 0;
const long CONTROL_INTERVAL = 10; // PID 제어 주기 (ms)

void setup() {
  // 시리얼 통신 초기화
  Serial.begin(9600);
  Serial.println("2-Wheel Robot Position Control System");
  Serial.println("Commands: 'forward', 'backward', 'test_pwm', 'direct_pwm', 'stop'");

  // 모터 드라이버 핀 설정
  pinMode(L_ENA, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_ENB, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);
  
  // PWM 출력 초기화
  analogWrite(L_ENA, 0);
  analogWrite(R_ENB, 0);
  
  // 시작 시 PWM 테스트 (펄스가 나오는지 확인용)
  Serial.println("Startup PWM test sequence");
  
  // 방향 핀 세팅
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, HIGH);
  digitalWrite(R_IN4, LOW);
  
  // 약한 PWM 출력
  analogWrite(L_ENA, 70);
  analogWrite(R_ENB, 70);
  Serial.println("PWM set to 70");
  delay(1000);
  
  // PWM 정지
  analogWrite(L_ENA, 0);
  analogWrite(R_ENB, 0);
  Serial.println("Startup PWM test completed");

  // 엔코더 핀 설정
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  // 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), leftEncoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), rightEncoderB_ISR, CHANGE);

  // 모터 정지
  stopMotors();
  
  Serial.println("System ready. Enter commands.");
}

void loop() {
  // 시리얼통신으로 받은 데이터가 존재하는 경우
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); //개행 전까지 읽기
    command.trim(); //공백제거
    // 'forward' 나 'backward' 명령만 시리얼 모니터로 준 경우 && 움직이고있지 않은 경우
    if (command == "forward" && !isMoving) {
      Serial.println("Moving forward...");
      moveForward(50); // 기본 forward 명령만 있는 경우 50cm 전진
    } 
    else if (command == "backward" && !isMoving) {
      Serial.println("Moving backward...");
      moveBackward(50); // 기본 backward 명령만 있는 경우 50cm 후진
    }
    // 'forward 30' or 'backward 20' 이런식으로 방향과 거리 명령도 준 경우 (거리는 cm단위로 알아먹는다.)
    else if (command.startsWith("forward ") && !isMoving) {
      int distance = command.substring(8).toInt(); //8번째 index 부터 끝까지 문자열 반환
      if (distance > 0) {
        Serial.print("Moving forward ");
        Serial.print(distance);
        Serial.println(" cm...");
        moveForward(distance);
      }
    }
    else if (command.startsWith("backward ") && !isMoving) {
      int distance = command.substring(9).toInt();
      if (distance > 0) {
        Serial.print("Moving backward ");
        Serial.print(distance);
        Serial.println(" cm...");
        moveBackward(distance);
      }
    }
    else if (command == "test_pwm") {
      Serial.println("Starting PWM test...");
      testPWM();
    }
    else if (command == "direct_pwm") {
      Serial.println("Starting direct PWM test...");
      directPWM();
    }
    else if (command == "stop") {
      Serial.println("Stopping motors...");
      stopMotors();
      isMoving = false;
      currentDirection = STOPPED;
    }
  }

  // 현재 시간 확인
  unsigned long currentMillis = millis();

  // PID 제어 주기에 따라 모터 제어
  if (isMoving && (currentMillis - previousMillis >= CONTROL_INTERVAL)) {
    previousMillis = currentMillis;
    
    // PID 제어 실행
    positionControl();
    
    // 목표 위치에 도달했는지 확인
    checkPositionReached();
  }
}

// 테스트용 함수 - 단순 PWM 출력
void directPWM() {
  // PWM 핀으로 직접 출력
  Serial.println("Direct PWM test");
  
  // 핀 모드 재설정
  pinMode(L_ENA, OUTPUT);
  pinMode(R_ENB, OUTPUT);
  
  // PWM 출력 테스트
  analogWrite(L_ENA, 200);
  analogWrite(R_ENB, 200);
  
  Serial.println("PWM set to 200");
  delay(5000);  // 5초 대기
  
  // PWM 출력 정지
  analogWrite(L_ENA, 0);
  analogWrite(R_ENB, 0);
  
  Serial.println("Direct PWM test completed");
}

// PWM 테스트 함수
void testPWM() {
  Serial.println("PWM 출력 테스트를 시작합니다...");
  
  // 모터 방향 핀 설정
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, HIGH);
  digitalWrite(R_IN4, LOW);
  
  // PWM 테스트 - 낮은 값부터 서서히 증가
  for (int pwm = 50; pwm <= 90; pwm += 20) {
    Serial.print("Setting PWM to: ");
    Serial.println(pwm);
    
    // PWM 출력 설정
    analogWrite(L_ENA, pwm);
    analogWrite(R_ENB, pwm);
    
    // 각 PWM 값에서 0.5초간 유지
    delay(500);
  }
  
  // 최대 PWM
  Serial.println("Setting PWM to MAX (255)");
  analogWrite(L_ENA, 255);
  analogWrite(R_ENB, 255);
  delay(2000);
  
  // 모터 정지
  stopMotors();
  Serial.println("PWM 출력 테스트 종료 ");
}

// 전진 명령 처리 함수
void moveForward(float distance) {
  // 엔코더 카운트 초기화
  resetEncoderCounts();
  
  // 목표 거리만큼 움직이기위해 얼마 만큼의 edge가 필요한지 계산해서 targetPulses에 저장
  long targetPulses = (long)(distance / CM_PER_PULSE);
  
  // 목표 위치 설정 (카운터 반대로쌓이는 경우 이거 부호 2개 반대로 바꿔주기)
  leftTargetPosition = targetPulses;
  rightTargetPosition = -targetPulses;  

  
  // PID 제어 변수 초기화 (각종 error 초기화)
  resetPIDVariables();
  
  // 이동 상태 설정
  currentDirection = FORWARD;
  isMoving = true;
  
  Serial.print("Target encoder count: ");
  Serial.println(targetPulses);
}

// 후진 명령 처리 함수
void moveBackward(float distance) {
  // 엔코더 카운트 초기화
  resetEncoderCounts();
  
  // 이동 거리를 엔코더 카운트로 변환
  long targetPulses = (long)(distance / CM_PER_PULSE);
  
  // 목표 위치 설정 (카운터 반대로쌓이면 이거 부호 2개 반대로 바꿔주기)
  leftTargetPosition = -targetPulses;
  rightTargetPosition = targetPulses;
  
  // PID 제어 변수 초기화
  resetPIDVariables();
  
  // 이동 상태 설정
  currentDirection = BACKWARD;
  isMoving = true;
  
  Serial.print("Target encoder count: ");
  Serial.println(targetPulses);
}

// PID 제어 변수(에러)초기화
void resetPIDVariables() {
  leftPrevError = 0;
  rightPrevError = 0;
  leftIntegral = 0;
  rightIntegral = 0;
}

// 엔코더 카운트 초기화
void resetEncoderCounts() {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
}

// 위치 제어 함수 (PID)
void positionControl() {
  // 현재 오차 계산
  long leftError = leftTargetPosition - leftEncoderCount;
  long rightError = rightTargetPosition - rightEncoderCount;
  
  // 적분 항 계산 (구분구적법과 유사하게 사각형을 계속합치는 거니까 사실상 적분임)
  leftIntegral += leftError*CONTROL_INTERVAL;
  rightIntegral += rightError*CONTROL_INTERVAL;
  
  // 적분 항 제한 (Anti-windup)
  leftIntegral = constrain(leftIntegral, -500, 500);
  rightIntegral = constrain(rightIntegral, -500, 500);
  
  // 미분 항 계산 (매우 짧은 시간에 해당하는 error 의 기울기니까 사실상 미분값임 )
  long leftDerivative = (leftError - leftPrevError)/CONTROL_INTERVAL;
  long rightDerivative = (rightError - rightPrevError)/CONTROL_INTERVAL;
  
  // PID 출력 계산
  float leftOutput = Kp * leftError + Ki * leftIntegral + Kd * leftDerivative;
  float rightOutput = Kp * rightError + Ki * rightIntegral + Kd * rightDerivative;
  
  // 이전 오차 저장
  leftPrevError = leftError;
  rightPrevError = rightError;
  
  // 출력이 0에 가까울 때 모터 정지 (데드존 설정)
  if (abs(leftOutput) < 10) leftOutput = 0;
  if (abs(rightOutput) < 10) rightOutput = 0;
  
  // PWM 값으로 변환 및 제한 - 무조건 MIN_PWM 이상 값 사용
  int leftPWM = 0;
  int rightPWM = 0;
  
  if (abs(leftOutput) > 0) {
    leftPWM = constrain(abs((int)leftOutput) , MIN_PWM, MAX_PWM);
  }
  
  if (abs(rightOutput) > 0) {
    rightPWM = constrain(abs((int)rightOutput) , MIN_PWM, MAX_PWM);
  }
  
  // 디버그 정보 출력
  Serial.print("L: ");
  Serial.print(leftEncoderCount);
  Serial.print("/");
  Serial.print(leftTargetPosition);
  Serial.print(" 실시간오차:");
  Serial.print(leftError);
  Serial.print(" 적분오차:");
  Serial.print(leftIntegral);
  Serial.print(" 미분오차:");
  Serial.print(leftDerivative);
  Serial.print(" PWM:");
  Serial.print(leftPWM);
  
  Serial.print(" | R: ");
  Serial.print(rightEncoderCount);
  Serial.print("/");
  Serial.print(rightTargetPosition);
  Serial.print(" 실시간오차:");
  Serial.print(rightError);
  Serial.print(" 적분오차:");
  Serial.print(rightIntegral);
  Serial.print(" 미분오차:");
  Serial.print(rightDerivative);
  Serial.print(" PWM:");
  Serial.println(rightPWM);
  
  // 모터 제어
  if (currentDirection == FORWARD) {
    setLeftMotorForward(leftPWM);
    setRightMotorForward(rightPWM);
  } else if (currentDirection == BACKWARD) {
    setLeftMotorBackward(leftPWM);
    setRightMotorBackward(rightPWM);
  }
}

// 목표 위치 도달 확인 함수
void checkPositionReached() {
  // 양쪽 모터 엔코더 카운트 반대로 싸이므로 절댓 값으로 오차 계산
  long leftError = abs(leftTargetPosition - leftEncoderCount);
  long rightError = abs(rightTargetPosition - rightEncoderCount);
  
  // 오차가 허용 범위 내에 있는지 확인
  if (leftError <= POSITION_TOLERANCE && rightError <= POSITION_TOLERANCE) {
    // 목표 위치에 도달함
    stopMotors();
    isMoving = false;
    currentDirection = STOPPED;
    
    Serial.println("Target position reached!");
    Serial.print("Final position - Left: ");
    Serial.print(leftEncoderCount);
    Serial.print(", Right: ");
    Serial.println(rightEncoderCount);
  }
}

// 왼쪽 모터 전진
void setLeftMotorForward(int pwm) {
  // 모터 방향 설정 (방향 핀이 먼저 설정되어야 함)
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  
  // 모터 속도 설정 (PWM 출력)
  // 최소 PWM이 효과가 없을 경우를 대비해 더 높은 값 사용
  pwm = max(pwm, MIN_PWM);
  analogWrite(L_ENA, pwm);
  
  // PWM 디버깅
  Serial.print("Left Motor PWM: ");
  Serial.println(pwm);
}

// 왼쪽 모터 후진
void setLeftMotorBackward(int pwm) {
  // 모터 방향 설정
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  
  // 모터 속도 설정 (PWM 출력)
  pwm = max(pwm, MIN_PWM);
  analogWrite(L_ENA, pwm);
  
  // PWM 디버깅
  Serial.print("Left Motor PWM: ");
  Serial.println(pwm);
}

// 오른쪽 모터 전진
void setRightMotorForward(int pwm) {
  // 모터 방향 설정
  digitalWrite(R_IN3, HIGH);
  digitalWrite(R_IN4, LOW);
  
  // 모터 속도 설정 (PWM 출력)
  pwm = max(pwm, MIN_PWM);
  analogWrite(R_ENB, pwm);
  
  // PWM 디버깅
  Serial.print("Right Motor PWM: ");
  Serial.println(pwm);
}

// 오른쪽 모터 후진
void setRightMotorBackward(int pwm) {
  // 모터 방향 설정
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, HIGH);
  
  // 모터 속도 설정 (PWM 출력)
  pwm = max(pwm, MIN_PWM);
  analogWrite(R_ENB, pwm);
  
  // PWM 디버깅
  Serial.print("Right Motor PWM: ");
  Serial.println(pwm);
}

// 모터 정지
void stopMotors() {
  // 모터 방향 핀 모두 LOW로 설정하여 정지
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_ENA, 0);
  
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, LOW);
  analogWrite(R_ENB, 0);
  
  Serial.println("Motors stopped");
}

// 엔코더 인터럽트 서비스 루틴(ISR) 함수들
void leftEncoderA_ISR() {
  // A상 변화 감지 시 방향에 따라 카운트 업데이트
  if (digitalRead(LEFT_ENCODER_A) == digitalRead(LEFT_ENCODER_B)) {
    leftEncoderCount--;
  } else {
    leftEncoderCount++;
  }
}

void leftEncoderB_ISR() {
  // B상 변화 감지 시 방향에 따라 카운트 업데이트
  if (digitalRead(LEFT_ENCODER_A) == digitalRead(LEFT_ENCODER_B)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void rightEncoderA_ISR() {
  // A상 변화 감지 시 방향에 따라 카운트 업데이트
  if (digitalRead(RIGHT_ENCODER_A) == digitalRead(RIGHT_ENCODER_B)) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}

void rightEncoderB_ISR() {
  // B상 변화 감지 시 방향에 따라 카운트 업데이트
  if (digitalRead(RIGHT_ENCODER_A) == digitalRead(RIGHT_ENCODER_B)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}