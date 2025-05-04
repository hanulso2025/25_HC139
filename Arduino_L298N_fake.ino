/**
 * 2륜구동 로봇 PID 제어 코드 (Arduino Mega 2560)
 * L298N 모터 드라이버와 엔코더 모터를 이용한 직선 왕복 제어
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
const int LEFT_ENCODER_A = 2;  // 왼쪽 모터 엔코더 A상 (인터럽트 0)
const int LEFT_ENCODER_B = 3;  // 왼쪽 모터 엔코더 B상 (인터럽트 1)
const int RIGHT_ENCODER_A = 18; // 오른쪽 모터 엔코더 A상 (인터럽트 5)
const int RIGHT_ENCODER_B = 19; // 오른쪽 모터 엔코더 B상 (인터럽트 4)

// 엔코더 카운트 변수
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// 엔코더 사양 및 로봇 파라미터
const float PULSES_PER_REVOLUTION = 17.0;  // PPR (95)
const float GEAR_RATIO = 21.0;            // 감속비 (1/200)
const float WHEEL_DIAMETER = 72.0;         // 바퀴 지름 (mm) - 실제 로봇에 맞게 조정 필요
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER; // 바퀴 둘레 (mm)

// 하나의 바퀴 회전당 엔코더 펄스 수 계산 (4체배 적용)
const float COUNTS_PER_REVOLUTION = PULSES_PER_REVOLUTION * 4 * GEAR_RATIO;

// mm당 엔코더 카운트 수. 1mm당 엔코더 카운트 몇에 상응하는지
const float COUNTS_PER_MM = COUNTS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

// PID 제어 상수 - 튜닝 필요
float KP = 1.0;  // 비례 상수
float KI = 0.05; // 적분 상수
float KD = 0.2;  // 미분 상수

// PID 제어 변수
long leftTargetCount = 0;
long rightTargetCount = 0;
long leftPrevError = 0;
long rightPrevError = 0;
float leftIntegral = 0;
float rightIntegral = 0;

// 이동 제어 변수
int baseSpeed = 150;        // 기본 PWM 값 (0-255)
int maxSpeed = 200;         // 최대 PWM 값
int minSpeed = 50;          // 최소 PWM 값
float slowdownDistance = 300.0; // 감속 시작 거리 (mm)
bool isMoving = false;      // 로봇 이동 중 상태
String moveDirection = "";  // 이동 방향 ("forward" 또는 "reverse")
float targetDistance = 0;   // 목표 이동 거리 (mm)

// 시간 관련 변수
unsigned long currentTime = 0;
unsigned long previousTime = 0;
const int pidInterval = 50; // PID 계산 간격 (ms)

void setup() {
  // 시리얼 통신 초기화
  Serial.begin(9600);  // USB 시리얼 모니터용
  Serial1.begin(9600); // UART 통신용 (추가 명령 수신)
  
  // 모터 드라이버 핀 설정
  pinMode(L_ENA, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_ENB, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);
  
  // 모터 초기 정지 상태
  digitalWrite(L_ENA, LOW);
  digitalWrite(R_ENB, LOW);
  
  // 엔코더 핀 설정
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  
  // 인터럽트 설정 - 각 엔코더 채널별로 별도 인터럽트 핸들러 연결
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), doLeftEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), doLeftEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), doRightEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), doRightEncoderB, CHANGE);
  
  Serial.println("2륜구동 로봇 PID 제어 시스템 초기화 완료 (L298N)");
  Serial.println("시리얼 모니터 또는 UART로 아래 명령어를 보내세요:");
  Serial.println("- '전진 [거리mm] [속도0-255]' (예: 전진 200 150)");
  Serial.println("- '후진 [거리mm] [속도0-255]' (예: 후진 300 180)");
  Serial.println("- '정지'");
}

void loop() {
  // 시리얼 명령 수신 확인 (USB 및 UART)
  checkCommand();
  
  // 로봇이 이동 중일 때 PID 제어 실행
  currentTime = millis();
  if (isMoving && (currentTime - previousTime >= pidInterval)) {
    updatePID();
    previousTime = currentTime;
    
    // 목표에 도달했는지 확인
    checkArrival();
  }
}

// 시리얼 명령 확인 (USB 및 UART 모두 지원)
void checkCommand() {
  // USB 시리얼 모니터에서 명령 확인
  checkSerialCommand(Serial);
  
  // UART Serial1에서 명령 확인
  checkSerialCommand(Serial1);
}

// 주어진 시리얼 포트에서 명령 처리
void checkSerialCommand(Stream &serialPort) {
  if (serialPort.available()) {
    String command = serialPort.readStringUntil('\n');
    command.trim();
    
    // 수신된 명령 확인 출력 (디버그용)
    Serial.print("수신된 명령: ");
    Serial.println(command);
    
    // 명령 분석
    if (command.startsWith("전진")) {
      // 전진 명령 형식: "전진 [거리mm] [속도0-255]" ex) 전진 200 220
      int firstSpace = command.indexOf(' ');
      int secondSpace = command.indexOf(' ', firstSpace + 1);
      
      if (firstSpace > 0 && secondSpace > 0) {
        float distance = command.substring(firstSpace + 1, secondSpace).toFloat();
        int speed = command.substring(secondSpace + 1).toInt();
        
        // 유효한 값 확인
        if (distance > 0 && speed > 0) {
          moveForward(distance, speed);
        }
      } else {
        // 기본값으로 전진
        moveForward(500, baseSpeed);
      }
    } 
    else if (command.startsWith("후진")) {
      // 후진 명령 형식: "후진 [거리mm] [속도0-255]"
      int firstSpace = command.indexOf(' ');
      int secondSpace = command.indexOf(' ', firstSpace + 1);
      
      if (firstSpace > 0 && secondSpace > 0) {
        float distance = command.substring(firstSpace + 1, secondSpace).toFloat();
        int speed = command.substring(secondSpace + 1).toInt();
        
        // 유효한 값 확인
        if (distance > 0 && speed > 0) {
          moveReverse(distance, speed);
        }
      } else {
        // 기본값으로 후진
        moveReverse(500, baseSpeed);
      }
    }
    else if (command == "정지") {
      stopMotors();
    }
    else if (command.startsWith("PID")) {
      // PID 파라미터 조정 명령: "PID [P] [I] [D]"
      int firstSpace = command.indexOf(' ');
      int secondSpace = command.indexOf(' ', firstSpace + 1);
      int thirdSpace = command.indexOf(' ', secondSpace + 1);
      
      if (firstSpace > 0 && secondSpace > 0 && thirdSpace > 0) {
        float kp = command.substring(firstSpace + 1, secondSpace).toFloat();
        float ki = command.substring(secondSpace + 1, thirdSpace).toFloat();
        float kd = command.substring(thirdSpace + 1).toFloat();
        
        // 유효한 값 확인
        if (kp >= 0 && ki >= 0 && kd >= 0) {
          KP = kp;
          KI = ki;
          KD = kd;
          Serial.print("PID 설정 변경: P=");
          Serial.print(KP);
          Serial.print(", I=");
          Serial.print(KI);
          Serial.print(", D=");
          Serial.println(KD);
        }
      }
    }
  }
}

// 왼쪽 모터 엔코더 인터럽트 핸들러 함수
void doLeftEncoderA() {
  // CCW 회전 시 증가(전진), CW 회전 시 감소(후진)
  leftEncoderCount += (digitalRead(LEFT_ENCODER_A) == digitalRead(LEFT_ENCODER_B)) ? 1 : -1;
}

void doLeftEncoderB() {
  leftEncoderCount += (digitalRead(LEFT_ENCODER_A) == digitalRead(LEFT_ENCODER_B)) ? -1 : 1;
}

// 오른쪽 모터 엔코더 인터럽트 핸들러 함수
void doRightEncoderA() {
  // CW 회전 시 증가(전진), CCW 회전 시 감소(후진)
  rightEncoderCount += (digitalRead(RIGHT_ENCODER_A) == digitalRead(RIGHT_ENCODER_B)) ? -1 : 1;
}

void doRightEncoderB() {
  rightEncoderCount += (digitalRead(RIGHT_ENCODER_A) == digitalRead(RIGHT_ENCODER_B)) ? 1 : -1;
}

// 전진 명령 처리
void moveForward(float distance, int speed) {
  // 이전 이동 중단
  stopMotors();
  resetPID();
  
  // 목표 거리 설정 (mm를 엔코더 카운트로 변환)
  targetDistance = distance;
  long targetCount = (long)(distance * COUNTS_PER_MM);
  leftTargetCount = leftEncoderCount + targetCount;
  rightTargetCount = rightEncoderCount + targetCount;
  
  // 속도 설정
  baseSpeed = constrain(speed, minSpeed, maxSpeed);
  
  // 이동 상태 설정
  isMoving = true;
  moveDirection = "forward";
  
  Serial.print("전진 시작: ");
  Serial.print(distance);
  Serial.print("mm, 속도: ");
  Serial.println(baseSpeed);
}

// 후진 명령 처리
void moveReverse(float distance, int speed) {
  // 이전 이동 중단
  stopMotors();
  resetPID();
  
  // 목표 거리 설정 (mm를 엔코더 카운트로 변환)
  targetDistance = distance;
  long targetCount = (long)(distance * COUNTS_PER_MM);
  leftTargetCount = leftEncoderCount - targetCount;  // 후진은 카운트 감소
  rightTargetCount = rightEncoderCount - targetCount;  // 후진은 카운트 감소
  
  // 속도 설정
  baseSpeed = constrain(speed, minSpeed, maxSpeed);
  
  // 이동 상태 설정
  isMoving = true;
  moveDirection = "reverse";
  
  Serial.print("후진 시작: ");
  Serial.print(distance);
  Serial.print("mm, 속도: ");
  Serial.println(baseSpeed);
}

// PID 제어 업데이트
void updatePID() {
  // 왼쪽 모터 PID 계산
  long leftError = leftTargetCount - leftEncoderCount;
  leftIntegral += leftError;
  long leftDerivative = leftError - leftPrevError;
  int leftPID = (KP * leftError) + (KI * leftIntegral) + (KD * leftDerivative);
  leftPrevError = leftError;
  
  // 오른쪽 모터 PID 계산
  long rightError = rightTargetCount - rightEncoderCount;
  rightIntegral += rightError;
  long rightDerivative = rightError - rightPrevError;
  int rightPID = (KP * rightError) + (KI * rightIntegral) + (KD * rightDerivative);
  rightPrevError = rightError;
  
  // 적분값 제한 (Anti-windup)
  leftIntegral = constrain(leftIntegral, -1000, 1000);
  rightIntegral = constrain(rightIntegral, -1000, 1000);
  
  // 거리에 따른 감속 계산(mm당 edge수 단위로 거리 업데이트)
  float leftRemainingDistance = abs((leftTargetCount - leftEncoderCount) / COUNTS_PER_MM);
  float rightRemainingDistance = abs((rightTargetCount - rightEncoderCount) / COUNTS_PER_MM);
  float remainingDistance = (leftRemainingDistance + rightRemainingDistance) / 2.0;
  
  // 감속 계수 계산 (목표 지점에 가까워질수록 속도 감소)
  float speedFactor = 1.0;
  if (remainingDistance < slowdownDistance) {
    speedFactor = remainingDistance / slowdownDistance;
    // 최소 속도 비율 보장
    speedFactor = max(speedFactor, 0.3);
  }

  speedFactor = 1.0; ////목표지점 가까워졌을떄느려지게 하고싶으면 이 코드 지우세요!! 근데 PID하면 자동으로 속도 느려지는거 아니야??
  
  // PWM 값 계산 및 제한
  int leftPWM = baseSpeed * speedFactor + leftPID;
  int rightPWM = baseSpeed * speedFactor + rightPID;
  
  leftPWM = constrain(leftPWM, minSpeed, maxSpeed);
  rightPWM = constrain(rightPWM, minSpeed, maxSpeed);
  
  // 모터 방향 및 속도 설정
  setMotorSpeeds(leftPWM, rightPWM);
  
  // 디버깅 정보 출력 (필요시 주석 해제)
  
  Serial.print("L: ");
  Serial.print(leftEncoderCount);
  Serial.print("/");
  Serial.print(leftTargetCount);
  Serial.print(" R: ");
  Serial.print(rightEncoderCount);
  Serial.print("/");
  Serial.print(rightTargetCount);
  Serial.print(" PWM L/R: ");
  Serial.print(leftPWM);
  Serial.print("/");
  Serial.print(rightPWM);
  Serial.print(" 남은거리: ");
  Serial.print(remainingDistance);
  Serial.print("mm 속도계수: ");
  Serial.println(speedFactor);
  
}

// 모터 속도 설정 (L298N 용으로 수정됨)
void setMotorSpeeds(int leftPWM, int rightPWM) {
  // PWM 값 제한
  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);
  
  if (moveDirection == "forward") {
    // 전진: 왼쪽 모터 CCW, 오른쪽 모터 CW
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_ENA, leftPWM);
    
    digitalWrite(R_IN3, HIGH);
    digitalWrite(R_IN4, LOW);
    analogWrite(R_ENB, rightPWM);
  } else if (moveDirection == "reverse") {
    // 후진: 왼쪽 모터 CW, 오른쪽 모터 CCW
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    analogWrite(L_ENA, leftPWM);
    
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, HIGH);
    analogWrite(R_ENB, rightPWM);
  } else {
    // 정지
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_ENA, 0);
    
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, LOW);
    analogWrite(R_ENB, 0);
  }
}

// 모터 정지
void stopMotors() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_ENA, 0);
  
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, LOW);
  analogWrite(R_ENB, 0);
  
  isMoving = false;
  moveDirection = "";
  
  Serial.println("모터 정지");
}

// 목표 도달 확인
void checkArrival() {
  long leftDiff = abs(leftTargetCount - leftEncoderCount);
  long rightDiff = abs(rightTargetCount - rightEncoderCount);
  
  // 양쪽 모터가 모두 목표 위치에 근접하면 정지
  if (leftDiff < 10 && rightDiff < 10) {
    Serial.println("목표 위치 도달!");
    stopMotors();
    
    // 최종 이동 거리 계산 및 출력
    float leftTraveledDistance = abs(leftEncoderCount / COUNTS_PER_MM);
    float rightTraveledDistance = abs(rightEncoderCount / COUNTS_PER_MM);
    float avgTraveledDistance = (leftTraveledDistance + rightTraveledDistance) / 2.0;
    
    Serial.print("이동 완료: ");
    Serial.print(avgTraveledDistance);
    Serial.println("mm");
  }
}

// PID 제어 변수 리셋
void resetPID() {
  leftPrevError = 0;
  rightPrevError = 0;
  leftIntegral = 0;
  rightIntegral = 0;
  leftEncoderCount = 0;
  rightEncoderCount = 0;
}