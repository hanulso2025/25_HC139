
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
