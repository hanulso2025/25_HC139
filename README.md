# [2025년 한이음 드림업 공모전]

## **💡1. 프로젝트 개요**

**1-1. 프로젝트 소개**
- 프로젝트 명 : UWB 기반 주차 유도 시스템 및 스마트 스토퍼
- 프로젝트 정의 : 차량의 위치를 정밀하게 추적하여 운전자에게 빈 주차공간까지 최적 경로를 안내하고, 장애인 구역 무단 진입을 차단하는 통합 주차 솔루션

  <img width="700" alt="image" src="images\image.png" /></br>

**1-2. 개발 배경 및 필요성**
- 현대 사회에서 주차 공간 부족은 운전자에게 큰 불편과 스트레스를 초래하고 있으며, 장시간의 주차 공간 탐색은 불필요한 공회전으로 이어져 환경 문제를 악화시키고 있습니다. 
- 장애인 전용 주차구역의 불법 주차는 최근 급격히 증가하여 사회적 갈등과 민원 문제로 대두되고 있습니다. 
- 이에 따라, 차량의 위치를 정밀하게 추적하고 장애인 구역 무단 점유를 사전에 차단할 수 있는 차세대 통합 주차 관리 시스템의 필요성이 커지고 있습니다.


**1-3. 프로젝트 특장점**
- UWB와 ESP32 기반 고정밀 위치 추적 (실내 주차장 환경에서도 안정적 측위 가능)
- 스마트 스토퍼를 통한 장애인 전용 주차구역 무단 진입 차단
- 차량 유형별 맞춤형 주차 배정 서비스 (일반/장애인/전기차)
- 차량 내 HMI 안내 및 관제 대시보드 실시간 모니터링
- ROS2 기반 통합 아키텍처 (micro-ROS로 ESP32까지 연동)

**1-4. 주요 기능**
- UWB 기반 위치 추적 : 태그–앵커 삼변측량 알고리즘으로 차량 위치를 센티미터 단위로 정밀 추적
- 경로 안내 기능 : 규칙 기반 알고리즘으로 빈 주차공간까지 최적 경로를 산출하고 HMI 화면에 시각화
- 목적지 기반 추천 : 사용자가 선택한 목적지 주변의 잔여 주차공간을 차량 유형별로 자동 추천  
  (ex. 백화점 내 영화관, 본관 입구, 문화시설 이용 고객에게 최적 주차 위치 제공)
- 차량 판별 기능 : BLE RSSI 신호를 활용해 차량을 자동 판별하고 유형(일반/전기/장애인) 분류
- 스마트 스토퍼 제어 : 인증된 장애인 차량만 진입을 허용하고 일반 차량은 물리적으로 차단
- 실시간 관제 서비스 : 관제 대시보드를 통해 전체 주차장 현황과 차량 이동 상태를 실시간 모니터링

**1-5. 기대 효과 및 활용 분야**
- 기대 효과 : 주차 공간 탐색 시간 단축으로 주차 효율성 향상, 장애인 전용 구역 무단 점유 방지로 이동권 보장, 주차 관리의 자동화로 운영 효율성 제고
- 활용 분야 : 대형 상업시설(백화점, 마트), 주거시설(아파트, 오피스텔), 공공기관 주차장, 스마트 시티 교통 및 주차 인프라 등 

**1-6. 기술 스택**

| 구분 | 사용 기술 |
|------|-----------|
| **UI** | ![PyQt5](https://img.shields.io/badge/PyQt5-41CD52?style=flat&logo=qt&logoColor=white) |
| **Framework** | ![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?style=flat&logo=ros&logoColor=white) ![micro-ROS](https://img.shields.io/badge/micro--ROS-lightgrey?style=flat&logo=ros&logoColor=white) |
| **Languages** | ![Python](https://img.shields.io/badge/Python-3776AB?style=flat&logo=python&logoColor=white) ![C](https://img.shields.io/badge/C-A8B9CC?style=flat&logo=c&logoColor=white) ![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=cplusplus&logoColor=white) |
| **Algorithms** | `Trilateration` `Kalman Filter` `Rule-based Path Planning` `Outlier Removal` `Median Filter` `Axis Alignment` |
| **MCU** | ![ESP32](https://img.shields.io/badge/ESP32-000000?style=flat&logo=espressif&logoColor=white) ![Arduino Mega](https://img.shields.io/badge/Arduino%20Mega-00979D?style=flat&logo=arduino&logoColor=white) |
| **SBC** | ![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-A22846?style=flat&logo=raspberrypi&logoColor=white) |
| **Communication** | `UWB (DWM1000)` `BLE` `Socket (TCP/IP)` `UART` `SPI` |
| **Project Management** | ![Git](https://img.shields.io/badge/Git-F05032?style=flat&logo=git&logoColor=white) ![GitHub](https://img.shields.io/badge/GitHub-181717?style=flat&logo=github&logoColor=white) ![Notion](https://img.shields.io/badge/Notion-000000?style=flat&logo=notion&logoColor=white) |

---

## **💡2. 팀원 소개**

| **한경빈 (팀장)** | **장준표** | **장서윤** | **정재윤** | **이유진** |
|:---:|:---:|:---:|:---:|:---:|
| <img width="120" height="150" src="images/한경빈.jpg" > | <img width="120" height="150" src="images/장준표.jpg" > | <img width="120" height="150" src="images/장서윤.jpg" > | <img width="120" height="150" src="images/정재윤.png" > | <img width="120" height="150" src="images/이유진.jpg" > |
| 🔗 [GitHub](https://github.com/hustlehan) | 🔗 [GitHub](https://github.com/IAMJP520) | 🔗 [GitHub](https://github.com/seoyun9) | 🔗 [GitHub](https://github.com/jjletsgo) | 🔗 [GitHub](https://github.com/euzin3) |
| • ROS2 노드 개발 <br> • 서버 및 관제 시스템 구축 <br> • 경로 계획 <br> &nbsp; | • 차량 HMI 인터페이스 구축 <br> • TCP/IP 기반 소켓 통신 모듈 개발 <br> &nbsp; | • 주차장 차단기 제어 <br> • BLE RSSI 기반 차량 정보 수신 <br> &nbsp; | • HW 및 시스템 아키텍처 설계 <br> • 스마트 스토퍼 제어 <br> • UWB 삼변측량 및 필터링 | • UWB 삼변측량 및 필터링 <br> &nbsp; <br> &nbsp; |
| ![Team Leader](https://img.shields.io/badge/Team-Leader-blue) <br> ![ROS2](https://img.shields.io/badge/ROS2-Node-brightgreen) <br> ![Server](https://img.shields.io/badge/Server-Control-lightblue) | ![HMI](https://img.shields.io/badge/HMI-Frontend-orange) <br> ![Comm](https://img.shields.io/badge/TCP%2FIP-Socket-yellowgreen) <br> &nbsp; | ![BLE](https://img.shields.io/badge/BLE-Comm-lightgrey) <br> ![Barrier](https://img.shields.io/badge/Barrier-Control-yellow) <br> &nbsp; | ![HW](https://img.shields.io/badge/HW-Design-brown) <br> ![Architecture](https://img.shields.io/badge/System-Architecture-lightseagreen) <br> ![UWB](https://img.shields.io/badge/UWB-Algorithm-red) | ![UWB](https://img.shields.io/badge/UWB-Algorithm-red) <br> &nbsp; <br> &nbsp; |


---
## **💡3. 시스템 구성도**
| **서비스 흐름도** |
|---|
| [<img src="images\서비스%20흐름도.png" width="700" alt="서비스 흐름도">](images\서비스%20흐름도.png) |

| **S/W 구성도** |
|---|
| [<img src="images\SW%20구성도.png" width="700" alt="S/W 구성도">](images\SW%20구성도.png) |

| **H/W 구성도** |
|---|
| [<img src="images\HW%20구성도.png" width="700" alt="H/W 구성도">](images\HW%20구성도.png) |

---
## **💡4. 작품 소개영상**
[![한이음 드림업 프로젝트 소개](images\1.png)](https://www.youtube.com/watch?v=wbkJ4pecB-A)

---
## **💡5. 핵심 소스코드**

- **삼변측량 (Trilateration 알고리즘)**  
UWB 태그와 앵커 사이의 거리 정보를 이용하여 차량의 2차원 좌표(x, y)를 계산하는 핵심 알고리즘입니다.  
3개의 앵커 좌표(`a[3]`)와 각각의 거리값(`r[3]`)을 입력받아, 선형방정식을 풀어 차량의 위치를 산출합니다.  
det(행렬식)이 0에 가까우면 해가 존재하지 않으므로 예외 처리를 수행합니다. 
  

```C
static bool trilat2D(const Vec3 a[3], const float r[3], float &x, float &y) {
    // 앵커 간 좌표 차이를 기반으로 행렬 A 구성
    float A11 = 2.0f*(a[1].x - a[0].x), A12 = 2.0f*(a[1].y - a[0].y);
    float A21 = 2.0f*(a[2].x - a[0].x), A22 = 2.0f*(a[2].y - a[0].y);

    // 각 앵커와 태그 사이의 거리 제곱과 좌표를 이용한 b 벡터 계산
    float b1  = (r[0]*r[0] - r[1]*r[1])
              + (a[1].x*a[1].x - a[0].x*a[0].x)
              + (a[1].y*a[1].y - a[0].y*a[0].y);
    float b2  = (r[0]*r[0] - r[2]*r[2])
              + (a[2].x*a[2].x - a[0].x*a[0].x)
              + (a[2].y*a[2].y - a[0].y*a[0].y);

    // 행렬식(det)이 0에 가까우면 연립방정식을 풀 수 없음
    float det = A11*A22 - A12*A21;
    if (fabsf(det) < 1e-6f) return false;

    // Cramer’s rule을 통해 (x, y) 좌표 계산
    x = ( b1*A22 - A12*b2) / det;
    y = (-b1*A21 + A11*b2) / det;
    return true;
}
```

- **Rule-based 경로 계획 알고리즘**  
차량이 진입한 후, 목적지 주차 구역까지의 waypoints를 계산하는 규칙 기반 알고리즘입니다.
주차장 구조를 미리 정의한 후 선택된 주차 구역 번호에 따라 필수 경유지를 다르게 설정합니다.
간단한 if-else 규칙을 통해 차량의 이동 경로를 유도하며, 목적지 좌표(target_waypoint)까지의 최적 경로를 반환합니다. 

```python
def calculate_waypoints(self, target_spot: int) -> List[Tuple[int, int]]:
        if target_spot not in self.parking_waypoints:
            return []
        
        # 선택된 주차 구역 좌표
        target_waypoint = self.parking_waypoints[target_spot]

        # 항상 거쳐야 하는 시작 지점 (입구)
        waypoints = [self.MANDATORY_WAYPOINT] 
        
        # 구역별 규칙 기반 경로 설정
        if target_spot == 1:              
            waypoints.append(target_waypoint)
        elif target_spot in [2, 3, 4, 5]:  
            waypoints.append((200, 1475))     
            waypoints.append(target_waypoint)
        elif target_spot == 6:  
            waypoints.append((200, 1475))
            waypoints.append((1475, 1475))    
            waypoints.append(target_waypoint)
        elif target_spot == 7:  
            waypoints.append((1475, 925))
            waypoints.append(target_waypoint)
        elif target_spot in [8, 9, 10, 11]: 
            waypoints.append(target_waypoint)
        
        return waypoints
```