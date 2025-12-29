# RMP401-ROS2

RMP401(Plus)'s Examples for ROS2.

/ Version 1.0


## Package Information

- Package:	segwayrmp, segway_msgs
- Node:		rmp401
 
Published Topics:
```

```

## Quick Start

Launch the node (defualt serial port is `/dev/ttyUSB0` for UART on the UP2 pin header):
```
ros2 launch segwayrmp rmp401.launch.py
```
```
ros2 launch segwayrmp rmp401.launch.py \
  params_file:=/path/to/your_params.yaml
```



# RMP PLUS 401 - ROS2 패키지 인터페이스 문서

## 1. 패키지 개요

### 1.1 기본 정보
- **패키지명**: segwayrmp
- **메시지 패키지**: segway_msgs
- **노드명**: SmartCar
- **지원 플랫폼**: x86_64, ARM64-v8a
- **통신 방식**: Serial (UART), CAN

### 1.2 제공 파일
| 파일명 | 설명 |
|--------|------|
| libctrl_x86_64.so | x86 플랫폼용 C/C++ 제어 라이브러리 |
| libctrl_arm64-v8a.so | ARM 플랫폼용 C/C++ 제어 라이브러리 |
| comm_ctrl_navigation.h | C/C++ API 헤더 파일 |
| ROS package | 섀시 제어를 위한 ROS 노드 |

## 2. 발행 토픽 (Published Topics)

### 2.1 배터리 정보
**토픽명**: `bms_fb`  
**메시지 타입**: `segway_msgs/Bms_fb`  
**주기**: 1 Hz  
**메시지 구조**:
```
int16 bat_soc           # 배터리 잔량 (1~100%)
int16 bat_charging      # 충전 상태 (0: 비충전, 1: 충전 중)
int32 bat_vol           # 배터리 전압 (mV)
int32 bat_current       # 배터리 전류 (mA)
int16 bat_temp          # 배터리 온도 (°C)
```

### 2.2 제어 명령 소스
**토픽명**: `chassis_ctrl_src_fb`  
**메시지 타입**: `segway_msgs/Chassis_ctrl_src_fb`  
**주기**: 1 Hz  
**메시지 구조**:
```
uint16 chassis_ctrl_cmd_src  # 제어 소스 (0: 리모컨, 1: 상위 시스템)
```

### 2.3 섀시 주행 거리
**토픽명**: `chassis_mileage_meter_fb`  
**메시지 타입**: `segway_msgs/Chassis_mileage_meter_fb`  
**주기**: 1 Hz  
**메시지 구조**:
```
uint32 vehicle_meters  # 누적 주행 거리 (m)
```

### 2.4 섀시 상태 모드
**토픽명**: `chassis_mode_fb`  
**메시지 타입**: `segway_msgs/Chassis_mode_fb`  
**주기**: 1 Hz  
**메시지 구조**:
```
uint16 chassis_mode  # 섀시 모드
                     # 0: 잠금(Lock)
                     # 1: 제어(Control)
                     # 2: 수동 밀기(Push)
                     # 3: 비상정지(Emergency Stop)
                     # 4: 오류(Error)
```

### 2.5 오류 코드
**토픽명**: `error_code_fb`  
**메시지 타입**: `segway_msgs/Error_code_fb`  
**주기**: 1 Hz  
**메시지 구조**:
```
uint32 host_error                 # 상위 시스템 오류
uint32 central_error              # 중앙 제어 보드 오류
uint32 front_left_motor_error     # 전방 좌측 모터 오류
uint32 front_right_motor_error    # 전방 우측 모터 오류
uint32 rear_left_motor_error      # 후방 좌측 모터 오류
uint32 rear_right_motor_error     # 후방 우측 모터 오류
uint32 bms_error                  # 배터리 관리 시스템 오류
```

### 2.6 모터 작동 모드
**토픽명**: `motor_work_mode_fb`  
**메시지 타입**: `segway_msgs/Motor_work_mode_fb`  
**주기**: 1 Hz  
**메시지 구조**:
```
uint16 motor_work_mode  # 모터 작동 상태
                        # 0: 토크 출력 없음
                        # 1: 토크 출력 중
```

### 2.7 속도 정보
**토픽명**: `speed_fb`  
**메시지 타입**: `segway_msgs/Speed_fb`  
**주기**: 40 Hz  
**메시지 구조**:
```
float32 car_speed          # 섀시 선속도 (m/s)
float32 turn_speed         # 섀시 각속도 (rad/s)
float32 fl_speed           # 전방 좌측 휠 속도
float32 fr_speed           # 전방 우측 휠 속도
float32 rl_speed           # 후방 좌측 휠 속도
float32 rr_speed           # 후방 우측 휠 속도
uint64 speed_timestamp     # 타임스탬프
```

### 2.8 엔코더 틱 정보
**토픽명**: `ticks_fb`  
**메시지 타입**: `segway_msgs/Ticks_fb`  
**주기**: 40 Hz  
**메시지 구조**:
```
int32 fl_ticks          # 전방 좌측 엔코더 틱
int32 fr_ticks          # 전방 우측 엔코더 틱
int32 rl_ticks          # 후방 좌측 엔코더 틱
int32 rr_ticks          # 후방 우측 엔코더 틱
uint64 ticks_timestamp  # 타임스탬프
```

### 2.9 오도메트리
**토픽명**: `odom`  
**메시지 타입**: `nav_msgs/Odometry`  
**주기**: 40 Hz  
**설명**: 섀시의 위치, 방향, 속도 정보 (시작 시 heading 각도는 0도 기준)

### 2.10 IMU 데이터
**토픽명**: `imu`  
**메시지 타입**: `sensor_msgs/Imu`  
**주기**: 40 Hz  
**설명**: 자이로스코프 및 가속도계 데이터 (좌표계: X=우측, Y=전방, Z=상방)

## 3. 구독 토픽 (Subscribed Topics)

### 3.1 속도 제어 명령
**토픽명**: `cmd_vel`  
**메시지 타입**: `geometry_msgs/Twist`  
**메시지 구조**:
```
geometry_msgs/Vector3 linear
  float64 x    # 선속도 (m/s)
geometry_msgs/Vector3 angular
  float64 z    # 각속도 (rad/s)
```
**주의사항**: 제어 모드에서 150ms 이내에 명령을 수신하지 못하면 통신 실패로 판단

## 4. 서비스 클라이언트

### 4.1 이벤트 전송
**서비스명**: `chassis_send_event_srv`  
**메시지 타입**: `segway_msgs/chassis_send_event`  
**기능**: 이벤트 번호 전송
**요청**:
```
uint16 chassis_send_event_id
```
**응답**:
```
bool ros_is_received
```

## 5. 서비스 서버

### 5.1 오류 코드 초기화
**서비스명**: `ros_clear_chassis_error_code_cmd`  
**메시지 타입**: `segway_msgs/ros_clear_chassis_error_code_cmd`  
**요청**:
```
bool clear_chassis_error_code_cmd
```
**응답**:
```
uint8 clear_chassis_error_code_result
```
**주의**: 경고, 예외, 배터리 오류는 제외. 신중히 사용 필요

### 5.2 제자리 회전 활성화
**서비스명**: `ros_enable_chassis_rotate_cmd`  
**메시지 타입**: `segway_msgs/ros_enable_chassis_rotate_cmd`  
**요청**:
```
bool ros_enable_chassis_rotate_cmd
```
**응답**:
```
int16 chassis_enable_rotate_result
```

### 5.3 제자리 회전 상태 조회
**서비스명**: `ros_get_chassis_rotate_switch_cmd`  
**메시지 타입**: `segway_msgs/ros_get_chassis_rotate_switch_cmd`  
**요청**:
```
bool ros_get_chassis_rotate_cmd
```
**응답**:
```
uint8 chassis_rotate_state
```

### 5.4 섀시 SN 조회
**서비스명**: `ros_get_chassis_SN_cmd`  
**메시지 타입**: `segway_msgs/ros_get_chassis_SN_cmd`  
**요청**:
```
bool ros_get_chassis_SN
```
**응답**:
```
string chassis_SN
```

### 5.5 적재 파라미터 조회
**서비스명**: `ros_get_load_param_cmd`  
**메시지 타입**: `segway_msgs/ros_get_load_param_cmd`  
**응답**:
```
uint8 get_load_param  # 0: 무부하, 1: 만재
```

### 5.6 소프트웨어 버전 조회
**서비스명**: `ros_get_sw_version_cmd`  
**메시지 타입**: `segway_msgs/ros_get_sw_version_cmd`  
**응답**:
```
uint16 host_version      # 상위 시스템 버전
uint16 central_version   # 중앙 보드 버전
uint16 motor_version     # 모터 보드 버전
```

### 5.7 속도 제한값 피드백 조회
**서비스명**: `ros_get_vel_max_feedback_cmd`  
**메시지 타입**: `segway_msgs/ros_get_vel_max_feedback_cmd`  
**응답**:
```
float32 forward_max_vel_fb   # 전진 최대 속도 (m/s)
float32 backward_max_vel_fb  # 후진 최대 속도 (m/s)
float32 angular_max_vel_fb   # 최대 각속도 (rad/s)
```

### 5.8 섀시 활성화
**서비스명**: `ros_set_chassis_enable_cmd`  
**메시지 타입**: `segway_msgs/ros_set_chassis_enable_cmd`  
**요청**:
```
bool ros_set_chassis_enable_cmd  # true: 활성화, false: 비활성화
```
**응답**:
```
uint8 chassis_set_chassis_enable_result
```

### 5.9 섀시 전원 종료
**서비스명**: `ros_set_chassis_poweroff_cmd`  
**메시지 타입**: `segway_msgs/ros_set_chassis_poweroff_cmd`  
**요청**:
```
bool ros_set_chassis_poweroff_cmd
```
**응답**:
```
uint8 chassis_set_poweroff_result
```

### 5.10 적재 파라미터 설정
**서비스명**: `ros_set_load_param_cmd`  
**메시지 타입**: `segway_msgs/ros_set_load_param_cmd`  
**요청**:
```
uint8 ros_set_load_param  # 0: 무부하, 1: 만재
```
**응답**:
```
uint8 chassis_set_load_param_result
```

### 5.11 속도 제한 설정
**서비스명**: `ros_set_vel_max_cmd`  
**메시지 타입**: `segway_msgs/ros_set_vel_max_cmd`  
**요청**:
```
float32 ros_set_forward_max_vel   # 전진 최대 속도 (m/s, 0~2.3)
float32 ros_set_backward_max_vel  # 후진 최대 속도 (m/s, -0.85~0)
float32 ros_set_angular_max_vel   # 최대 각속도 (rad/s, 0~2)
```
**응답**:
```
uint8 chassis_set_max_vel_result
```

### 5.12 상위 시스템 전원 리셋
**서비스명**: `ros_reset_host_power_cmd`  
**메시지 타입**: `segway_msgs/ros_reset_host_power_cmd`  
**요청**:
```
uint16 reset_interval_time  # 리셋 간격 시간 (초, 0~65535)
```
**응답**:
```
uint8 reset_result
```

## 6. 액션 서버

### 6.1 펌웨어 IAP 업그레이드
**액션명**: `ros_set_iap_cmd_action`  
**메시지 타입**: `segway_msgs/ros_set_iap_cmdAction`  
**Goal**:
```
uint16 board_index_for_iap  # 업그레이드 대상 보드
```
**Result**:
```
int16 iap_result     # 3: 완료, 4: 실패, 5: 중단
int16 error_code     # iap_result가 4일 때 오류 코드
```
**Feedback**:
```
int16 iap_percent    # 업그레이드 진행률 (0~100%)
```

## 7. Callback 데이터 타입

### 7.1 4륜 속도 정보
**타입**: `Chassis_Data_Motors_Speed` (Index: 1)
```c
typedef struct {
    int16_t fl_speed;  # 전방 좌측
    int16_t fr_speed;  # 전방 우측
    int16_t rl_speed;  # 후방 좌측
    int16_t rr_speed;  # 후방 우측
} chassis_motors_speed_data_t;
```

### 7.2 섀시 속도 정보
**타입**: `Chassis_Data_Car_Speed` (Index: 2)
```c
typedef struct {
    int16_t car_speed;   # 선속도
    int16_t turn_speed;  # 각속도
} chassis_car_speed_data_t;
```

### 7.3 전방 휠 엔코더
**타입**: `Chassis_Data_Front_Ticks` (Index: 3)
```c
typedef struct {
    int32_t fl_ticks;  # 전방 좌측 틱
    int32_t fr_ticks;  # 전방 우측 틱
} front_motors_ticks_t;
```

### 7.4 후방 휠 엔코더
**타입**: `Chassis_Data_Rear_Ticks` (Index: 4)
```c
typedef struct {
    int32_t rl_ticks;  # 후방 좌측 틱
    int32_t rr_ticks;  # 후방 우측 틱
} rear_motors_ticks_t;
```

### 7.5 Odom 위치
**타입**: `Chassis_Data_Odom_Pose_xy` (Index: 5)
```c
typedef struct {
    float pos_x;
    float pos_y;
} odom_pos_xy_t;
```

### 7.6 Odom Euler X/Y
**타입**: `Chassis_Data_Odom_Euler_xy` (Index: 6)
```c
typedef struct {
    float euler_x;
    float euler_y;
} odom_euler_xy_t;
```

### 7.7 Odom Euler Z
**타입**: `Chassis_Data_Odom_Euler_z` (Index: 7)
```c
typedef struct {
    float euler_z;
} odom_euler_z_t;
```

### 7.8 Odom 선속도
**타입**: `Chassis_Data_Odom_Linevel_xy` (Index: 8)
```c
typedef struct {
    float vel_line_x;
    float vel_line_y;
} odom_vel_line_xy_t;
```

### 7.9 자이로스코프
**타입**: `Chassis_Data_Imu_Gyr` (Index: 9)
```c
typedef struct {
    int16_t gyr[3];  # X, Y, Z 각속도
} imu_gyr_original_data_t;
```

### 7.10 가속도계
**타입**: `Chassis_Data_Imu_Acc` (Index: 10)
```c
typedef struct {
    int16_t acc[3];  # X, Y, Z 가속도
} imu_acc_original_data_t;
```

## 8. 이벤트 정의

| 이벤트 타입 | Index | 설명 |
|------------|-------|------|
| ChassisBootReadyEvent | 1 | 섀시 중앙 패널 시작 완료 |
| PadPowerOffEvent | 2 | 섀시 전원 종료 |
| OnEmergeStopEvent | 3 | 비상정지 진입 |
| OutEmergeStopEvent | 4 | 비상정지 해제 |
| OnLockedRotorProtectEvent | 5 | 휠 구속 보호 발생 |
| OutLockedRotorProtectEvent | 6 | 휠 구속 보호 해제 |
| OnLostCtrlProtectEvent | 7 | 제어 불능 회전 발생 |
| OutLostCtrlProtectEvent | 8 | 제어 불능 회전 해제 |
| CalibrateGyroSuccess | 9 | 자이로 캘리브레이션 성공 |
| CalibrateGyroFail | 10 | 자이로 캘리브레이션 실패 |
| CalibratePasheCurrentSuccess | 11 | 위상 전류 캘리브레이션 성공 |
| CalibratePasheCurrentFail | 12 | 위상 전류 캘리브레이션 실패 |
| ChassisLockRotorWarning | 13 | 구속 회전 경고 |


## 10. 주요 사양

| 항목 | 사양 |
|------|------|
| 크기 (L×W×H) | 672 × 617 × 274 mm |
| 축거 × 윤거 × 지상고 | 456 × 545 × 58 mm |
| 타이어 크기 | 8.5 inch |
| 자체 중량 | 28 kg |
| 정격 적재량 | 28 kg |
| 최대 속도 | 3.56 m/s |
| 최대 조향 속도 | 2 rad/s |
| 최소 회전 반경 | 1.36 m |
| 제동 거리 | 만재 3.56m/s에서 약 1m |
| 배터리 | 36V 15.3Ah |
| 주행 거리 | 만재 2m/s에서 약 40km |
| 방수 등급 | IPX5 |
| 통신 인터페이스 | UART, CAN |

## 11. 주의사항

### 11.1 제어 명령
- 제어 모드에서 `cmd_vel` 토픽은 150ms 이내 주기로 지속 발행 필요
- 통신 두절 시 자동으로 잠금 모드로 전환

### 11.2 제자리 회전 기능
- 선속도가 0이고 각속도가 0이 아닐 때 작동
- 후륜 전류 과부하 가능성으로 필요시에만 사용 권장
- 구속 발생 시:
  - 약 5초 후: 제자리 회전 기능 취소, 경고 이벤트 발생
  - 약 10초 후: 자동 전기 에너지 방출
  - 약 15초 후: 구속 오류 이벤트 발생, 오류 모드 전환
- 기능 재활성화는 중지 후 30초 경과 후 가능

### 11.3 오류 코드 초기화
- `clear_chassis_error_code` 서비스는 신중히 사용
- 경고, 예외, 배터리 오류는 제외됨
- 복구 불가능한 오류는 하드웨어 점검 필요

### 11.4 좌표계
- **Odom 데이터**: 시작 시 heading 각도 0도 기준
- **IMU 좌표계**: X=우측, Y=전방, Z=상방
