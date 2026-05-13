# Go2 + Ouster + GLIM 3D 매핑 패키지 설계 문서

> ROS2 Humble · CUDA 13 GLIM Docker 환경 대상.
> 시뮬레이션(Gazebo Classic 11) 1차 검증 → 추후 실물 Go2+Ouster.
> 본 문서의 사실 주장은 [리서치 근거](#부록-a--리서치-근거인용) 절에 1차 소스로 인용됨.

---

## 1. 목표와 범위

Unitree Go2 + Ouster LiDAR로 **3D 포인트클라우드 맵을 생성·저장**하는 ROS2 Humble 패키지를 만든다. SLAM 엔진은 koide3 **GLIM**(`glim` + `glim_ros2`). 1차로 **Gazebo Classic 시뮬**에서 파이프라인 전체(센서 입력 → GLIM → 맵 저장)를 검증한다. 실물 로봇은 당분간 사용하지 않으나, 토픽 계약은 실물 `ouster-ros` 드라이버와 호환되게 설계한다.

**범위 밖(이번 단계 제외)**: 2D occupancy grid / Nav2 연동, 실시간 자율주행, 멀티세션 재최적화.

### 확정된 설계 결정

| 항목 | 결정 | 근거 |
|------|------|------|
| 시뮬레이터 | **Gazebo Classic 11** | Humble 네이티브 페어링, Go2 URDF 가용, 플러그인 성숙. EOL이나 sim 전용 개발엔 무방 |
| LiDAR | Ouster **OS1 프로파일** (Gazebo `gpu_ray`/velodyne 플러그인 재설정) | Gazebo에 네이티브 Ouster 플러그인 없음 → 기존 ray 센서를 OS1 스펙으로 구성 |
| Odometry | **IMU 기반(기본 모드)**, CT 모드 회피 | CT는 per-point time 필수, sim LiDAR는 미제공 ([F8](#f8)) |
| 디스큐 | **비활성** (`global_shutter_lidar: true`) | sim LiDAR per-point 타임스탬프 없음 → GLIM 공식 플래그로 처리 ([F7](#f7)) |
| 맵 저장 | 3D만. `/tmp/dump` → offline_viewer PLY → Open3D PCD | GLIM 라이브 PCD 네이티브 없음 ([F5](#f5)) |
| 패키지 타입 | **ament_python (오케스트레이션 전용)** | `glim_ros`는 ament_cmake C++. 빌드 안 하고 런치/config/export만 ([F2](#f2)) |

---

## 2. 큰 그림 — 데이터 흐름

```
┌─────────────────────────────────────────────────────────────────────┐
│ Gazebo Classic 11  (use_sim_time:=true)                              │
│                                                                       │
│  Go2 URDF (champ 기반)                                                │
│    └─ os1_sensor 링크                                                  │
│         ├─ gpu_ray 플러그인 (OS1 프로파일: 64ch, 360°, 45° V-FOV)      │
│         │     → sensor_msgs/PointCloud2   /go2/os1/points              │
│         └─ imu 플러그인 (update_rate 200Hz)                            │
│               → sensor_msgs/Imu           /go2/os1/imu                 │
└──────────────────────────────┬────────────────────────────────────────┘
                               │  launch remap
                               ▼
        /os_cloud_node/points   /os_cloud_node/imu      ← GLIM 기본 토픽명 [F3]
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│ glim_rosnode  (config_path:=<pkg>/config/glim)         [F4]           │
│   - config_sensors.json:  T_lidar_imu, global_shutter_lidar=true      │
│   - config_ros.json:      points_topic / imu_topic                    │
│   - config_odometry_*.json (IMU 모드, GPU 또는 CPU)                    │
│                                                                       │
│   라이브 → RViz (rviz_viewer) + Iridescence GUI 뷰어                  │
│   노드 종료 시 → /tmp/dump (팩터그래프 덤프)            [F6]           │
└──────────────────────────────┬────────────────────────────────────────┘
                               │  (수동, 1회)
                               ▼
   ros2 run glim_ros offline_viewer  → File>Save>Export Points → map.ply  [F5]
                               │
                               ▼
   ply_to_pcd.py (Open3D)  → map.pcd        ← 본 패키지 헬퍼 노드        [F5]
```

**핵심 통찰**: ament_python 패키지는 GLIM을 빌드하지 않는다. `glim_rosnode`를 **런치**하고, 시뮬 토픽을 GLIM 기본 토픽명으로 **리맵**하고, **config를 주입**하고, 종료 후 덤프를 **PCD로 후처리**하는 글루(glue)다.

---

## 3. GLIM 입력 계약 (검증 완료)

### 3.1 토픽 ([F3](#f3))

`glim_ros`(패키지명, v1.2.1, ament_cmake)는 `config_ros.json`에서 **단수형** 키로 토픽을 읽는다:

```jsonc
// config_ros.json  (glim_ros 블록)
"imu_topic":    "/os_cloud_node/imu",     // 기본값 (레거시 Ouster 이름)
"points_topic": "/os_cloud_node/points"
```

- ⚠️ `imu_topics` / `points_topics` (복수형)은 **존재하지 않음** — 리서치에서 기각된 주장 ([기각](#기각된-주장)).
- ⚠️ 최신 `ouster-ros` 실물 드라이버는 `/ouster/points`, `/ouster/imu`를 발행 → 실물에서도 리맵 필요.
- 메시지 타입: `sensor_msgs/PointCloud2`, `sensor_msgs/Imu`. `tf2_ros`로 프레임 처리.

**전략**: 시뮬 발행 토픽(`/go2/os1/points`, `/go2/os1/imu`)을 launch `remappings`로 GLIM 기본명에 매핑. config 파일은 기본값 그대로 두면 됨.

### 3.2 외부 파라미터 T_lidar_imu ([F4](#f4))

`config_sensors.json`의 `T_lidar_imu` = IMU 프레임 → LiDAR 프레임 변환, TUM SE3 `[x,y,z,qx,qy,qz,qw]`, REP-145(정지 시 IMU +Z ≈ +9.81).

```
p_lidar    = T_lidar_imu * p_imu
T_world_imu = T_world_lidar * T_lidar_imu
```

**할 일**: Go2 URDF의 os1_sensor 링크 ↔ imu 링크 상대 변환을 그대로 `T_lidar_imu`에 기입. URDF에서 두 센서를 **동일 링크에 동일 포즈로** 부착하면 `T_lidar_imu = identity [0,0,0,0,0,0,1]`로 단순화 가능(시뮬 권장).

### 3.3 타임스탬프 없는 sim LiDAR 처리 (가장 중요한 함정)

GLIM의 `extract_raw_points()`는 PointCloud2에서 `t`/`time`/`time_stamp`/`timestamp` 필드를 찾아 per-point 시간을 만든다. 없으면 `raw_points->times`는 **빈 채로** 남는다.

Gazebo `gpu_ray`/velodyne 플러그인은 per-point 타임스탬프를 **발행하지 않음**.

> ✅ **스파이크 #1로 경험적 해결 (2026-06-10) — [결과](#11-스파이크-1-결과-검증-완료) 참조.** 결론: **republisher 노드 불필요.** GLIM `TimeKeeper`(`time_keeper.cpp:99-109`)가 `points->times`가 비면 **전처리 이전 단계에서** 점 개수만큼 times를 채우고 스캔 duration 기반 의사 타임스탬프를 할당("use pseudo per-point timestamps based on the order of points"). 따라서 `cloud_preprocessor.cpp:135`의 sort가 빈 times를 접근할 일이 없음. **이 동작은 `global_shutter_lidar`와 무관한 기본 동작** — 기본 config로도 무타임스탬프 클라우드 처리됨.

정리:
- **권장 설정**: `global_shutter_lidar: true`. **이유**: Gazebo `gpu_ray` 클라우드의 *점 순서*는 실제 스캔 *시간순*과 일치하지 않으므로, TimeKeeper가 순서 기반으로 만든 의사 타임스탬프로 deskew하면 왜곡을 *제거*가 아니라 *주입*할 수 있음. zero-fill(deskew off)이 안전. (단 스파이크 로그에 찍힌 "pseudo per-point timestamps"는 TimeKeeper 경로 증거이지 `cloud_preprocessor.cpp:138` zero-fill 발화를 직접 증명한 것은 아님 — flag 미설정으로도 크래시 없이 동작하나, 안전상 true 권장.)
- **CT odometry 회피 ([F8](#f8))**: `odometry_estimation_ct`는 `frame->times[i]` 직접 인덱싱 → 사용 금지. **IMU 모드(기본 `config_odometry_gpu.json`) 사용** — 스파이크에서 검증됨.
- ⚠️ **타임스탬프 단조성 필수**: GLIM은 큰 폭의 **역방향(backward) 타임스탬프**에 `IndexedSlidingWindow: index out of range`로 throw(graceful 아님). Gazebo는 `use_sim_time:=true`로 단조 보장. 다중 인스턴스/잔류 퍼블리셔로 인한 스탬프 크로스토크 주의.

---

## 4. ament_python 패키지 설계

### 4.1 생성

```bash
cd ~/glim-humble-docker/src      # 서브모듈 glim, glim_ros2 와 동일 레벨
ros2 pkg create --build-type ament_python go2_glim_mapping \
  --dependencies rclpy sensor_msgs std_srvs launch launch_ros
```

> 패키지 위치: `src/go2_glim_mapping` — `src/glim` / `src/glim_ros2` 서브모듈과 형제. 레포 루트에서 `colcon build`가 셋을 모두 빌드.

### 4.2 디렉터리 레이아웃

```
go2_glim_mapping/
├── package.xml
├── setup.py                      # data_files로 launch/config/rviz/urdf 설치
├── setup.cfg
├── resource/go2_glim_mapping
├── go2_glim_mapping/
│   ├── __init__.py
│   ├── ply_to_pcd.py             # /tmp/dump PLY → PCD 변환 (Open3D)
│   └── lidar_time_republisher.py # (폴백, 스파이크 #1 실패 시만) 합성 t 필드 추가
├── launch/
│   ├── sim_gazebo.launch.py      # Gazebo Classic + Go2 + OS1 LiDAR/IMU 스폰
│   ├── glim_mapping.launch.py    # glim_rosnode + 토픽 리맵 + config 경로
│   └── bringup.launch.py         # 위 둘 + RViz, use_sim_time:=true
├── config/
│   ├── glim/                     # GLIM config 오버라이드 (기본에서 복사 후 수정)
│   │   ├── config.json
│   │   ├── config_ros.json
│   │   ├── config_sensors.json   # global_shutter_lidar=true, T_lidar_imu
│   │   ├── config_preprocess.json
│   │   └── config_odometry_gpu.json (또는 _cpu)
│   └── rviz/mapping.rviz
└── urdf/
    └── ouster_os1.urdf.xacro     # OS1 gpu_ray + IMU 센서 매크로 (Go2에 부착)
```

### 4.3 노드 / 실행 단위

| 단위 | 종류 | 역할 |
|------|------|------|
| `glim_rosnode` | 외부(glim_ros2) | 라이브 SLAM. 본 패키지가 런치만 함 ([F4](#f4)) |
| `ply_to_pcd.py` | 본 패키지 rclpy 아님(CLI 스크립트) | Open3D `read_point_cloud`→`write_point_cloud`로 PLY→PCD. intensity/time 필드 손실 주의 ([F5](#f5)) |
| ~~`lidar_time_republisher.py`~~ | **불필요** | 스파이크 #1에서 GLIM이 무타임스탬프 클라우드 자동 처리 확인 → 노드 목록에서 제거 |

> `offline_viewer`(PLY export)와 `map_editor`는 `BUILD_WITH_VIEWER=ON` + X11 필요 ([F4](#f4)). 본 레포 `install-deps.sh`가 iridescence를 빌드하므로 뷰어 지원 존재 — **단 Docker에서 X11 포워딩 동작 확인 필요(스파이크 #3)**.

### 4.4 launch 핵심 (의사코드)

```python
# glim_mapping.launch.py
Node(
    package='glim_ros', executable='glim_rosnode', output='screen',
    parameters=[{
        'config_path': PathJoinSubstitution([pkg_share, 'config', 'glim']),
        'use_sim_time': True,
    }],
    remappings=[
        ('/os_cloud_node/points', '/go2/os1/points'),
        ('/os_cloud_node/imu',    '/go2/os1/imu'),
    ],
)
```

> 리맵 방향 주의: GLIM 구독 토픽(`/os_cloud_node/*`)을 시뮬 발행 토픽으로 매핑. 또는 config 파일에서 `points_topic`을 직접 `/go2/os1/points`로 적어도 됨(리맵보다 명시적).
>
> **`config_path` 메커니즘 (소스 확인 `glim_ros.cpp:69-75`)**: ROS2 파라미터. **절대경로**면 그대로, 상대경로면 `glim` 패키지 share 디렉터리 기준으로 해석됨 → 반드시 절대경로 전달. `PathJoinSubstitution([FindPackageShare('go2_glim_mapping'), 'config', 'glim'])`는 절대경로를 산출하므로 OK.
>
> **덤프 트리거 (소스 확인 `glim_rosnode.cpp`)**: `glim->save(dump_path)`는 노드 **종료(`rclcpp::shutdown`/Ctrl-C)** 시 호출. `dump_path` 파라미터(기본 `/tmp/dump`)로 경로 변경 가능. `dump_on_unload` 파라미터(`glim_ros.cpp:62`)도 존재 — 헤드리스 자동 덤프 옵션으로 PoC에서 검토.

---

## 5. Gazebo Classic 시뮬 구성

### 5.1 Go2 모델

- 베이스: **CHAMP 기반 Go2 URDF** 또는 커뮤니티 레포(`anujjain-dev/unitree-go2-ros2`, Humble+Gazebo Classic+CHAMP — 단 Velodyne만, SLAM 미구현 [F9](#f9)). URDF/제어만 차용하고 LiDAR는 교체.
- ⚠️ Go2+Ouster+GLIM 엔드투엔드 검증 레포는 **존재하지 않음**([부록 B](#부록-b--검증-갭과-리스크)). 통합은 직접 작업.

### 5.2 OS1 프로파일 LiDAR

Gazebo에 네이티브 Ouster 플러그인 없음 → `velodyne_gazebo_plugins`의 `gpu_ray`(또는 `libgazebo_ros_velodyne_gpu_laser`)를 OS1 스펙으로 구성:

| 파라미터 | OS1-64 값 |
|----------|-----------|
| 수평 샘플 | 1024 (또는 512/2048) |
| 수평 FOV | 360° |
| 채널(수직) | 64 |
| 수직 FOV | -22.5° ~ +22.5° (45°) |
| 갱신율 | 10 Hz |
| 최대 거리 | 100 m (config `distance_far_thresh` 정렬) |
| 출력 | `sensor_msgs/PointCloud2` (frame_id: `os1_sensor` 또는 `lidar`) |

### 5.3 IMU

- Gazebo `libgazebo_ros_imu_sensor`, `update_rate: 200`(GLIM은 ~100–400Hz 기대), os1_sensor와 동일 링크 부착.
- 발행: `sensor_msgs/Imu`, REP-145 방향(정지 시 +Z≈+9.81).
- **모든 노드 `use_sim_time:=true`** → LiDAR/IMU 시각 동기 보장(실물 Ouster의 "indeterminant linear system" 류 동기 문제 회피, [부록 B](#부록-b--검증-갭과-리스크)).

---

## 6. 맵 저장 파이프라인 (PLY 후처리, 비침습)

확정 방식 ([F5](#f5), [F6](#f6)):

1. 매핑 중 라이브로 RViz/Iridescence에서 확인.
2. `glim_rosnode` **정상 종료**(GUI 닫기) → `/tmp/dump`에 팩터그래프 덤프 자동 저장. *덤프는 포인트클라우드 파일이 아니라 재최적화용 데이터.*
3. `ros2 run glim_ros offline_viewer` 실행 → `/tmp/dump` 로드 → **File > Save > Export Points** → `map.ply`. *(GUI 수동 1회 — 라이브 서비스/토픽 없음)*
4. `python3 -m go2_glim_mapping.ply_to_pcd map.ply map.pcd` → Open3D로 PCD 변환.
   - ⚠️ Open3D는 이미지에 미포함 → venv에 `uv add open3d` 필요(`link_ros_to_venv.sh` 흐름). 또는 의존성 회피하려면 `pcl_ros`의 PLY→PCD 변환 사용.

> 완전 자동화/헤드리스가 꼭 필요하면(향후): `global_mapping->export_points()`를 `GlimROS` 소멸자에서 호출하는 **소스 패치**가 대안 ([F5](#f5)). 단 최신 master에서 반환형이 `gtsam_points::PointCloud::Ptr`로 바뀜 — 침습적이라 이번 단계 제외. GLIM issue #88(라이브 PCD 저장)은 열린 enhancement이므로 향후 릴리스 재확인.

---

## 7. 구현 순서 (스파이크 우선)

> 어드바이저 권고: **데이터 계약을 패키지 조립보다 먼저 검증**. 아래 1번이 노드 목록을 확정한다.

1. ~~**스파이크 #1 — 데이터 계약**~~ ✅ **완료 (2026-06-10)** — [결과](#11-스파이크-1-결과-검증-완료). 합성 무타임스탬프 PointCloud2+IMU로 검증. **republisher 불필요** 확정. 4번으로.
2. **스파이크 #2 — IMU 정합**: IMU rate(200Hz)·방향(REP-145)·`use_sim_time` 동기가 GLIM odometry를 수렴시키는지 확인.
3. **스파이크 #3 — 뷰어/저장**: Docker에서 `offline_viewer` 빌드 여부(`BUILD_WITH_VIEWER`)·X11 포워딩(`xhost +local:`)·PLY export 동작 확인.
4. **Go2 조립**: 검증된 LiDAR/IMU 매크로(`ouster_os1.urdf.xacro`)를 Go2 URDF에 부착, CHAMP로 보행 → 환경 돌며 매핑.
5. **저장 파이프라인 완성**: `ply_to_pcd.py` + 문서화.
6. **패키지화**: launch 3종 + config 정리 + README.

---

## 11. 스파이크 #1 결과 (검증 완료)

> 2026-06-10, `glim-humble-docker-gpu` 컨테이너(CUDA ON 빌드 성공). (검증 스크립트는 throwaway — 정리 시 제거됨.) Gazebo 없이 합성 퍼블리셔로 데이터 계약만 격리 검증.

**셋업**: 시간필드 **없는** PointCloud2(x,y,z float32만, point_step=12) 10Hz + IMU 200Hz를 `/os_cloud_node/{points,imu}`로 발행. 12×12×4m 박스룸, 센서 등속 0.3m/s 직진. `glim_rosnode` 헤드리스(`libstandard_viewer.so` 제거), `global_shutter_lidar=true`, 기본 GPU IMU odometry.

**결과**:

| 검증 항목 | 결과 |
|-----------|------|
| 무타임스탬프 클라우드 크래시? | ❌ 크래시 없음. `[warning] per-point timestamps are not given!! use pseudo per-point timestamps based on the order of points` → TimeKeeper가 의사 타임스탬프 할당 |
| republisher 필요? | **불필요** 확정 |
| odometry 동작? | ✅ `estimate initial IMU state` → `v_world_imu=vec(0.301,...)` — 실제 0.3m/s 속도 정확히 복원 |
| 맵 빌드·저장? | ✅ SIGINT → `[global] saving to /tmp/dump... saved`. `/tmp/dump/000000/`에 `points_compact.bin`(~91KB)·`covs_compact.bin`·`intensities_compact.bin`·`imu_rate.txt` |

**부수 발견(설계 반영)**:
- ⚠️ **비단조 타임스탬프 → throw (프로덕션 리스크)**: 큰 역방향 시간 점프 → `local_index: -1` → `terminate: IndexedSlidingWindow: index out of range`(graceful 아님). 본 스파이크에선 잔류 퍼블리셔 stale 메시지(85초 역점프)가 원인이었고 클린 재실행 0건. **단 `use_sim_time:=true`는 *실행 중* 단조성만 보장하고 *시뮬 리셋* 시 클럭이 더 낮은 값으로 재시작하면 동일 throw 발생** → **시뮬 리셋/rosbag 루프 시 glim_rosnode 재시작 필요**. 운영 절차로 문서화.
- ⚠️ **startup CDR 경고**: 기동 직후 `sequence size exceeds remaining buffer` ×4 후 정상 복구. 모듈 로드와 첫 메시지 레이스로 추정, 무해. QoS는 GLIM 기본 `sensor_data`(best_effort) — reliable 퍼블리셔 호환.
- ⚠️ **IMU 품질 경고**: `IMU prediction is not good. Possibly T_lidar_imu is not accurate` 반복 — 기본 T_lidar_imu(OS0 오프셋)가 합성 셋업과 불일치 + 단순 IMU 탓. odometry 자체는 정상(trans 오차 0.001m). → 실사용 시 lidar/imu 동일 링크면 `T_lidar_imu=identity`로 설정.

---

## 12. 스파이크 #2 결과 (gait 매핑 — 부분 성공)

> 2026-06-10. anujjain `unitree-go2-ros2`(champ + go2_description, VLP-16 + IMU) 클론·빌드(11패키지) → Gazebo Classic 헤드리스(`gui:=false`, CPU `type=ray` 센서) → glim_rosnode(use_sim_time). (검증 스크립트는 throwaway — 정리 시 제거됨.)

**검증 통과 ✅**:
- Gazebo 헤드리스 기동, **실제 Gazebo Velodyne 클라우드** `/velodyne_points` 9.94Hz + IMU `/imu/data` ~80Hz 발행.
- **GLIM이 실제 Gazebo PointCloud2(intensity+ring 필드) 수용 — extract 에러 0, 크래시 0.** 합성이 아닌 실제 시뮬 메시지 포맷 계약 검증(스파이크 #1의 합성 클라우드보다 강한 증거).
- 헤드리스 엔드투엔드 동작 + 덤프 저장(`/tmp/gait_dump/000000/`).
- 토픽 매핑 확정: `points_topic=/velodyne_points`, `imu_topic=/imu/data`, `T_lidar_imu=[-0.2,0,-0.1177,0,0,0,1]`(velodyne@base[0.2,0,0.1177] vs imu@trunk), velodyne 클라우드 필드 `intensity_field=intensity`, `ring_field=ring`.

**미해결 ❌ — 로봇 보행 안 됨 (GLIM 무관)**:
- 25초 walk(cmd_vel 0.3m/s + yaw 0.15) 했으나 궤적 dx=0.175m·dy=0.159m = **제자리 흔들림**(전진 거의 0, 서브맵 1개). 정상이면 ~7m 이동·다수 서브맵.
- champ 컨트롤러는 살아있고 로봇 안 넘어짐(z=0.116 유지). cmd_vel 487건 정상 발행·수신.
- 의심 원인: `hold_joints already declared` 에러 / 하드웨어 `velocity` 인터페이스 vs champ effort 컨트롤러 불일치 / 발-지면 마찰 슬립. → **Gazebo Go2 locomotion 튜닝 영역**(anujjain 레포는 리서치에서 "SLAM 미구현"으로 표시된 미성숙 베이스).

**결론**: GLIM 통합의 핵심 리스크(실제 시뮬 센서 → GLIM)는 해소. 남은 "gait 궤적 → 쓸만한 맵" 검증은 **로봇이 실제로 걸어야** 가능 → locomotion 튜닝이 선결 과제. 이는 별도 작업(GLIM/패키지 설계와 분리).

---

## 13. 스파이크 #3 결과 (디커플 맵 품질 — ✅ 검증 완료)

> 2026-06-10. Go2 보행 튜닝 우회 결정에 따라, **diff-drive 센서 리그**(cmd_vel로 확실히 주행)에 OS1-ish 32빔 LiDAR + 200Hz IMU를 얹어 모션 하 GLIM 맵 품질을 격리 검증. (검증 sim 모델·러너는 throwaway — 정리 시 제거; sim URDF/world·맵저장 로직은 `go2_glim_mapping` 패키지(`sim/`, `map_saver`)로 이관됨.) 산출물: `docs/glim_map_topdown.png`(맵 56k점 top-down 증거).

**셋업**: 16×16m 방(벽+기둥 4개) + diff-drive 로봇, 중앙 스폰 후 개방 공간 **원형 주행**(r≈2.3m). `/points` OS1-32 9.4Hz, `/imu` 200Hz. glim_rosnode(use_sim_time, `global_shutter_lidar=true`, `T_lidar_imu=[-0.1,0,-0.1]`).

**맵 품질 정량 결과**:

| 지표 | 결과 | 판정 |
|------|------|------|
| 맵 bounding box | x[-7.95,7.95] y[-7.95,7.95] z[-0.22,2.32] | ✅ 방 구조(벽 ±7.9, 천장 2.5, 바닥 0)와 정확히 일치 |
| 벽면 두께(z∈[0.5,2.0] std) | 동 0.0101 / 서 0.0109 / 북 0.0094 / 남 0.0109 m | ✅ **~1cm = LiDAR 노이즈(0.008) 수준**, 드리프트 번짐 없음 |
| 벽 위치(mean) | ±7.900 m | ✅ 정확 |
| 맵 점수 / 서브맵 | 56,313점 / 5 서브맵 | ✅ |
| 궤적 추적 | 매끄러운 원형(회전+병진), tracking loss/crash 0 | ✅ |
| top-down 시각 | 사각형 4벽 선명 + 내부 기둥(cylinder/box) 식별 | ✅ 인식 가능한 방 맵 |

**결론: GLIM이 시뮬 모션 하에서 메트릭 정확(벽 ±7.90m, 두께 1cm)한 고품질 3D 맵을 생성·저장함을 검증.** Go2 보행과 분리해 GLIM/패키지 신뢰성 확보.

**부수 발견(패키지 설계 반영 필수)**:
- 🔑 **노드명 = `glim_ros`** (`Node("glim_ros")`, 실행파일명 glim_rosnode와 다름). 따라서 발행 토픽은 `/glim_ros/map`, `/glim_ros/points_corrected`, `/glim_ros/aligned_points`, `/glim_ros/odom` 등. config_path/use_sim_time/dump_path도 노드 `glim_ros`의 파라미터.
- 🔑 **글로벌 맵(`/glim_ros/map`, RELIABLE+TRANSIENT_LOCAL latched)은 서브맵 finalize 후에만 발행.** 밀폐공간 저속주행은 스캔 overlap이 높아 기본 `max_num_keyframes=15`에 도달 못해 서브맵이 종료 시점까지 finalize 안 됨 → 런 중 맵 토픽 빈 채. 라이브 맵 필요 시 `config_sub_mapping_*.json`의 `max_num_keyframes`↓ / `max_keyframe_overlap`↓ 튜닝. (덤프 `/tmp/dump`는 항상 정상 저장.)
- 디커플 리그는 OS1-32빔 CPU `type=ray`로 헤드리스 9.4Hz 안정. OS1-64로 격상 가능(CPU 부하↑).

---

## 8. 참고 레포 / 소스 (인용)

| 레포 / 소스 | 용도 | 비고 |
|-------------|------|------|
| [koide3/glim](https://github.com/koide3/glim) · [docs](https://koide3.github.io/glim/) | SLAM 코어, config, 맵 저장 | 1차. OS1 공식 지원, CUDA13/GTSAM4.3a0 ([F1](#f1)) |
| [koide3/glim_ros2](https://github.com/koide3/glim_ros2) | ROS2 노드(`glim_rosnode` 등) | 1차. 런치 대상 ([F4](#f4)) |
| [koide3/gtsam_points](https://github.com/koide3/gtsam_points) | 의존성, `BUILD_WITH_CUDA` | CUDA OFF 가능 ([F2](#f2)) |
| [glim issue #88](https://github.com/koide3/glim/issues/88) | PCD 저장 방법 | 라이브 저장 없음 확인 ([F5](#f5)) |
| [anujjain-dev/unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2) | Go2 Gazebo Classic+CHAMP URDF | Velodyne만, SLAM 미구현 ([F9](#f9)) |
| [ouster-lidar/ouster-ros](https://github.com/ouster-lidar/ouster-ros) | 실물 OS1 드라이버 토픽 계약 | `/ouster/*` 발행 → 리맵 참고 |
| [jizhang-cmu/autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2) | Go2 자율 스택(참고) | Unity+L1+Point-LIO. Gazebo/Ouster 아님 — LiDAR-IMU SLAM 참고만 |
| [markhedleyjones GLIM-Ouster T_lidar_imu](https://markhedleyjones.com/notes/glim-ouster-t-lidar-imu-transform) | 실물 OS1 extrinsic 계산 | T_lidar_imu 참고(blog) |
| (대안) [abizovnuralem/go2_omniverse](https://github.com/abizovnuralem/go2_omniverse) · [ChristophKin/master_kin](https://github.com/ChristophKin/master_kin) | Isaac Sim Go2+OS1 | Gazebo 대신 Isaac 갈 경우만 ([F8-sim](#fsim)) |

---

## 부록 A — 리서치 근거(인용)

> deep-research: 18개 1차 소스, 87개 주장 추출, 25개 적대적 검증(3표제, 24 확정/1 기각). 핵심 코드 사실은 본 레포 서브모듈(`src/glim` @ v1.2.1, `src/glim_ros2` @ a62811d)에서 직접 확인.

<a id="f1"></a>**F1 — GLIM OS1/CUDA13 지원 (확정 3-0)**: 공식 문서에 'Spinning-type LiDAR (Ouster OS1)', 'Tested on Ubuntu 22.04/24.04 with CUDA 12.2/12.6/13.1'. v1.2.0(2026-01-24) GTSAM 4.2a9/4.3a0 + CUDA 13.1 지원 → 현 Docker 타깃 유효. *koide3.github.io/glim, github.com/koide3/glim*

<a id="f2"></a>**F2 — 필수 의존성 / CUDA OFF (확정 3-0)**: 필수=Eigen·nanoflann·GTSAM·gtsam_points. CUDA·OpenCV·OpenMP·ROS·Iridescence는 선택. `gtsam_points -DBUILD_WITH_CUDA=OFF`로 CPU 동작 가능(GLIM 자동 CPU 폴백). 단 issue #96: 비CUDA 빌드 후 CMake가 CUDA 참조하면 nvcc 에러. *github.com/koide3/glim, /gtsam_points*

<a id="f3"></a>**F3 — 입력 토픽 계약 (확정 3-0)**: `config_ros.json` 단수형 키 `imu_topic`(`/os_cloud_node/imu`), `points_topic`(`/os_cloud_node/points`). `glim_ros.cpp`에서 `sensor_msgs::Imu`/`PointCloud2` 구독 생성. 패키지명 `glim_ros` v1.2.1 ament_cmake. 복수형 키는 없음(기각). 실물 ouster-ros는 `/ouster/*` 발행 → 리맵 필요. *quickstart.html, glim_ros2, config_ros.json*

<a id="f4"></a>**F4 — extrinsic & 실행파일 (확정 3-0)**: `T_lidar_imu`(config_sensors.json) = IMU→LiDAR, TUM SE3, REP-145. glim_ros2는 5개 실행파일: `glim_rosnode`(라이브)·`glim_rosbag`(오프라인)·`validator_node` + `BUILD_WITH_VIEWER` 시 `offline_viewer`·`map_editor`. *parameters.md, config_sensors.json, glim_ros2 CMakeLists*

<a id="f5"></a>**F5 — 맵 저장 (확정 3-0)**: GLIM 라이브 PCD 저장 네이티브 없음. PLY는 `offline_viewer` GUI(File>Save>Export Points)만. PCD는 (a) PLY→Open3D 변환 또는 (b) 소스 패치 `global_mapping->export_points()`. 메인테이너 koide3 확인(issue #88). 반환형 최신 master에서 `gtsam_points::PointCloud::Ptr`로 변경(버전 의존). PLY→PCD 시 intensity/time 손실 가능. *glim issue #88, quickstart.html*

<a id="f6"></a>**F6 — /tmp/dump (확정 3-0)**: `glim_rosnode`/`glim_rosbag` 정상 종료 시 `/tmp/dump`에 팩터그래프 덤프 자동 저장. 이는 offline_viewer가 로드/재최적화하는 데이터이지 포인트클라우드 맵 파일이 아님. *quickstart.html, issue #172*

<a id="f7"></a>**F7 — global_shutter_lidar (코드 직접 확인)**: `config_sensors.json:52` `global_shutter_lidar: false`(기본). 주석: "If true, fill per-point timestamps with zero and disable points deskewing". `cloud_preprocessor.cpp:26`에서 읽고, 라인 138 `std::fill(frame->times, ... , 0.0)`. 무타임스탬프 sim LiDAR의 공식 처리 경로. *src/glim/config/config_sensors.json, src/glim/src/glim/preprocess/cloud_preprocessor.cpp*

<a id="f8"></a>**F8 — times 처리 / odometry 모드 (코드 직접 확인)**: `extract_raw_points`(ros_cloud_converter.hpp)는 `t`/`time`/`time_stamp`/`timestamp` 필드 인식, 없으면 times 빈 채. `cloud_deskewing.cpp:17,63` 두 deskew 오버로드 모두 `if(times.empty()) return {}`. **IMU odometry는 빈 deskew 허용**. 반면 `odometry_estimation_ct.cpp:127,134,142`는 `frame->times[i]` 직접 인덱싱 → CT 모드는 빈 times면 위험 → 회피. ⚠️ `cloud_preprocessor.cpp:135` sort가 fill보다 먼저 times 접근 → PoC 경험적 확인 필요. *src/glim 소스 직접 확인*

<a id="f9"></a>**F9 — Gazebo 대안 평가 (확정 3-0)**: `anujjain-dev/unitree-go2-ros2` = Go2 Gazebo Classic+CHAMP, Humble, 그러나 Velodyne 3D LiDAR(Ouster 아님) + SLAM 미구현, Gazebo Classic EOL(2025-01). `go2_ros2_sdk` = 실물 전용, 내장 L1, slam_toolbox 2D(GLIM 아님). *github.com/anujjain-dev/unitree-go2-ros2, /abizovnuralem/go2_ros2_sdk*

<a id="fsim"></a>**F-sim — Isaac Sim 대안 (확정 3-0)**: Isaac Sim이 Go2+실제 OS1 프로파일 + ROS2 Humble PointCloud2+IMU 발행 유일 네이티브 매치(`master_kin`은 OS1 config 명시, `go2_omniverse`는 RTX LiDAR L1 기본). 단 NVIDIA GPU 필수, Isaac Sim 2023.1.1 구버전 핀. *github.com/ChristophKin/master_kin, /abizovnuralem/go2_omniverse*

### 기각된 주장
- ❌ (0-3) "config_ros.json이 `imu_topics`/`points_topics`/`image_topics` 복수형 파라미터를 쓴다" — 실제는 단수형. *parameters.md*

---

## 부록 B — 검증 갭과 리스크

1. **엔드투엔드 미검증**: Go2+Ouster+GLIM 통합 레포 부재. 모든 시뮬 소스가 GLIM을 다루지 않음 → 토픽↔GLIM 배선(필드 레이아웃, IMU rate, 시간동기)은 직접 엔지니어링. Isaac 대안 레포들도 소규모 개인 프로젝트.
2. ~~**빈 times sort 크래시 가능성**~~ ✅ **해결**: 스파이크 #1에서 TimeKeeper가 의사 타임스탬프 자동 할당 확인 → 크래시 없음, republisher 불필요 ([§11](#11-스파이크-1-결과-검증-완료)). 단 비단조 타임스탬프는 throw하므로 `use_sim_time` 필수.
3. **실물 OS1 동기 이슈**: GLIM issue #197에서 실물 OS1 IMU 시간동기/"indeterminant linear system" 보고 — 실물 전환 시 extrinsic·time-sync 튜닝 필요(시뮬은 `use_sim_time`로 회피).
4. **레거시 vs 신형 Ouster 토픽**: 기본 config는 레거시 `/os_cloud_node/*`. 신형 ouster-ros는 `/ouster/*` → 리맵 필수(시뮬·실물 공통).
5. **시간 민감성**: GLIM 빠르게 변화(v1.2.0 2026-01-24). export_points() 시그니처 변경됨. 라이브 PCD 저장 서비스가 향후 릴리스에 추가될 수 있음(issue #88) → 재확인.
6. **패키지 레이아웃**: §4는 1차 소스 직접 인용이 아니라 입력 계약(F3·F4)에서 합성한 권장안.
