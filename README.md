# 학습 기반 플래너 안내

이 저장소는 F1TENTH 자율주행 차량을 위한 PyTorch 기반 학습 플래너 예제를 제공합니다.
기존 README에 있던 규칙 기반(라티스 플래너, Follow-The-Gap 등) 내용은 제거되었으며,
학습 기반 플래너 실행에 필요한 절차만을 정리했습니다.

## 준비

```bash
# 저장소 클론 (서브모듈 포함)
git clone --recursive <repository-url>
cd learning_all_js
```

## 빌드

```bash
colcon build
source install/setup.bash
```

## 실행 순서

### 1. PolarGrid 생성 노드
전역 경로를 PolarGrid 형태로 변환합니다.

```bash
ros2 run global_to_polar_cpp global_to_polar_node \
  --ros-args -p path_csv_file:=<전역경로 CSV>
```

### 2. 학습 플래너 노드
사전 학습된 PyTorch 모델을 이용해 로컬 경로를 생성합니다.

```bash
ros2 run pytorch_planner_pkg planner_node \\
  --ros-args --params-file src/pytorch_planner_pkg/params.yaml
```

`params.yaml` 파일의 `model_path` 항목을 수정하면 다른 학습 모델을 사용할 수 있습니다.

노드는 `/scan`과 `/polar_grid` 토픽을 구독하고,
`/planned_path_with_velocity` 토픽으로 10개의 웨이포인트를 발행합니다.

## 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | LIDAR 스캔 데이터 |
| `/polar_grid` | `global_to_polar_cpp/PolarGrid` | 전역 경로를 PolarGrid로 표현한 데이터 |
| `/planned_path_with_velocity` | `f1tenth_planning_custom_msgs/PathWithVelocity` | 학습 플래너가 생성한 경로 및 속도 |

## 모델

사전 학습된 모델 파일은 `src/pytorch_planner_pkg/model` 폴더에 포함되어 있으며,
다른 모델로 교체하거나 재학습한 모델을 사용하려면 `model_path` 파라미터를 수정하면 됩니다.
