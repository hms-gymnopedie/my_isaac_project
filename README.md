# Isaac Sim + Pegasus Simulator Project

산업현장 드론 및 4족로봇 시뮬레이션 프로젝트

## 📁 프로젝트 구조

```
my_isaac_project/
├── scripts/                    # Python 제어 스크립트
│   ├── control/               # 로봇 제어 (드론, 4족로봇)
│   ├── slam/                  # SLAM 및 자율주행
│   ├── setup/                 # 환경 설정 스크립트
│   └── utils/                 # 유틸리티 함수
├── assets/                    # 로봇 모델 및 환경 에셋
│   ├── robots/               # URDF/USD 로봇 파일
│   ├── environments/         # 환경 USD 파일
│   └── textures/             # 텍스처 파일
├── configs/                   # 설정 파일 (YAML)
│   ├── simulation.yaml       # 시뮬레이션 설정
│   ├── drone.yaml            # 드론 설정
│   ├── quadruped.yaml        # 4족로봇 설정
│   └── environment.yaml      # 환경 설정
├── data/                      # 수집 데이터
│   ├── logs/                 # 로그 파일
│   ├── collected/            # 수집된 센서 데이터
│   └── benchmarks/           # 벤치마크 결과
├── tests/                     # 테스트 코드
└── docs/                      # 문서
```

## 🚀 시작하기

### 1. Docker 환경 실행

```bash
docker run --gpus all -it --network=host \
    --name isaac-pegasus-headless \
    -v ~/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v /scratch/minsuh/my_isaac_project:/workspace:rw \
    isaac-sim-5.1-pegasus:optimized \
    -c "cd /isaacsim && ./isaac-sim.sh --enable omni.kit.livestream.webrtc --/app/livestream/enabled=true --no-window"
```

### 2. 설정 파일 로드

```python
from scripts.utils.config_loader import load_config

# 시뮬레이션 설정 로드
sim_config = load_config("/workspace/configs/simulation.yaml")

# 드론 설정 로드
drone_config = load_config("/workspace/configs/drone.yaml")
```

### 3. WebRTC 스트리밍 접속

브라우저에서 접속:
```
http://<서버IP>:8211/streaming/webrtc-client
```

## 📋 개발 로드맵

- [x] Phase 1: 환경 구축 및 기반 설정
- [ ] Phase 2: 환경 모델링 (산업현장, 계단)
- [ ] Phase 3: 로봇 URDF 임포트 (드론, 4족로봇)
- [ ] Phase 4: 물리 설정 및 기본 제어
- [ ] Phase 5: SLAM 구현
- [ ] Phase 6: QR Tag 감지 및 데이터 수집
- [ ] Phase 7: 성능 지표 측정
- [ ] Phase 8: Anomaly Detection
- [ ] Phase 9: ROS2 통합
- [ ] Phase 10: Domain Randomization
- [ ] Phase 11: 통합 테스트 및 최적화

## 🔧 개발 환경

- **Isaac Sim**: 5.1.0
- **Pegasus Simulator**: 1.14.3
- **PX4-Autopilot**: v1.14.3
- **Ubuntu**: 22.04 LTS
- **CUDA**: 12.5
- **Python**: 3.10+

## 📞 문의

프로젝트 관련 문의사항은 Issues에 등록해주세요.
