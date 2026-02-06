# 환경 모델링 설계 (Phase 2.1)

이 문서는 산업현장 기본 환경의 **레이아웃**과 **USD 파일 구조**를 정의합니다.

## 1. 좌표계 및 스테이지 기본 설정

- 단위: `metersPerUnit = 1`
- Up axis: `Z`
- 월드 원점 `(0, 0, 0)`:
  - 바닥 중앙이 아니라, **서쪽-남쪽 모서리 (x=0, y=-15, z=0)**를 기준으로 설계
  - 바닥은 x: \[0, 50], y: \[-15, 15], z: 0에 위치

## 2. USD 파일 구조

- 파일 경로: `assets/environments/industrial_factory.usda`
- 루트 구조:

```text
/World
  /Environment
    /Floor
    /Walls
    /Zones
    /StairsAnchor
    /EquipmentAnchors
    /Materials
```

### 2.1 Floor

- 경로: `/World/Environment/Floor/floor_mesh`
- 크기: `50m x 30m x 0.1m`
- 위치: `(25, 0, -0.05)` → z=0 평면에 상단이 오도록 오프셋
- 재질: `Materials/Concrete`에 바인딩

### 2.2 Walls

- 경로: `/World/Environment/Walls/*`
- 4면 벽:
  - `wall_north`: y=+15
  - `wall_south`: y=-15
  - `wall_east`: x=50
  - `wall_west`: x=0
- 높이: 4m

### 2.3 Zones (작업 구역)

- 경로: `/World/Environment/Zones/*`
- 목적: **작업 구역**을 논리적으로 나누기 위한 Xform
- 각 Zone에는 `customData.zoneType` 메타데이터를 부여:
  - `/Zones/Zone_Loading` → `zoneType = "loading"`
  - `/Zones/Zone_Processing` → `zoneType = "processing"`
  - `/Zones/Zone_Storage` → `zoneType = "storage"`
- 추후 `configs/environment.yaml`와 매핑하여:
  - 장애물 스폰 영역
  - 로봇 출발 위치
  - 설비 배치 영역
  를 정의할 예정

### 2.4 StairsAnchor (계단 기준점)

- 경로: `/World/Environment/StairsAnchor`
- 역할:
  - `configs/environment.yaml`의 `stairs.position`과 **1:1 매핑**
  - Phase 2.3에서 실제 계단 USD/프리미티브를 여기에 인스턴싱 또는 생성
- 현재 값:
  - `xformOp:translate = (20, 0, 0)`

### 2.5 EquipmentAnchors (설비/QR 기준점)

- 경로: `/World/Environment/EquipmentAnchors/*`
- 역할:
  - `configs/environment.yaml`의 `equipment` 리스트와 위치를 맞추는 기준점
  - 각 설비에 대응하는 앵커:
    - `pump_station_1_anchor` ↔ `equipment[0]` (PUMP001)
    - `control_panel_1_anchor` ↔ `equipment[1]` (PANEL001)
    - `valve_1_anchor` ↔ `equipment[2]` (VALVE001)
- 좌표는 `environment.yaml`의 `position` 값과 동일하게 설정

### 2.6 Materials

- 경로: `/World/Environment/Materials/*`
- 현재는 **placeholder Material**만 존재:
  - `Materials/Concrete`
  - 실제 OmniPBR 머티리얼은 Isaac Sim 에디터 또는 스크립트에서 설정 예정

## 3. 환경 설정 파일과의 매핑

- 설정 파일: `configs/environment.yaml`
- 주요 매핑:
  - `environment.dimensions` ↔ Floor 및 Walls 크기
  - `stairs.position` ↔ `/World/Environment/StairsAnchor` 위치
  - `equipment[*].position` ↔ `/World/Environment/EquipmentAnchors/*_anchor` 위치

## 4. 다음 단계에서 할 일 (Phase 2.2+)

- 장애물(Static + Random)을 USD 레벨에서 표현
- 계단 모델을 별도 USD로 분리 후 `StairsAnchor`에 인스턴싱
- 조명(Lighting) 프리미티브 추가:
  - Dome Light (주 조명)
  - Rect Light (작업 구역 위)
- Domain Randomization을 위한 Prim 구조 및 Metadata 확장

