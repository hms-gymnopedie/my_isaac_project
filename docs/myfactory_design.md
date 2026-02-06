# 공장 환경 설계 (260202_my_factory_withCameras.usda)

이 문서는 Blender에서 제작한 **단층 공장 환경**의 **레이아웃**, **USD 파일 구조**, **재질 및 CCTV 카메라**를 정의합니다.

## 1. 좌표계 및 스테이지 기본 설정

| 항목 | 값 | 설명 |
|------|-----|------|
| `metersPerUnit` | 1 | 1 단위 = 1미터 |
| `upAxis` | Z | Z축이 위쪽 |
| `defaultPrim` | root | 기본 루트 프림 |

- **제작 도구**: Blender v4.5.4 LTS
- 월드 원점 `(0, 0, 0)`: **남서쪽 모서리** 기준
- **공장 범위**: X: [-3, 48], Y: [-3, 45], Z: [0, 5]
- **바닥**: z = 0
- **벽 높이**: 5m

## 2. USD 파일 구조

- **파일 경로**: `assets/environments/260202_my_factory_withCameras.usda`
- **루트 구조**:

```text
/root
  /Looks                    # 재질 정의
    /Floor_Concrete         # 바닥 콘크리트
    /Wall_Concrete          # 벽 콘크리트
    /Metal_Equipment         # 금속 설비
    /Motor_Blue             # 모터 페인트
  /Camera                   # Blender 메인 카메라
  /Motor_base_1             # 모터 유닛 1
    /Cube_002               # 베이스 메쉬
    /NAUO42/MH_22_01_05_*   # 모터 상세
  /Motor_base_1_001         # 모터 유닛 2
  /Motor_base_1_002         # 모터 유닛 3
  /Motor_base_1_003         # 모터 유닛 4
  /wall                     # 벽 (Plane 메쉬)
  /MH_22_01_00M             # 추가 설비
    /NAUO37
    /NAUO20
  /bottom                   # 바닥 (Plane_001 메쉬)
  /Light                    # SphereLight
  /env_light                # DomeLight (HDR)
  /CCTV_Cameras             # CCTV 카메라 5대
    /Camera_01
    /Camera_02
    /Camera_03
    /Camera_04
    /Camera_05
  /GroundPlane              # 지면 (CollisionMesh)
```

## 3. 환경 구조물

### 3.1 바닥 (bottom)

| 항목 | 값 |
|------|-----|
| 경로 | `/root/bottom` |
| 메쉬 | Plane_001 |
| Extent | (-3, -3, 0) ~ (48, 45, 0) |
| 재질 | Floor_Concrete |

- **바닥 면적**: X 51m × Y 48m

### 3.2 벽 (wall)

| 항목 | 값 |
|------|-----|
| 경로 | `/root/wall` |
| 메쉬 | Plane |
| Extent | (-3, -3, 0) ~ (48, 45, 5) |
| 재질 | Wall_Concrete |
| 높이 | 5m |

### 3.3 모터 설비 (Motor_base_*)

| 유닛 | 경로 | 베이스 Extent (대략) | 비고 |
|------|------|---------------------|------|
| Motor_base_1 | `/root/Motor_base_1` | (32, 38.5, 0) ~ (34, 41.5, 0.5) | NAUO42 |
| Motor_base_1_001 | `/root/Motor_base_1_001` | (37, 38.5, 0) ~ (39, 41.5, 0.5) | NAUO42_001 |
| Motor_base_1_002 | `/root/Motor_base_1_002` | (37, 38.5, 0) ~ (39, 41.5, 0.5) | NAUO42_002 |
| Motor_base_1_003 | `/root/Motor_base_1_003` | (42, 38.5, 0) ~ (44, 41.5, 0.5) | NAUO42_003 |

- **재질**: Metal_Equipment (베이스), Motor_Blue (모터 본체)
- **배치**: 공장 내부 여러 위치

### 3.4 GroundPlane

| 항목 | 값 |
|------|-----|
| 경로 | `/root/GroundPlane` |
| Scale | (2.4, 2.2, 1) |
| 용도 | 지면 / CollisionMesh |

## 4. 재질 (Looks)

### 4.1 재질 목록

| 재질명 | 경로 | 용도 |
|--------|------|------|
| Floor_Concrete | `/root/Looks/Floor_Concrete` | 바닥 |
| Wall_Concrete | `/root/Looks/Wall_Concrete` | 벽 |
| Metal_Equipment | `/root/Looks/Metal_Equipment` | 설비 베이스 |
| Motor_Blue | `/root/Looks/Motor_Blue` | 모터 본체 |

- **쉐이더**: UsdPreviewSurface (PBR)
- **텍스처**: 단색 또는 프로젝트 텍스처 경로 사용 가능

## 5. 조명 (Lighting)

| 조명 | 경로 | 타입 | 설정 |
|------|------|------|------|
| Light | `/root/Light` | SphereLight | intensity 318.31, normalize=1, radius=0.1 |
| env_light | `/root/env_light` | DomeLight | intensity 1000, HDR `./textures/color_121212.hdr` |

- **DomeLight 회전**: (90, 0, 90)°

## 6. CCTV 카메라 (CCTV_Cameras)

| 카메라 | 경로 | 위치 (X, Y, Z) | 용도 |
|--------|------|----------------|------|
| Camera_01 | `/root/CCTV_Cameras/Camera_01` | (-2.1, 19.9, 4.8) | 입구/서쪽 구역 |
| Camera_02 | `/root/CCTV_Cameras/Camera_02` | (20.4, 15.3, 4.5) | 중앙 구역 |
| Camera_03 | `/root/CCTV_Cameras/Camera_03` | (15.6, 29.4, 4.8) | 북쪽 구역 |
| Camera_04 | `/root/CCTV_Cameras/Camera_04` | (45.8, 24.3, 4.8) | 동쪽 구역 |
| Camera_05 | `/root/CCTV_Cameras/Camera_05` | (47.5, 44.0, 4.8) | 북동 모서리 (기본 뷰) |

- **높이**: 약 4.5~4.8m
- **Transform**: xformOp:translate + xformOp:orient (quatd)
- **clippingRange**: (0.01, 10000000)
- **focusDistance**: 400
- **boundCamera**: 기본 뷰는 Camera_05 사용

## 7. 폴더 구조

```text
assets/
├── environments/
│   ├── 260202_my_factory_withCameras.usda
│   └── textures/
│       └── color_121212.hdr    # DomeLight HDR
└── robots/
    └── Unitree_go2.usd
```

- **authoring_layer**: `./260202_my_factory_withCameras.usd` (편집 시 참조)

## 8. 공간 좌표 요약

```text
      Y (45m)
      ^
      |                    Camera_05 (47.5, 44)
   45 +-----------------------------------+
      |                           Camera_04
      |                         (45.8, 24.3)
      |         Camera_03
      |       (15.6, 29.4)     Motors
      |                 Camera_02
      |               (20.4, 15.3)
      |  Camera_01
      | (-2.1, 19.9)
   -3 +-----------------------------------+-> X (48m)
     -3                                  48
```

## 9. 관련 파일

| 파일 | 설명 |
|------|------|
| `myfactory_ing.usda` | 2층 공장 (코드로 작성, 층고 5m, 계단참 포함) |
| `260130_my_factory.usda` | Blender 1차 내보내기 (재질/물리/CCTV 없음) |
| `260202_my_factory_withCameras.usda` | Blender 공장 + 재질 + CCTV 5대 (본 명세 대상) |

## 10. 향후 확장 계획

- [ ] 2층 구조 추가 (선택)
- [ ] 벽/바닥 PBR 텍스처 적용
- [ ] PhysicsCollisionAPI 일괄 적용 검토
- [ ] 시맨틱 레이블 (SemanticsAPI) 추가
- [ ] 로봇 스폰 앵커 및 Zone 정의
- [ ] 추가 조명 (Rect Light 등) 배치
