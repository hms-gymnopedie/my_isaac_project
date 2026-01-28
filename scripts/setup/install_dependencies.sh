#!/bin/bash
# =============================================================================
# 의존성 설치 스크립트
# =============================================================================
# 
# 이 스크립트는 Docker 컨테이너 내부에서 실행됩니다.
# Isaac Sim Python 환경에 필요한 추가 패키지를 설치합니다.
#
# 사용법:
#   docker exec -it isaac-pegasus-headless bash /workspace/scripts/setup/install_dependencies.sh
#
# =============================================================================

set -e

echo "=============================================="
echo "Isaac Sim Project - 의존성 설치 시작"
echo "=============================================="

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 경로 설정
ISAACSIM_PATH="${ISAACSIM_PATH:-/isaacsim}"
ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
WORKSPACE="/workspace"
REQUIREMENTS_FILE="${WORKSPACE}/requirements.txt"

# Isaac Sim Python 확인
if [ ! -x "$ISAACSIM_PYTHON" ]; then
    echo -e "${RED}❌ Isaac Sim Python을 찾을 수 없습니다: $ISAACSIM_PYTHON${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Isaac Sim Python: $ISAACSIM_PYTHON${NC}"

# -----------------------------------------------------------------------------
# 1. 시스템 패키지 설치 (pyzbar 의존성)
# -----------------------------------------------------------------------------
echo ""
echo -e "${YELLOW}[1/4] 시스템 패키지 설치 중...${NC}"

# root 권한 확인
if [ "$EUID" -eq 0 ]; then
    apt-get update -qq
    apt-get install -y --no-install-recommends \
        libzbar0 \
        libzbar-dev \
        && rm -rf /var/lib/apt/lists/*
    echo -e "${GREEN}✅ 시스템 패키지 설치 완료${NC}"
else
    echo -e "${YELLOW}⚠️ root 권한이 없습니다. libzbar가 이미 설치되어 있어야 합니다.${NC}"
fi

# -----------------------------------------------------------------------------
# 2. pip 업그레이드
# -----------------------------------------------------------------------------
echo ""
echo -e "${YELLOW}[2/4] pip 업그레이드 중...${NC}"
"$ISAACSIM_PYTHON" -m pip install --upgrade pip --quiet
echo -e "${GREEN}✅ pip 업그레이드 완료${NC}"

# -----------------------------------------------------------------------------
# 3. requirements.txt 설치
# -----------------------------------------------------------------------------
echo ""
echo -e "${YELLOW}[3/4] Python 패키지 설치 중...${NC}"

if [ -f "$REQUIREMENTS_FILE" ]; then
    "$ISAACSIM_PYTHON" -m pip install -r "$REQUIREMENTS_FILE" --quiet
    echo -e "${GREEN}✅ Python 패키지 설치 완료${NC}"
else
    echo -e "${RED}❌ requirements.txt를 찾을 수 없습니다: $REQUIREMENTS_FILE${NC}"
    exit 1
fi

# -----------------------------------------------------------------------------
# 4. 설치 확인
# -----------------------------------------------------------------------------
echo ""
echo -e "${YELLOW}[4/4] 설치 확인 중...${NC}"

# 핵심 패키지 import 테스트
"$ISAACSIM_PYTHON" << 'EOF'
import sys
print(f"Python: {sys.version}")

packages = [
    ("numpy", "numpy"),
    ("scipy", "scipy"),
    ("pandas", "pandas"),
    ("sklearn", "scikit-learn"),
    ("yaml", "PyYAML"),
    ("cv2", "opencv"),
    ("tqdm", "tqdm"),
]

print("\n패키지 버전:")
print("-" * 40)
for module_name, display_name in packages:
    try:
        module = __import__(module_name)
        version = getattr(module, "__version__", "unknown")
        print(f"✅ {display_name}: {version}")
    except ImportError as e:
        print(f"❌ {display_name}: 설치 안됨 ({e})")

# pyzbar 특별 처리 (libzbar 필요)
try:
    from pyzbar import pyzbar
    print("✅ pyzbar: 설치됨")
except ImportError as e:
    print(f"⚠️ pyzbar: {e}")
    print("   (QR 코드 기능 없이 진행 가능)")

print("-" * 40)
EOF

echo ""
echo "=============================================="
echo -e "${GREEN}✅ 의존성 설치 완료!${NC}"
echo "=============================================="
