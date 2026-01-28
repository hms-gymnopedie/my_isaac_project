# =============================================================================
# Makefile - 개발 도구 명령어 모음
# =============================================================================
#
# 사용법:
#   make help       - 도움말 표시
#   make format     - 코드 포맷팅
#   make lint       - 코드 검사
#   make test       - 테스트 실행
#
# =============================================================================

.PHONY: help format lint test clean install

# 기본 타겟
.DEFAULT_GOAL := help

# Python 경로 (Isaac Sim 환경 또는 일반)
PYTHON ?= python3
ISAACSIM_PYTHON ?= /isaacsim/python.sh

# 소스 디렉토리
SRC_DIR := scripts
TEST_DIR := tests

# =============================================================================
# 도움말
# =============================================================================

help:
	@echo "=============================================="
	@echo " Isaac Sim Project - 개발 도구"
	@echo "=============================================="
	@echo ""
	@echo "포맷팅:"
	@echo "  make format      - black + isort로 코드 포맷팅"
	@echo "  make format-check - 포맷팅 검사만 (수정 안함)"
	@echo ""
	@echo "코드 검사:"
	@echo "  make lint        - flake8 + pylint 실행"
	@echo "  make flake8      - flake8만 실행"
	@echo "  make pylint      - pylint만 실행"
	@echo "  make mypy        - 타입 검사"
	@echo ""
	@echo "테스트:"
	@echo "  make test        - pytest 실행"
	@echo "  make test-cov    - 커버리지 포함 테스트"
	@echo ""
	@echo "기타:"
	@echo "  make clean       - 캐시 파일 정리"
	@echo "  make install     - 개발 의존성 설치"
	@echo "  make pre-commit  - pre-commit 훅 설치"
	@echo ""

# =============================================================================
# 포맷팅
# =============================================================================

format:
	@echo "🎨 코드 포맷팅 중..."
	$(PYTHON) -m black $(SRC_DIR) $(TEST_DIR)
	$(PYTHON) -m isort $(SRC_DIR) $(TEST_DIR)
	@echo "✅ 포맷팅 완료"

format-check:
	@echo "🔍 포맷팅 검사 중..."
	$(PYTHON) -m black --check --diff $(SRC_DIR) $(TEST_DIR)
	$(PYTHON) -m isort --check-only --diff $(SRC_DIR) $(TEST_DIR)

# =============================================================================
# 코드 검사 (Linting)
# =============================================================================

lint: flake8 pylint
	@echo "✅ 모든 검사 완료"

flake8:
	@echo "🔍 Flake8 검사 중..."
	$(PYTHON) -m flake8 $(SRC_DIR)

pylint:
	@echo "🔍 Pylint 검사 중..."
	$(PYTHON) -m pylint $(SRC_DIR) --rcfile=.pylintrc || true

mypy:
	@echo "🔍 타입 검사 중..."
	$(PYTHON) -m mypy $(SRC_DIR) --config-file mypy.ini

# =============================================================================
# 테스트
# =============================================================================

test:
	@echo "🧪 테스트 실행 중..."
	$(PYTHON) -m pytest $(TEST_DIR) -v

test-cov:
	@echo "🧪 커버리지 테스트 실행 중..."
	$(PYTHON) -m pytest $(TEST_DIR) -v --cov=$(SRC_DIR) --cov-report=html --cov-report=term

# =============================================================================
# 유틸리티
# =============================================================================

clean:
	@echo "🧹 캐시 정리 중..."
	find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name ".pytest_cache" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name ".mypy_cache" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name "htmlcov" -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete 2>/dev/null || true
	find . -type f -name ".coverage" -delete 2>/dev/null || true
	@echo "✅ 정리 완료"

install:
	@echo "📦 개발 의존성 설치 중..."
	$(PYTHON) -m pip install black isort flake8 pylint mypy pytest pytest-cov pre-commit
	@echo "✅ 설치 완료"

pre-commit:
	@echo "🔗 Pre-commit 훅 설치 중..."
	$(PYTHON) -m pre_commit install
	@echo "✅ Pre-commit 훅 설치 완료"

# =============================================================================
# Isaac Sim 환경 전용
# =============================================================================

isaacsim-install:
	@echo "📦 Isaac Sim 환경에 개발 도구 설치 중..."
	$(ISAACSIM_PYTHON) -m pip install black isort flake8 pylint mypy pytest pytest-cov
	@echo "✅ 설치 완료"

isaacsim-test:
	@echo "🧪 Isaac Sim 환경에서 테스트 실행 중..."
	$(ISAACSIM_PYTHON) -m pytest $(TEST_DIR) -v

isaacsim-verify:
	@echo "🔍 Isaac Sim 환경 검증 중..."
	$(ISAACSIM_PYTHON) $(SRC_DIR)/setup/verify_environment.py
