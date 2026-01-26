"""
Isaac Sim + Pegasus Simulator Project Scripts
=============================================

산업현장 드론 및 4족로봇 시뮬레이션 프로젝트

Modules:
    - control: 로봇 제어 스크립트 (드론, 4족로봇)
    - slam: SLAM 및 자율주행 관련 스크립트
    - setup: 환경 설정 및 초기화 스크립트
    - utils: 유틸리티 함수
"""

__version__ = "0.1.0"
__author__ = "minsuh"

from . import control
from . import slam
from . import setup
from . import utils
