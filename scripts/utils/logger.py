"""
Logging Utility
===============

프로젝트 전용 로깅 유틸리티
"""

import logging
import sys
from pathlib import Path
from datetime import datetime
from typing import Optional


def setup_logger(
    name: str = "isaac_project",
    level: int = logging.INFO,
    log_dir: Optional[str] = None,
    console: bool = True
) -> logging.Logger:
    """
    로거를 설정합니다.
    
    Args:
        name: 로거 이름
        level: 로깅 레벨
        log_dir: 로그 파일 저장 디렉토리 (None이면 파일 저장 안함)
        console: 콘솔 출력 여부
        
    Returns:
        설정된 로거
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # 기존 핸들러 제거
    logger.handlers.clear()
    
    # 포맷터 설정
    formatter = logging.Formatter(
        '[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # 콘솔 핸들러
    if console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)
    
    # 파일 핸들러
    if log_dir:
        log_path = Path(log_dir)
        log_path.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = log_path / f"{name}_{timestamp}.log"
        
        file_handler = logging.FileHandler(log_file, encoding='utf-8')
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        
        logger.info(f"Log file: {log_file}")
    
    return logger


# 기본 로거 인스턴스
_default_logger: Optional[logging.Logger] = None


def get_logger(name: Optional[str] = None) -> logging.Logger:
    """
    로거를 가져옵니다.
    
    Args:
        name: 로거 이름 (None이면 기본 로거)
        
    Returns:
        로거 인스턴스
    """
    global _default_logger
    
    if name:
        return logging.getLogger(name)
    
    if _default_logger is None:
        _default_logger = setup_logger()
    
    return _default_logger
