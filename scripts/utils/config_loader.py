"""
Configuration Loader Utility
============================

YAML/JSON 설정 파일 로드 및 저장 유틸리티
"""

import yaml
import json
from pathlib import Path
from typing import Dict, Any, Optional


def load_config(config_path: str) -> Dict[str, Any]:
    """
    설정 파일을 로드합니다.
    
    Args:
        config_path: 설정 파일 경로 (YAML 또는 JSON)
        
    Returns:
        설정 딕셔너리
        
    Raises:
        FileNotFoundError: 파일이 존재하지 않을 때
        ValueError: 지원하지 않는 파일 형식일 때
    """
    path = Path(config_path)
    
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")
    
    suffix = path.suffix.lower()
    
    with open(path, 'r', encoding='utf-8') as f:
        if suffix in ['.yaml', '.yml']:
            return yaml.safe_load(f)
        elif suffix == '.json':
            return json.load(f)
        else:
            raise ValueError(f"Unsupported config format: {suffix}")


def save_config(config: Dict[str, Any], config_path: str) -> None:
    """
    설정을 파일로 저장합니다.
    
    Args:
        config: 저장할 설정 딕셔너리
        config_path: 저장할 파일 경로
    """
    path = Path(config_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    
    suffix = path.suffix.lower()
    
    with open(path, 'w', encoding='utf-8') as f:
        if suffix in ['.yaml', '.yml']:
            yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
        elif suffix == '.json':
            json.dump(config, f, indent=2, ensure_ascii=False)
        else:
            raise ValueError(f"Unsupported config format: {suffix}")


def merge_configs(base_config: Dict[str, Any], override_config: Dict[str, Any]) -> Dict[str, Any]:
    """
    두 설정을 병합합니다 (override_config가 우선).
    
    Args:
        base_config: 기본 설정
        override_config: 덮어쓸 설정
        
    Returns:
        병합된 설정
    """
    result = base_config.copy()
    
    for key, value in override_config.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = merge_configs(result[key], value)
        else:
            result[key] = value
    
    return result
