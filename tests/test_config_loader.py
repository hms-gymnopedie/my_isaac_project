"""
Config Loader 테스트
"""

import sys
from pathlib import Path

# 프로젝트 루트를 path에 추가
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from scripts.utils.config_loader import load_config, save_config, merge_configs


def test_load_yaml_config():
    """YAML 설정 파일 로드 테스트"""
    config_path = project_root / "configs" / "simulation.yaml"
    
    if config_path.exists():
        config = load_config(str(config_path))
        assert "simulation" in config
        print("✅ YAML config load: PASSED")
    else:
        print("⚠️ YAML config load: SKIPPED (file not found)")


def test_merge_configs():
    """설정 병합 테스트"""
    base = {"a": 1, "b": {"c": 2, "d": 3}}
    override = {"b": {"c": 10}, "e": 5}
    
    merged = merge_configs(base, override)
    
    assert merged["a"] == 1
    assert merged["b"]["c"] == 10
    assert merged["b"]["d"] == 3
    assert merged["e"] == 5
    
    print("✅ Config merge: PASSED")


if __name__ == "__main__":
    test_load_yaml_config()
    test_merge_configs()
    print("\n✅ All tests passed!")
