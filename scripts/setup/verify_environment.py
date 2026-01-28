#!/usr/bin/env python3
"""
환경 및 버전 호환성 검증 스크립트
=================================

Isaac Sim 환경에서 모든 필요한 패키지가 올바르게 설치되어 있는지 확인합니다.

사용법 (컨테이너 내부):
    /isaacsim/python.sh /workspace/scripts/setup/verify_environment.py
"""

import sys
import importlib
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class PackageInfo:
    """패키지 정보"""
    name: str
    module_name: str
    min_version: Optional[str] = None
    required: bool = True
    category: str = "기타"


# 검증할 패키지 목록
PACKAGES_TO_VERIFY: List[PackageInfo] = [
    # Isaac Sim 내장 패키지
    PackageInfo("numpy", "numpy", "1.24.0", True, "데이터 처리"),
    PackageInfo("scipy", "scipy", "1.10.0", True, "데이터 처리"),
    PackageInfo("torch", "torch", "2.0.0", True, "딥러닝"),
    PackageInfo("opencv", "cv2", "4.5.0", True, "컴퓨터 비전"),
    PackageInfo("PIL", "PIL", "9.0.0", True, "이미지 처리"),
    PackageInfo("matplotlib", "matplotlib", "3.7.0", True, "시각화"),
    
    # 추가 설치 패키지
    PackageInfo("pandas", "pandas", "2.0.0", True, "데이터 분석"),
    PackageInfo("scikit-learn", "sklearn", "1.3.0", True, "머신러닝"),
    PackageInfo("PyYAML", "yaml", "6.0", True, "설정 파일"),
    PackageInfo("tqdm", "tqdm", "4.65.0", True, "유틸리티"),
    PackageInfo("pyzbar", "pyzbar", None, False, "QR 코드"),  # 선택적
    PackageInfo("plotly", "plotly", "5.15.0", False, "시각화"),  # 선택적
    PackageInfo("pytest", "pytest", "7.4.0", False, "테스트"),  # 선택적
]


def parse_version(version_str: str) -> Tuple[int, ...]:
    """버전 문자열을 비교 가능한 튜플로 변환"""
    try:
        # "1.2.3" -> (1, 2, 3)
        parts = version_str.split('.')
        return tuple(int(p.split('+')[0].split('a')[0].split('b')[0].split('rc')[0]) 
                     for p in parts[:3])
    except (ValueError, AttributeError):
        return (0, 0, 0)


def check_version(current: str, minimum: str) -> bool:
    """현재 버전이 최소 버전 이상인지 확인"""
    return parse_version(current) >= parse_version(minimum)


def verify_package(pkg: PackageInfo) -> Dict:
    """단일 패키지 검증"""
    result = {
        "name": pkg.name,
        "module": pkg.module_name,
        "category": pkg.category,
        "required": pkg.required,
        "installed": False,
        "version": None,
        "version_ok": True,
        "error": None,
    }
    
    try:
        module = importlib.import_module(pkg.module_name)
        result["installed"] = True
        
        # 버전 확인
        version = getattr(module, "__version__", None)
        if version is None:
            version = getattr(module, "VERSION", None)
        if version is None and hasattr(module, "version"):
            v = module.version
            version = v if isinstance(v, str) else getattr(v, "version", None)
        
        result["version"] = str(version) if version else "unknown"
        
        # 최소 버전 확인
        if pkg.min_version and version and version != "unknown":
            result["version_ok"] = check_version(str(version), pkg.min_version)
            
    except ImportError as e:
        result["error"] = str(e)
    except Exception as e:
        result["error"] = f"예외 발생: {e}"
    
    return result


def verify_isaacsim_components() -> List[Dict]:
    """Isaac Sim 핵심 컴포넌트 검증 (참고용)"""
    components = []
    
    isaacsim_modules = [
        ("isaacsim.core.api", "Isaac Sim Core API"),
        ("isaacsim.core.prims", "Isaac Sim Prims"),
        ("isaacsim.core.utils", "Isaac Sim Utils"),
        ("isaacsim.sensors.camera", "Camera Sensor"),
        ("isaacsim.sensors.rtx", "RTX Sensor"),
        ("isaacsim.robot.manipulators", "Robot Manipulators"),
        ("isaacsim.ros2.bridge", "ROS2 Bridge"),
        ("omni.isaac.core", "Omni Isaac Core (deprecated)"),
        ("pegasus.simulator", "Pegasus Simulator"),
    ]
    
    for module_name, display_name in isaacsim_modules:
        result = {
            "name": display_name,
            "module": module_name,
            "installed": False,
            "error": None,
        }
        
        try:
            importlib.import_module(module_name)
            result["installed"] = True
        except ImportError as e:
            result["error"] = str(e)
        except Exception as e:
            result["error"] = f"예외: {e}"
        
        components.append(result)
    
    return components


def check_isaacsim_runtime() -> bool:
    """Isaac Sim 런타임 환경인지 확인"""
    try:
        import carb
        return True
    except ImportError:
        return False


def print_results(results: List[Dict], title: str):
    """결과 출력"""
    print(f"\n{'='*60}")
    print(f" {title}")
    print('='*60)
    
    # 카테고리별 그룹화
    categories = {}
    for r in results:
        cat = r.get("category", "기타")
        if cat not in categories:
            categories[cat] = []
        categories[cat].append(r)
    
    total_ok = 0
    total_fail = 0
    total_warn = 0
    
    for cat, items in categories.items():
        print(f"\n[{cat}]")
        print("-" * 50)
        
        for r in items:
            name = r["name"]
            installed = r["installed"]
            version = r.get("version", "")
            version_ok = r.get("version_ok", True)
            required = r.get("required", True)
            error = r.get("error")
            
            if installed and version_ok:
                status = "✅"
                total_ok += 1
                version_str = f"v{version}" if version else ""
                print(f"  {status} {name}: {version_str}")
            elif installed and not version_ok:
                status = "⚠️"
                total_warn += 1
                print(f"  {status} {name}: v{version} (버전 낮음)")
            elif not installed and not required:
                status = "⚪"
                total_warn += 1
                print(f"  {status} {name}: 미설치 (선택적)")
            else:
                status = "❌"
                total_fail += 1
                err_msg = f" - {error}" if error else ""
                print(f"  {status} {name}: 미설치{err_msg}")
    
    print(f"\n{'='*60}")
    print(f" 결과: ✅ {total_ok} 성공 | ⚠️ {total_warn} 경고 | ❌ {total_fail} 실패")
    print('='*60)
    
    return total_fail == 0


def main():
    """메인 함수"""
    print("\n" + "="*60)
    print(" Isaac Sim 환경 검증")
    print("="*60)
    
    # Python 버전 확인
    print(f"\n🐍 Python: {sys.version}")
    print(f"📍 경로: {sys.executable}")
    
    # 패키지 검증
    print("\n패키지 검증 중...")
    results = [verify_package(pkg) for pkg in PACKAGES_TO_VERIFY]
    packages_ok = print_results(results, "Python 패키지")
    
    # Isaac Sim 런타임 환경 확인
    is_runtime = check_isaacsim_runtime()
    
    print(f"\n{'='*60}")
    print(" Isaac Sim 컴포넌트 (참고)")
    print('='*60)
    
    if not is_runtime:
        print("\n  ℹ️  현재 Isaac Sim 런타임 외부에서 실행 중입니다.")
        print("     Isaac Sim 컴포넌트는 시뮬레이션 실행 시에만 사용 가능합니다.")
        print("     (이것은 정상입니다 - Python 패키지만 검증하면 됩니다)\n")
        print("  📝 Isaac Sim 내부에서 테스트하려면:")
        print("     isaac-sim.sh --exec '/workspace/scripts/setup/verify_environment.py'")
        components_ok = True  # 런타임 외부에서는 성공으로 처리
    else:
        # Isaac Sim 런타임 내부에서 실행 중
        print("\n  ✅ Isaac Sim 런타임 환경 감지됨\n")
        components = verify_isaacsim_components()
        
        components_ok = True
        for c in components:
            if c["installed"]:
                print(f"  ✅ {c['name']}")
            else:
                # Pegasus는 선택적
                if "Pegasus" in c["name"] or "deprecated" in c["name"]:
                    print(f"  ⚪ {c['name']}: 선택적")
                else:
                    print(f"  ❌ {c['name']}: {c['error']}")
                    components_ok = False
    
    # 최종 결과
    print("\n" + "="*60)
    if packages_ok:
        print(" 🎉 환경 검증 완료! Python 패키지가 모두 준비되었습니다.")
        if not is_runtime:
            print("    (Isaac Sim 컴포넌트는 시뮬레이션 실행 시 자동 로드)")
    else:
        print(" ⚠️ 일부 패키지가 누락되었습니다. install_dependencies.sh를 실행하세요.")
    print("="*60 + "\n")
    
    return 0 if packages_ok else 1


if __name__ == "__main__":
    sys.exit(main())
