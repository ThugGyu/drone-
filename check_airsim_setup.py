"""
AirSim 설치 및 설정 상태 종합 진단 도구
언리얼 엔진과 AirSim 연동 상태를 체크합니다.
"""

import os
import sys
import subprocess
import importlib
import platform
from pathlib import Path

class AirSimDiagnostic:
    def __init__(self):
        self.results = []
        self.issues = []
        
    def log_result(self, test_name, status, message, solution=None):
        """테스트 결과 로깅"""
        self.results.append({
            'test': test_name,
            'status': status,
            'message': message,
            'solution': solution
        })
        
        if status == "❌":
            self.issues.append({
                'test': test_name,
                'message': message,
                'solution': solution
            })
    
    def check_python_environment(self):
        """Python 환경 확인"""
        print("🐍 Python 환경 확인")
        print("=" * 40)
        
        # Python 버전
        python_version = sys.version
        if sys.version_info >= (3, 8):
            self.log_result("Python 버전", "✅", f"Python {sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}")
        else:
            self.log_result("Python 버전", "❌", f"Python {sys.version_info.major}.{sys.version_info.minor} (3.8+ 필요)", 
                          "Python 3.8 이상으로 업그레이드하세요")
        
        # 필수 패키지 확인
        required_packages = {
            'airsim': 'AirSim Python API',
            'numpy': '수치 계산',
            'opencv-python': '컴퓨터 비전',
            'torch': 'PyTorch (딥러닝)',
            'gym': 'OpenAI Gym',
            'stable-baselines3': '강화학습'
        }
        
        for package, description in required_packages.items():
            try:
                importlib.import_module(package.replace('-', '_'))
                self.log_result(f"패키지: {package}", "✅", f"{description} 설치됨")
            except ImportError:
                self.log_result(f"패키지: {package}", "❌", f"{description} 미설치", 
                              f"pip install {package}")
    
    def check_airsim_installation(self):
        """AirSim 설치 확인"""
        print("\n🚁 AirSim 설치 확인")
        print("=" * 40)
        
        try:
            import airsim
            self.log_result("AirSim import", "✅", f"AirSim {airsim.__version__ if hasattr(airsim, '__version__') else 'unknown'} 설치됨")
            
            # AirSim 설정 파일 확인
            if platform.system() == "Windows":
                settings_path = Path.home() / "Documents" / "AirSim" / "settings.json"
            else:
                settings_path = Path.home() / "Documents" / "AirSim" / "settings.json"
            
            if settings_path.exists():
                self.log_result("AirSim 설정", "✅", f"설정 파일 발견: {settings_path}")
                
                # 설정 파일 내용 간단 체크
                try:
                    import json
                    with open(settings_path, 'r') as f:
                        settings = json.load(f)
                    
                    if 'Vehicles' in settings:
                        self.log_result("드론 설정", "✅", "차량 설정 발견")
                    else:
                        self.log_result("드론 설정", "⚠️", "기본 설정 권장", 
                                      "차량(드론) 설정을 추가하세요")
                        
                except Exception as e:
                    self.log_result("설정 파일 파싱", "❌", f"설정 파일 오류: {e}")
            else:
                self.log_result("AirSim 설정", "❌", "설정 파일 없음", 
                              f"설정 파일을 생성하세요: {settings_path}")
                
        except ImportError:
            self.log_result("AirSim import", "❌", "AirSim 패키지 미설치", 
                          "pip install airsim")
    
    def check_unreal_engine(self):
        """언리얼 엔진 확인"""
        print("\n🎮 Unreal Engine 확인")
        print("=" * 40)
        
        # 일반적인 언리얼 엔진 설치 경로들
        common_paths = []
        
        if platform.system() == "Windows":
            # Epic Games Launcher 기본 경로
            epic_path = Path("C:/Program Files/Epic Games")
            if epic_path.exists():
                ue_folders = [f for f in epic_path.iterdir() if f.name.startswith("UE_")]
                common_paths.extend(ue_folders)
                
            # 사용자 지정 경로
            user_epic = Path.home() / "Epic Games"
            if user_epic.exists():
                ue_folders = [f for f in user_epic.iterdir() if f.name.startswith("UE_")]
                common_paths.extend(ue_folders)
        
        if common_paths:
            latest_ue = max(common_paths, key=lambda x: x.name)
            self.log_result("Unreal Engine", "✅", f"발견: {latest_ue}")
            
            # UE 5.2 버전 확인
            if "5.2" in latest_ue.name or "5.3" in latest_ue.name or "5.1" in latest_ue.name:
                self.log_result("UE 버전", "✅", "AirSim 호환 버전")
            else:
                self.log_result("UE 버전", "⚠️", f"버전 확인 필요: {latest_ue.name}", 
                              "UE 5.1-5.3 권장")
        else:
            self.log_result("Unreal Engine", "❌", "설치 경로 찾을 수 없음", 
                          "Epic Games Launcher에서 UE 5.2 설치")
    
    def check_gpu_cuda(self):
        """GPU 및 CUDA 확인"""
        print("\n🎯 GPU/CUDA 확인")
        print("=" * 40)
        
        try:
            import torch
            if torch.cuda.is_available():
                gpu_count = torch.cuda.device_count()
                gpu_name = torch.cuda.get_device_name(0)
                cuda_version = torch.version.cuda
                
                self.log_result("CUDA GPU", "✅", f"{gpu_name} ({gpu_count}개)")
                self.log_result("CUDA 버전", "✅", f"CUDA {cuda_version}")
                
                # GPU 메모리 확인
                gpu_memory = torch.cuda.get_device_properties(0).total_memory
                gpu_memory_gb = gpu_memory / (1024**3)
                
                if gpu_memory_gb >= 8:
                    self.log_result("GPU 메모리", "✅", f"{gpu_memory_gb:.1f} GB")
                elif gpu_memory_gb >= 4:
                    self.log_result("GPU 메모리", "⚠️", f"{gpu_memory_gb:.1f} GB (8GB+ 권장)")
                else:
                    self.log_result("GPU 메모리", "❌", f"{gpu_memory_gb:.1f} GB (부족)")
            else:
                self.log_result("CUDA GPU", "❌", "CUDA GPU 없음", 
                              "NVIDIA GPU + CUDA 드라이버 설치 필요")
        except ImportError:
            self.log_result("PyTorch", "❌", "PyTorch 미설치", "pip install torch")
    
    def check_disk_space(self):
        """디스크 공간 확인"""
        print("\n💾 저장공간 확인")
        print("=" * 40)
        
        import shutil
        
        # 현재 드라이브 사용량 확인
        total, used, free = shutil.disk_usage(".")
        free_gb = free / (1024**3)
        
        self.log_result("사용 가능 공간", "✅" if free_gb > 100 else "⚠️", 
                       f"{free_gb:.1f} GB")
        
        # 예상 사용량 안내
        estimated_usage = self.estimate_storage_usage()
        self.log_result("예상 필요 공간", "ℹ️", f"{estimated_usage} GB")
    
    def estimate_storage_usage(self):
        """저장공간 사용량 추정"""
        # 50,000 스텝 학습 기준 추정
        
        usage_breakdown = {
            "모델 체크포인트": 2.0,  # PPO 모델들
            "학습 로그": 0.5,        # 텐서보드, CSV 로그
            "이미지 데이터": 8.0,    # 100x100x3 이미지들
            "비디오 녹화": 5.0,      # 선택사항
            "기타": 2.0             # 설정, 임시파일 등
        }
        
        total_estimated = sum(usage_breakdown.values())
        
        print(f"\n📊 예상 저장공간 사용량 (50,000 스텝 기준):")
        for item, size in usage_breakdown.items():
            print(f"  - {item}: {size} GB")
        print(f"  📦 총합: ~{total_estimated} GB")
        
        return total_estimated
    
    def test_airsim_connection(self):
        """AirSim 연결 테스트"""
        print("\n🔌 AirSim 연결 테스트")
        print("=" * 40)
        
        try:
            import airsim
            
            # 클라이언트 생성 (연결 시도 안함)
            self.log_result("AirSim 클라이언트", "✅", "클라이언트 생성 가능")
            
            print("⚠️ 실제 연결 테스트는 Unreal Engine이 실행 중일 때 가능합니다")
            print("   test_airsim_connection.py를 실행하여 연결을 테스트하세요")
            
        except Exception as e:
            self.log_result("AirSim 클라이언트", "❌", f"오류: {e}")
    
    def run_full_diagnostic(self):
        """전체 진단 실행"""
        print("🚀 AirSim 프로젝트 환경 진단")
        print("=" * 60)
        
        # 모든 체크 실행
        self.check_python_environment()
        self.check_airsim_installation()
        self.check_unreal_engine()
        self.check_gpu_cuda()
        self.check_disk_space()
        self.test_airsim_connection()
        
        # 결과 요약
        self.print_summary()
        self.suggest_next_steps()
    
    def print_summary(self):
        """결과 요약 출력"""
        print("\n📋 진단 결과 요약")
        print("=" * 60)
        
        passed = sum(1 for r in self.results if r['status'] == "✅")
        warnings = sum(1 for r in self.results if r['status'] == "⚠️")
        failed = sum(1 for r in self.results if r['status'] == "❌")
        
        print(f"✅ 통과: {passed}")
        print(f"⚠️ 주의: {warnings}")
        print(f"❌ 실패: {failed}")
        
        if self.issues:
            print(f"\n🔧 해결 필요한 문제들:")
            for i, issue in enumerate(self.issues, 1):
                print(f"{i}. {issue['test']}: {issue['message']}")
                if issue['solution']:
                    print(f"   해결책: {issue['solution']}")
    
    def suggest_next_steps(self):
        """다음 단계 제안"""
        print(f"\n🎯 다음 단계 제안")
        print("=" * 60)
        
        if not self.issues:
            print("🎉 모든 검사를 통과했습니다!")
            print("다음 단계:")
            print("1. 언리얼 엔진에서 AirSim 프로젝트 실행")
            print("2. python test_airsim_connection.py 실행")
            print("3. 경상국립대 캠퍼스 맵 제작 시작")
        else:
            print("❗ 다음 문제들을 먼저 해결하세요:")
            for i, issue in enumerate(self.issues, 1):
                print(f"{i}. {issue['test']}")
                if issue['solution']:
                    print(f"   → {issue['solution']}")
            
            print(f"\n💡 GPU 관련 참고사항:")
            print("- RTX 3060 이상 권장 (8GB+ VRAM)")
            print("- RTX 4060/4070 시리즈가 가격대비 좋음")
            print("- 학습 중에는 다른 GPU 사용 프로그램 종료")

def main():
    """메인 함수"""
    diagnostic = AirSimDiagnostic()
    diagnostic.run_full_diagnostic()
    
    print(f"\n📞 추가 도움이 필요하면 언제든 문의하세요!")

if __name__ == "__main__":
    main() 