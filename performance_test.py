#!/usr/bin/env python3
"""
🔬 DRONE 프로젝트 성능 테스트 및 벤치마크 도구
최적화 전후 성능 비교와 병목점 분석
"""

import time
import psutil
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple
import gc
import tracemalloc
from collections import defaultdict

class PerformanceBenchmark:
    """성능 벤치마크 도구"""
    
    def __init__(self):
        self.results = defaultdict(list)
        self.process = psutil.Process()
        
    def measure_time(self, func, *args, **kwargs):
        """함수 실행 시간 측정"""
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        execution_time = end_time - start_time
        return result, execution_time
    
    def measure_memory(self, func, *args, **kwargs):
        """함수 메모리 사용량 측정"""
        gc.collect()  # 가비지 컬렉션
        
        tracemalloc.start()
        memory_before = self.process.memory_info().rss
        
        result = func(*args, **kwargs)
        
        memory_after = self.process.memory_info().rss
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        
        memory_usage = {
            'rss_diff_mb': (memory_after - memory_before) / 1024 / 1024,
            'traced_current_mb': current / 1024 / 1024,
            'traced_peak_mb': peak / 1024 / 1024
        }
        
        return result, memory_usage
    
    def benchmark_function(self, name: str, func, iterations: int = 100, *args, **kwargs):
        """함수 벤치마크 실행"""
        print(f"🔬 벤치마킹: {name} ({iterations} iterations)")
        
        times = []
        memory_usages = []
        
        for i in range(iterations):
            # 시간 측정
            _, exec_time = self.measure_time(func, *args, **kwargs)
            times.append(exec_time)
            
            # 메모리 측정 (일부만)
            if i % 10 == 0:
                _, memory_info = self.measure_memory(func, *args, **kwargs)
                memory_usages.append(memory_info['traced_peak_mb'])
        
        # 통계 계산
        stats = {
            'mean_time': np.mean(times),
            'std_time': np.std(times),
            'min_time': np.min(times),
            'max_time': np.max(times),
            'total_time': np.sum(times),
            'mean_memory': np.mean(memory_usages) if memory_usages else 0,
            'max_memory': np.max(memory_usages) if memory_usages else 0
        }
        
        self.results[name] = stats
        self._print_stats(name, stats)
        
        return stats
    
    def _print_stats(self, name: str, stats: Dict):
        """통계 출력"""
        print(f"  ⚡ 평균 시간: {stats['mean_time']*1000:.2f}ms")
        print(f"  📊 표준편차: {stats['std_time']*1000:.2f}ms") 
        print(f"  💾 평균 메모리: {stats['mean_memory']:.2f}MB")
        print(f"  🔺 최대 메모리: {stats['max_memory']:.2f}MB")
        print()

def test_original_vs_optimized():
    """원본 vs 최적화 코드 성능 비교"""
    print("🏁 성능 비교 테스트 시작")
    print("=" * 50)
    
    benchmark = PerformanceBenchmark()
    
    # 테스트 데이터 준비
    test_positions = [np.array([i*10.0, i*8.0]) for i in range(100)]
    test_targets = [np.array([i*12.0, i*6.0]) for i in range(100)]
    
    # 1. 원본 스타일 구현 (비효율적)
    def original_navigation(positions, targets):
        results = []
        for pos, target in zip(positions, targets):
            # 매번 새로운 계산 (캐싱 없음)
            direction = target - pos
            distance = np.linalg.norm(direction)
            
            if distance > 0:
                unit_direction = direction / distance
                # 복잡한 계산들...
                for _ in range(10):  # 불필요한 반복
                    temp = np.random.random(2)
                    unit_direction += temp * 0.01
                
            results.append(unit_direction if distance > 0 else np.zeros(2))
        return results
    
    # 2. 최적화된 구현
    def optimized_navigation(positions, targets):
        positions_array = np.array(positions)
        targets_array = np.array(targets)
        
        # 벡터화된 계산
        directions = targets_array - positions_array
        distances = np.linalg.norm(directions, axis=1)
        
        # 안전한 나눗셈 (0으로 나누기 방지)
        mask = distances > 0
        unit_directions = np.zeros_like(directions)
        unit_directions[mask] = directions[mask] / distances[mask, np.newaxis]
        
        return unit_directions
    
    # 벤치마크 실행
    benchmark.benchmark_function(
        "Original Navigation", 
        original_navigation, 
        50, 
        test_positions, 
        test_targets
    )
    
    benchmark.benchmark_function(
        "Optimized Navigation", 
        optimized_navigation, 
        50, 
        test_positions, 
        test_targets
    )
    
    # 성능 개선 비율 계산
    original_stats = benchmark.results["Original Navigation"]
    optimized_stats = benchmark.results["Optimized Navigation"]
    
    time_improvement = original_stats['mean_time'] / optimized_stats['mean_time']
    memory_improvement = original_stats['mean_memory'] / max(optimized_stats['mean_memory'], 0.001)
    
    print("📊 성능 개선 결과")
    print(f"⚡ 시간 성능: {time_improvement:.1f}배 향상")
    print(f"💾 메모리 효율: {memory_improvement:.1f}배 향상")
    
    return benchmark.results

def test_memory_optimization():
    """메모리 최적화 테스트"""
    print("\n💾 메모리 최적화 테스트")
    print("=" * 30)
    
    # 1. 비효율적인 메모리 사용
    def memory_inefficient():
        data = []
        for i in range(10000):
            # 큰 배열을 계속 생성
            big_array = np.random.random((100, 100))
            data.append(big_array)
        return len(data)
    
    # 2. 효율적인 메모리 사용
    def memory_efficient():
        from collections import deque
        data = deque(maxlen=100)  # 크기 제한
        for i in range(10000):
            # 재사용 가능한 작은 배열
            small_array = np.random.random(10)
            data.append(small_array)
        return len(data)
    
    benchmark = PerformanceBenchmark()
    
    print("📈 메모리 비효율적 방법:")
    _, memory_info_bad = benchmark.measure_memory(memory_inefficient)
    print(f"  피크 메모리: {memory_info_bad['traced_peak_mb']:.1f}MB")
    
    print("📉 메모리 효율적 방법:")
    _, memory_info_good = benchmark.measure_memory(memory_efficient)
    print(f"  피크 메모리: {memory_info_good['traced_peak_mb']:.1f}MB")
    
    improvement = memory_info_bad['traced_peak_mb'] / memory_info_good['traced_peak_mb']
    print(f"💡 메모리 사용량 {improvement:.1f}배 감소")

def test_jit_compilation():
    """JIT 컴파일 성능 테스트"""
    print("\n🚀 JIT 컴파일 성능 테스트")
    print("=" * 35)
    
    # 일반 Python 함수
    def python_function(arr):
        result = np.zeros_like(arr)
        for i in range(len(arr)):
            result[i] = arr[i] ** 2 + np.sin(arr[i])
        return result
    
    # NumPy 벡터화 함수
    def numpy_function(arr):
        return arr ** 2 + np.sin(arr)
    
    # Numba JIT 컴파일 (가능한 경우)
    try:
        import numba
        
        @numba.jit(nopython=True)
        def numba_function(arr):
            result = np.zeros_like(arr)
            for i in range(len(arr)):
                result[i] = arr[i] ** 2 + np.sin(arr[i])
            return result
        
        numba_available = True
    except ImportError:
        print("⚠️ Numba가 설치되지 않았습니다. Numba 테스트를 건너뜁니다.")
        numba_available = False
    
    # 테스트 데이터
    test_array = np.random.random(10000).astype(np.float32)
    
    benchmark = PerformanceBenchmark()
    
    # 벤치마크 실행
    benchmark.benchmark_function("Python Loop", python_function, 20, test_array)
    benchmark.benchmark_function("NumPy Vectorized", numpy_function, 100, test_array)
    
    if numba_available:
        # JIT 웜업
        numba_function(test_array[:10])
        benchmark.benchmark_function("Numba JIT", numba_function, 100, test_array)

def test_visualization_performance():
    """시각화 성능 테스트"""
    print("\n🎨 시각화 성능 테스트")
    print("=" * 30)
    
    # 1. 매번 전체 다시 그리기
    def full_redraw():
        fig, ax = plt.subplots()
        x = np.linspace(0, 10, 1000)
        y = np.sin(x)
        
        for i in range(10):
            ax.clear()
            ax.plot(x, y + i*0.1)
            ax.set_title(f"Frame {i}")
        
        plt.close(fig)
    
    # 2. 데이터만 업데이트
    def efficient_update():
        fig, ax = plt.subplots()
        x = np.linspace(0, 10, 1000)
        y = np.sin(x)
        
        line, = ax.plot(x, y)
        ax.set_ylim(-2, 12)
        
        for i in range(10):
            line.set_ydata(y + i*0.1)
            ax.set_title(f"Frame {i}")
        
        plt.close(fig)
    
    benchmark = PerformanceBenchmark()
    
    benchmark.benchmark_function("Full Redraw", full_redraw, 5)
    benchmark.benchmark_function("Efficient Update", efficient_update, 5)

def generate_performance_report():
    """종합 성능 리포트 생성"""
    print("\n📋 성능 최적화 종합 리포트")
    print("=" * 40)
    
    # 시스템 정보
    cpu_count = psutil.cpu_count()
    memory_total = psutil.virtual_memory().total / (1024**3)
    
    print(f"🖥️ 시스템 정보:")
    print(f"  CPU 코어: {cpu_count}")
    print(f"  총 메모리: {memory_total:.1f}GB")
    print()
    
    # 모든 테스트 실행
    results = test_original_vs_optimized()
    test_memory_optimization()
    test_jit_compilation()
    test_visualization_performance()
    
    # 추천 사항
    print("\n💡 최적화 추천 사항:")
    print("1. numpy 배열을 float32로 사용하여 메모리 절약")
    print("2. deque를 사용하여 메모리 사용량 제한")
    print("3. 벡터화 연산으로 반복문 대체")
    print("4. numba JIT 컴파일러 사용")
    print("5. 시각화에서 불필요한 다시 그리기 제거")
    
    return results

def quick_performance_check():
    """빠른 성능 체크"""
    print("⚡ 빠른 성능 체크")
    print("=" * 20)
    
    # CPU 성능
    start = time.time()
    result = np.random.random((1000, 1000)).sum()
    cpu_time = time.time() - start
    print(f"CPU 성능: {cpu_time:.3f}초")
    
    # 메모리 정보
    memory = psutil.virtual_memory()
    print(f"메모리 사용률: {memory.percent:.1f}%")
    print(f"사용 가능: {memory.available / (1024**3):.1f}GB")
    
    # GPU 확인 (PyTorch)
    try:
        import torch
        if torch.cuda.is_available():
            gpu_name = torch.cuda.get_device_name(0)
            print(f"🎮 GPU: {gpu_name}")
        else:
            print("🎮 GPU: 사용 불가")
    except ImportError:
        print("🎮 GPU: PyTorch 미설치")

if __name__ == "__main__":
    print("🔬 DRONE 프로젝트 성능 테스트 도구")
    print("=" * 50)
    
    # 빠른 체크
    quick_performance_check()
    print()
    
    # 전체 리포트
    generate_performance_report()
    
    print("\n✅ 성능 테스트 완료!")
    print("💡 자세한 최적화 방법은 code_optimization_guide.md를 참조하세요.") 