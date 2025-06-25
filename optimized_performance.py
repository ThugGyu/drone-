#!/usr/bin/env python3
"""
🚀 성능 최적화된 드론 시스템
메모리 효율성과 실행 속도 개선
"""

import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import numba
from typing import List, Tuple, Dict
import gc

class OptimizedDroneSystem:
    """메모리와 속도 최적화된 드론 시스템"""
    
    def __init__(self, max_history: int = 100):
        # 메모리 효율적인 데이터 구조 사용
        self.position = np.array([45.0, 82.0], dtype=np.float32)  # float32 사용
        self.flight_history = deque(maxlen=max_history)  # 크기 제한된 deque
        self.last_positions = deque(maxlen=10)  # 최근 위치만 저장
        
        # 캠퍼스 데이터를 numpy 배열로 최적화
        self.buildings = self._optimize_buildings_data()
        self.waypoints = self._optimize_waypoints_data()
        
        # 사전 계산된 값들 캐싱
        self._cached_directions = {}
        self._cached_distances = {}
        
    def _optimize_buildings_data(self) -> np.ndarray:
        """건물 데이터를 numpy 배열로 최적화"""
        # [x, y, width, height, center_x, center_y] 형태로 구조화
        buildings_data = np.array([
            [45, 82, 12, 6, 51, 85],    # Main Gate
            [85, 87, 12, 10, 91, 92],   # Central Plaza
            [55, 110, 20, 18, 65, 119], # Engineering
            [110, 100, 20, 16, 120, 108], # Library
            [140, 115, 20, 16, 150, 123], # Science Hall
        ], dtype=np.float32)
        return buildings_data
    
    def _optimize_waypoints_data(self) -> np.ndarray:
        """경유점 데이터를 numpy 배열로 최적화"""
        waypoints = np.array([
            [45.0, 82.0], [92.0, 95.0], [103.0, 75.0],
            [175.0, 70.0], [140.0, 80.0], [155.0, 130.0],
            [105.0, 130.0], [125.0, 108.0], [65.0, 145.0],
            [65.0, 120.0], [55.0, 55.0], [45.0, 82.0]
        ], dtype=np.float32)
        return waypoints
    
    @numba.jit(nopython=True)  # JIT 컴파일로 속도 향상
    def _fast_distance_calculation(self, pos1: np.ndarray, pos2: np.ndarray) -> float:
        """빠른 거리 계산"""
        diff = pos1 - pos2
        return np.sqrt(diff[0]**2 + diff[1]**2)
    
    @numba.jit(nopython=True)
    def _fast_obstacle_avoidance(self, pos: np.ndarray, buildings: np.ndarray) -> np.ndarray:
        """JIT 컴파일된 빠른 장애물 회피"""
        avoidance = np.zeros(2, dtype=np.float32)
        
        for i in range(len(buildings)):
            building = buildings[i]
            center_x, center_y = building[4], building[5]
            
            dx = pos[0] - center_x
            dy = pos[1] - center_y
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance < 20.0 and distance > 0.1:
                strength = (20.0 - distance) / 20.0
                avoidance[0] += (dx / distance) * strength
                avoidance[1] += (dy / distance) * strength
        
        return avoidance
    
    def optimized_navigation(self, target_pos: np.ndarray) -> np.ndarray:
        """최적화된 네비게이션 시스템"""
        # 캐시 확인
        cache_key = (tuple(self.position), tuple(target_pos))
        if cache_key in self._cached_directions:
            return self._cached_directions[cache_key]
        
        # 벡터화된 계산
        direction_vector = target_pos - self.position
        distance = np.linalg.norm(direction_vector)
        
        if distance < 1.0:
            result = np.random.uniform(-0.5, 0.5, 2).astype(np.float32)
        else:
            unit_direction = direction_vector / distance
            obstacle_avoidance = self._fast_obstacle_avoidance(self.position, self.buildings)
            result = unit_direction + obstacle_avoidance * 0.3
            result = result / np.linalg.norm(result) if np.linalg.norm(result) > 0 else result
        
        # 결과 캐싱 (메모리 제한)
        if len(self._cached_directions) < 1000:
            self._cached_directions[cache_key] = result
        
        return result * 2.0  # 속도 적용
    
    def memory_efficient_update(self, target_pos: np.ndarray):
        """메모리 효율적인 위치 업데이트"""
        # 이전 위치 저장 (메모리 제한됨)
        self.last_positions.append(self.position.copy())
        
        # 최적화된 이동 계산
        movement = self.optimized_navigation(target_pos)
        self.position += movement * 0.1
        
        # 비행 기록 추가 (자동으로 오래된 데이터 제거)
        self.flight_history.append(self.position.copy())
        
        # 주기적 메모리 정리
        if len(self.flight_history) % 50 == 0:
            gc.collect()  # 가비지 컬렉션
    
    def batch_process_waypoints(self, waypoint_indices: List[int]) -> List[np.ndarray]:
        """배치 처리로 여러 경유점 동시 계산"""
        batch_results = []
        
        # 벡터화된 배치 계산
        targets = self.waypoints[waypoint_indices]
        current_pos_expanded = np.expand_dims(self.position, 0)
        
        # 모든 목표점까지의 거리를 한 번에 계산
        distances = np.linalg.norm(targets - current_pos_expanded, axis=1)
        
        for i, target in enumerate(targets):
            if distances[i] < 1.0:
                movement = np.random.uniform(-0.5, 0.5, 2).astype(np.float32)
            else:
                direction = (target - self.position) / distances[i]
                obstacle_avoidance = self._fast_obstacle_avoidance(self.position, self.buildings)
                movement = direction + obstacle_avoidance * 0.3
                movement = movement / np.linalg.norm(movement) if np.linalg.norm(movement) > 0 else movement
            
            batch_results.append(movement * 2.0)
        
        return batch_results
    
    def get_performance_stats(self) -> Dict[str, float]:
        """성능 통계 반환"""
        return {
            "memory_usage_mb": self._get_memory_usage(),
            "cache_hit_ratio": len(self._cached_directions) / max(1, len(self.flight_history)),
            "history_length": len(self.flight_history),
            "position_precision": np.finfo(self.position.dtype).precision
        }
    
    def _get_memory_usage(self) -> float:
        """대략적인 메모리 사용량 계산"""
        history_size = len(self.flight_history) * 2 * 4  # float32 * 2 coordinates
        cache_size = len(self._cached_directions) * 4 * 4  # 4 floats per entry
        buildings_size = self.buildings.nbytes
        waypoints_size = self.waypoints.nbytes
        
        total_bytes = history_size + cache_size + buildings_size + waypoints_size
        return total_bytes / (1024 * 1024)  # MB로 변환

def performance_comparison():
    """성능 비교 함수"""
    print("🚀 성능 최적화 비교")
    print("=" * 50)
    
    # 최적화된 시스템
    optimized_system = OptimizedDroneSystem()
    
    # 성능 테스트
    import time
    
    start_time = time.time()
    for i in range(1000):
        target_idx = i % len(optimized_system.waypoints)
        target = optimized_system.waypoints[target_idx]
        optimized_system.memory_efficient_update(target)
    
    execution_time = time.time() - start_time
    stats = optimized_system.get_performance_stats()
    
    print(f"⚡ 실행 시간: {execution_time:.3f}초 (1000 iterations)")
    print(f"💾 메모리 사용량: {stats['memory_usage_mb']:.2f}MB")
    print(f"📊 캐시 효율성: {stats['cache_hit_ratio']:.3f}")
    print(f"🎯 위치 정밀도: {stats['position_precision']}")
    
    return optimized_system

if __name__ == "__main__":
    performance_comparison() 