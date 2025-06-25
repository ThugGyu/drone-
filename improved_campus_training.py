#!/usr/bin/env python3
"""
🎓 개선된 경상국립대 드론 자율주행 강화학습
문제점 개선:
1. 더 강한 경계 페널티
2. 중간 체크포인트 추가
3. 더 나은 보상 함수
4. 개선된 탐색 전략
"""

import numpy as np
import matplotlib.pyplot as plt
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
import gymnasium as gym
from gymnasium import spaces
import math
import time
import os

class ImprovedCampusDroneEnv(gym.Env):
    """개선된 경상국립대 캠퍼스 드론 환경"""
    
    def __init__(self):
        super().__init__()
        
        # 캠퍼스 설정 (1/10 스케일: 300m x 200m)
        self.campus_width = 300
        self.campus_height = 200
        
        # 액션 스페이스: [전진/후진, 좌/우, 상/하, 회전]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32
        )
        
        # 관찰 스페이스: 20차원
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(20,), dtype=np.float32
        )
        
        # 캠퍼스 건물들 (위험 구역으로 더 넓게 설정)
        self.buildings = [
            {"name": "Engineering", "pos": (90, 70), "size": (60, 50), "danger_zone": 20},
            {"name": "Library", "pos": (190, 90), "size": (55, 45), "danger_zone": 20},
            {"name": "Dormitory", "pos": (40, 140), "size": (50, 60), "danger_zone": 20},
            {"name": "Student Hall", "pos": (170, 40), "size": (45, 40), "danger_zone": 20},
            {"name": "Gymnasium", "pos": (240, 140), "size": (50, 55), "danger_zone": 20},
        ]
        
        # 🎯 개선된 경유점들 (더 많은 중간 체크포인트)
        self.waypoints = [
            (30, 30),      # 1. 시작점 (정문)
            (80, 60),      # 2. 첫 번째 중간점
            (130, 80),     # 3. 두 번째 중간점  
            (150, 100),    # 4. 중앙광장
            (120, 95),     # 5. 공과대학 앞
            (65, 170),     # 6. 기숙사 앞
            (100, 120),    # 7. 복귀 중간점
            (30, 30),      # 8. 복귀점 (정문)
        ]
        
        self.reset()
    
    def reset(self, seed=None):
        if seed is not None:
            np.random.seed(seed)
        
        # 드론 초기 상태
        self.position = np.array([30.0, 30.0], dtype=np.float32)
        self.velocity = np.array([0.0, 0.0], dtype=np.float32)
        self.heading = 0.0
        self.altitude = 20.0
        
        # 목표 관리
        self.current_waypoint_idx = 1
        self.visited_waypoints = [0]
        self.max_steps = 1000  # 더 긴 에피소드
        self.step_count = 0
        
        # 성능 추적
        self.total_distance_traveled = 0
        self.last_distance_to_target = self._get_distance_to_current_target()
        self.stuck_counter = 0  # 갇힘 감지
        self.last_positions = []  # 최근 위치 기록
        
        return self._get_observation(), {}
    
    def _get_distance_to_current_target(self):
        """현재 목표까지의 거리"""
        target = self.waypoints[self.current_waypoint_idx]
        return np.linalg.norm(np.array(target) - self.position)
    
    def _get_observation(self):
        """현재 상태를 관찰값으로 변환"""
        # 현재 목표점
        target = self.waypoints[self.current_waypoint_idx]
        
        # 목표까지의 거리와 방향
        target_vector = np.array(target) - self.position
        target_distance = np.linalg.norm(target_vector)
        target_angle = math.atan2(target_vector[1], target_vector[0])
        
        # 다음 목표점 정보 (미리보기)
        next_target = self.waypoints[min(self.current_waypoint_idx + 1, len(self.waypoints) - 1)]
        next_target_vector = np.array(next_target) - self.position
        next_target_distance = np.linalg.norm(next_target_vector)
        
        # 8방향 LiDAR 시뮬레이션
        lidar_ranges = self._get_lidar_readings()
        
        # 경계까지의 거리
        boundary_distances = [
            self.position[0],  # 왼쪽 경계
            self.campus_width - self.position[0],  # 오른쪽 경계
            self.position[1],  # 아래 경계  
            self.campus_height - self.position[1]  # 위 경계
        ]
        min_boundary_distance = min(boundary_distances) / 50.0  # 정규화
        
        # 관찰값 구성 (20차원)
        obs = np.concatenate([
            self.position / 150.0,                # 정규화된 현재 위치 (2)
            self.velocity / 10.0,                 # 정규화된 속도 (2)
            [self.heading / np.pi],               # 정규화된 방향 (1)
            [target_distance / 200.0],            # 정규화된 목표 거리 (1)
            [target_angle / np.pi],               # 정규화된 목표 각도 (1)
            [self.current_waypoint_idx / len(self.waypoints)],  # 진행률 (1)
            [len(self.visited_waypoints) / len(self.waypoints)], # 방문률 (1)
            lidar_ranges,                         # LiDAR 8방향 (8)
            [min_boundary_distance],              # 경계 거리 (1)
            [self.step_count / self.max_steps],   # 시간 진행률 (1)
            [np.sin(self.heading)]                # 방향 벡터 (1)
        ])
        
        return obs.astype(np.float32)
    
    def _get_lidar_readings(self):
        """개선된 8방향 LiDAR 센서"""
        ranges = []
        max_range = 80.0
        
        for i in range(8):
            angle = i * math.pi / 4
            direction = np.array([math.cos(angle), math.sin(angle)])
            
            distance = max_range
            
            # 건물 충돌 검사
            for building in self.buildings:
                bx, by = building["pos"]
                bw, bh = building["size"]
                danger = building["danger_zone"]
                
                # 확장된 위험 구역 포함
                expanded_box = {
                    "x": bx - danger,
                    "y": by - danger, 
                    "w": bw + 2 * danger,
                    "h": bh + 2 * danger
                }
                
                if self._ray_box_intersection(self.position, direction, expanded_box):
                    dist = np.linalg.norm(np.array([bx + bw/2, by + bh/2]) - self.position)
                    distance = min(distance, max(dist - danger, 5))
            
            # 경계 충돌 검사
            boundary_dist = self._ray_boundary_intersection(self.position, direction)
            distance = min(distance, boundary_dist)
            
            ranges.append(distance / max_range)  # 정규화
        
        return np.array(ranges)
    
    def _ray_boundary_intersection(self, origin, direction):
        """레이와 경계의 교차점까지 거리"""
        distances = []
        
        # 각 경계와의 교차점 계산
        if direction[0] > 0:  # 동쪽
            distances.append((self.campus_width - origin[0]) / direction[0])
        elif direction[0] < 0:  # 서쪽
            distances.append(-origin[0] / direction[0])
            
        if direction[1] > 0:  # 북쪽
            distances.append((self.campus_height - origin[1]) / direction[1])
        elif direction[1] < 0:  # 남쪽
            distances.append(-origin[1] / direction[1])
        
        return min([d for d in distances if d > 0] + [100])
    
    def _ray_box_intersection(self, origin, direction, box):
        """레이와 박스의 교차점 검사"""
        box_center = np.array([box["x"] + box["w"]/2, box["y"] + box["h"]/2])
        to_box = box_center - origin
        distance_to_center = np.linalg.norm(to_box)
        
        return distance_to_center < (box["w"] + box["h"]) / 3
    
    def step(self, action):
        self.step_count += 1
        
        # 이전 위치 저장
        prev_position = self.position.copy()
        
        # 🚀 개선된 액션 적용
        acceleration = action[:2] * 1.5  # 가속도 조정
        self.velocity += acceleration
        self.velocity = np.clip(self.velocity, -8, 8)  # 속도 제한
        
        # 위치 업데이트 (관성 추가)
        self.position += self.velocity * 0.3 + acceleration * 0.1
        self.heading += action[3] * 0.15  # 회전
        self.altitude += action[2] * 0.8
        self.altitude = np.clip(self.altitude, 10, 40)
        
        # 이동 거리 추적
        movement = np.linalg.norm(self.position - prev_position)
        self.total_distance_traveled += movement
        
        # 🔒 강화된 경계 제한
        old_position = self.position.copy()
        self.position = np.clip(self.position, [10, 10], [self.campus_width-10, self.campus_height-10])
        
        # 갇힘 감지
        self.last_positions.append(self.position.copy())
        if len(self.last_positions) > 20:
            self.last_positions.pop(0)
            
        if len(self.last_positions) >= 20:
            recent_movement = np.std([np.linalg.norm(pos - self.last_positions[0]) 
                                    for pos in self.last_positions[-10:]])
            if recent_movement < 5:
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0
        
        # 보상 계산
        reward = self._calculate_improved_reward(old_position, movement)
        
        # 종료 조건
        done = (self.step_count >= self.max_steps or 
                self.current_waypoint_idx >= len(self.waypoints) or
                self.stuck_counter > 50)  # 갇힘 감지 종료
        
        return self._get_observation(), reward, done, False, {}
    
    def _calculate_improved_reward(self, old_position, movement):
        """🎯 크게 개선된 보상 함수"""
        reward = 0
        
        # 현재 목표점
        target = self.waypoints[self.current_waypoint_idx]
        current_distance = np.linalg.norm(np.array(target) - self.position)
        
        # 1. 목표 접근 보상 (거리 기반)
        distance_improvement = self.last_distance_to_target - current_distance
        reward += distance_improvement * 5  # 가까워지면 보상
        self.last_distance_to_target = current_distance
        
        # 2. 🎯 목표점 도달 대형 보상
        if current_distance < 12:  # 12m 내 도달
            if self.current_waypoint_idx not in self.visited_waypoints:
                self.visited_waypoints.append(self.current_waypoint_idx)
                reward += 500  # 🎉 대형 보상!
                print(f"🎯 경유점 {self.current_waypoint_idx} 도달! 보상 +500")
            
            # 다음 목표점으로 이동
            if self.current_waypoint_idx < len(self.waypoints) - 1:
                self.current_waypoint_idx += 1
                self.last_distance_to_target = self._get_distance_to_current_target()
                reward += 100  # 진행 보상
        
        # 3. 💥 강화된 충돌 및 경계 페널티
        # 건물 충돌
        for building in self.buildings:
            bx, by = building["pos"]
            bw, bh = building["size"]
            danger = building["danger_zone"]
            
            # 위험 구역 침입
            if (bx - danger <= self.position[0] <= bx + bw + danger and 
                by - danger <= self.position[1] <= by + bh + danger):
                reward -= 100
                
            # 건물 직접 충돌
            if (bx <= self.position[0] <= bx + bw and 
                by <= self.position[1] <= by + bh):
                reward -= 1000  # 💥 매우 큰 페널티
        
        # 4. 🚫 경계 근접 및 이탈 페널티
        boundary_distances = [
            self.position[0],  # 왼쪽
            self.campus_width - self.position[0],  # 오른쪽
            self.position[1],  # 아래
            self.campus_height - self.position[1]  # 위
        ]
        min_boundary = min(boundary_distances)
        
        if min_boundary < 15:  # 경계 15m 내 접근
            reward -= (15 - min_boundary) * 20  # 거리 비례 페널티
            
        if min_boundary < 5:  # 경계 5m 내 위험
            reward -= 500  # 강한 페널티
        
        # 5. 🔄 갇힘 방지 페널티
        if self.stuck_counter > 10:
            reward -= self.stuck_counter * 5  # 갇힘 시간 비례 페널티
        
        # 6. ⚡ 효율성 보상
        if movement > 0.5:  # 적절한 이동
            reward += 2
        elif movement < 0.1:  # 정체
            reward -= 5
            
        # 7. 🎯 방향성 보상
        target_vector = np.array(target) - self.position
        if np.linalg.norm(target_vector) > 0:
            target_direction = target_vector / np.linalg.norm(target_vector)
            velocity_direction = self.velocity / (np.linalg.norm(self.velocity) + 1e-6)
            alignment = np.dot(target_direction, velocity_direction)
            reward += alignment * 3  # 올바른 방향으로 이동 시 보상
        
        # 8. 🏆 진행 보상
        progress_reward = len(self.visited_waypoints) * 50
        reward += progress_reward * 0.01  # 지속적인 진행 보상
        
        return reward

def train_improved_drone():
    """개선된 드론 훈련"""
    print("🚀 개선된 경상국립대 드론 자율주행 훈련")
    print("="*60)
    
    # GPU 사용 가능 여부 확인
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"🖥️ 사용 디바이스: {device}")
    
    # 환경 생성
    env = DummyVecEnv([lambda: ImprovedCampusDroneEnv()])
    
    # 🎯 개선된 PPO 하이퍼파라미터
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=5e-4,        # 더 높은 학습률
        n_steps=2048,              # 더 긴 롤아웃
        batch_size=256,            # 더 큰 배치
        n_epochs=20,               # 더 많은 에포크
        gamma=0.995,               # 더 높은 할인율
        gae_lambda=0.98,           # GAE 람다
        clip_range=0.2,
        ent_coef=0.01,             # 탐색 장려
        vf_coef=0.5,
        max_grad_norm=0.5,
        device=device,
        verbose=1
    )
    
    print(f"🎯 총 학습 스텝: 50,000")
    print(f"⏰ 시작 시간: {time.strftime('%H:%M:%S')}")
    print("-" * 60)
    
    # 학습 실행
    callback_count = 0
    def progress_callback(locals, globals):
        nonlocal callback_count
        callback_count += 1
        
        if callback_count % 5 == 0:  # 매 5번째 업데이트마다
            steps = locals['self'].num_timesteps
            if steps % 5000 == 0:
                # 중간 모델 저장
                model_path = f"DRONE/models/improved_drone_{steps}"
                locals['self'].save(model_path)
                print(f"Step {steps}: 모델 저장됨")
        
        return True
    
    # 훈련 시작
    start_time = time.time()
    model.learn(total_timesteps=50000, callback=progress_callback)
    end_time = time.time()
    
    # 최종 모델 저장
    model.save("DRONE/models/improved_drone_final")
    
    training_time = (end_time - start_time) / 60
    print("="*60)
    print(f"🎉 개선된 훈련 완료!")
    print(f"⏱️ 총 훈련 시간: {training_time:.1f}분")
    print(f"💾 최종 모델 저장: DRONE/models/improved_drone_final")

if __name__ == "__main__":
    print("🎓 개선된 드론 훈련 프로그램")
    print("="*40)
    print("개선 사항:")
    print("✅ 강화된 경계 페널티")
    print("✅ 더 많은 중간 체크포인트")  
    print("✅ 개선된 보상 함수")
    print("✅ 갇힘 감지 및 방지")
    print("✅ 더 긴 훈련 (50,000 스텝)")
    print("="*40)
    
    choice = input("훈련을 시작하시겠습니까? (y/N): ").lower()
    if choice in ['y', 'yes', '예']:
        train_improved_drone()
    else:
        print("훈련이 취소되었습니다.") 