#!/usr/bin/env python3
"""
🎓 최적화된 강화학습 드론 훈련 시스템
- 더 빠른 학습 속도
- 향상된 수렴성
- 메모리 효율적인 환경
"""

import numpy as np
import torch
import torch.nn as nn
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize
from stable_baselines3.common.callbacks import BaseCallback
import gymnasium as gym
from gymnasium import spaces
import multiprocessing as mp
from typing import Dict, Any
import wandb  # 실험 추적용

class OptimizedDroneEnv(gym.Env):
    """최적화된 드론 환경 - 빠른 학습용"""
    
    def __init__(self, config: Dict[str, Any] = None):
        super().__init__()
        
        self.config = config or self.get_default_config()
        
        # 작은 액션 스페이스로 학습 효율성 향상
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32  # [x, y] 이동만
        )
        
        # 압축된 관찰 스페이스 (12차원으로 축소)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32
        )
        
        # 최적화된 캠퍼스 데이터 (numpy 배열)
        self.buildings = np.array([
            [90, 70, 40, 30],   # Engineering [x, y, w, h]
            [190, 90, 35, 25],  # Library
            [40, 140, 30, 40],  # Dormitory
            [170, 40, 25, 25],  # Student Hall
            [240, 140, 30, 35], # Gymnasium
        ], dtype=np.float32)
        
        # 단순화된 경유점 (5개로 축소)
        self.waypoints = np.array([
            [30, 30],   # Start
            [100, 80],  # Checkpoint 1
            [150, 120], # Checkpoint 2
            [100, 120], # Checkpoint 3
            [30, 30],   # End
        ], dtype=np.float32)
        
        self.reset()
    
    @staticmethod
    def get_default_config():
        """기본 설정"""
        return {
            "max_steps": 500,
            "success_reward": 100.0,
            "collision_penalty": -50.0,
            "progress_reward_scale": 10.0,
            "step_penalty": -0.1,
            "boundary_penalty": -10.0
        }
    
    def reset(self, seed=None):
        """빠른 리셋"""
        if seed is not None:
            np.random.seed(seed)
        
        # 초기 상태
        self.position = np.array([30.0, 30.0], dtype=np.float32)
        self.velocity = np.zeros(2, dtype=np.float32)
        self.current_waypoint = 1
        self.step_count = 0
        self.visited = [0]
        
        # 성능 추적 변수들
        self.last_distance = self._distance_to_target()
        self.total_reward = 0.0
        
        return self._get_observation(), {}
    
    def _get_observation(self):
        """압축된 12차원 관찰값"""
        target = self.waypoints[self.current_waypoint]
        to_target = target - self.position
        distance = np.linalg.norm(to_target)
        
        # 4방향 간단한 센서
        sensors = self._get_simple_sensors()
        
        obs = np.concatenate([
            self.position / 150.0,              # 정규화된 위치 (2)
            to_target / 200.0,                  # 목표 방향 (2)
            [distance / 200.0],                 # 목표 거리 (1)
            [self.current_waypoint / len(self.waypoints)], # 진행률 (1)
            sensors,                            # 4방향 센서 (4)
            self.velocity / 5.0,                # 속도 (2)
        ])
        
        return obs.astype(np.float32)
    
    def _get_simple_sensors(self):
        """4방향 간단한 센서 (동, 서, 남, 북)"""
        directions = np.array([[1,0], [-1,0], [0,-1], [0,1]], dtype=np.float32)
        sensors = []
        
        for direction in directions:
            distance = 50.0  # 기본 거리
            
            # 건물과의 최단 거리 계산
            for building in self.buildings:
                bx, by, bw, bh = building
                center = np.array([bx + bw/2, by + bh/2])
                to_building = center - self.position
                
                if np.dot(to_building, direction) > 0:  # 해당 방향에 있으면
                    dist = np.linalg.norm(to_building)
                    distance = min(distance, max(dist - 20, 5))
            
            sensors.append(distance / 50.0)  # 정규화
        
        return np.array(sensors, dtype=np.float32)
    
    def _distance_to_target(self):
        """목표까지의 거리"""
        return np.linalg.norm(self.waypoints[self.current_waypoint] - self.position)
    
    def step(self, action):
        """최적화된 스텝 함수"""
        self.step_count += 1
        
        # 액션 적용 (스케일링)
        movement = action * 3.0  # 움직임 크기
        self.velocity = movement * 0.7 + self.velocity * 0.3  # 관성
        self.position += self.velocity
        
        # 보상 계산
        reward = self._calculate_optimized_reward()
        
        # 종료 조건
        done = self._check_termination()
        
        # 목표 달성 확인
        if self._distance_to_target() < 10.0:
            if self.current_waypoint not in self.visited:
                self.visited.append(self.current_waypoint)
                reward += self.config["success_reward"]
            
            self.current_waypoint = min(self.current_waypoint + 1, len(self.waypoints) - 1)
        
        info = {
            "success": len(self.visited) == len(self.waypoints),
            "progress": len(self.visited) / len(self.waypoints),
            "distance": self._distance_to_target()
        }
        
        return self._get_observation(), reward, done, False, info
    
    def _calculate_optimized_reward(self):
        """최적화된 간단한 보상 함수"""
        # 목표까지의 거리 기반 보상
        current_distance = self._distance_to_target()
        progress_reward = (self.last_distance - current_distance) * self.config["progress_reward_scale"]
        self.last_distance = current_distance
        
        # 기본 페널티
        reward = progress_reward + self.config["step_penalty"]
        
        # 충돌 검사 (간단화)
        if self._is_collision():
            reward += self.config["collision_penalty"]
        
        # 경계 검사
        if not (10 < self.position[0] < 290 and 10 < self.position[1] < 190):
            reward += self.config["boundary_penalty"]
            self.position = np.clip(self.position, [10, 10], [290, 190])
        
        return reward
    
    def _is_collision(self):
        """간단한 충돌 검사"""
        for building in self.buildings:
            bx, by, bw, bh = building
            if (bx - 5 <= self.position[0] <= bx + bw + 5 and 
                by - 5 <= self.position[1] <= by + bh + 5):
                return True
        return False
    
    def _check_termination(self):
        """종료 조건"""
        return (self.step_count >= self.config["max_steps"] or 
                len(self.visited) == len(self.waypoints))

class OptimizedTrainingCallback(BaseCallback):
    """최적화된 학습 콜백"""
    
    def __init__(self, eval_freq=1000, save_freq=5000):
        super().__init__()
        self.eval_freq = eval_freq
        self.save_freq = save_freq
        self.best_mean_reward = -np.inf
    
    def _on_step(self):
        """스텝마다 실행"""
        if self.n_calls % self.eval_freq == 0:
            self._evaluate_model()
        
        if self.n_calls % self.save_freq == 0:
            self._save_model()
        
        return True
    
    def _evaluate_model(self):
        """모델 평가"""
        # 간단한 평가 로직
        recent_rewards = [info.get("episode", {}).get("r", 0) 
                         for info in self.locals.get("infos", [])]
        
        if recent_rewards:
            mean_reward = np.mean(recent_rewards)
            print(f"평가 스텝 {self.n_calls}: 평균 보상 = {mean_reward:.2f}")
            
            if mean_reward > self.best_mean_reward:
                self.best_mean_reward = mean_reward
                print(f"새로운 최고 성능! {mean_reward:.2f}")
    
    def _save_model(self):
        """모델 저장"""
        model_path = f"models/optimized_drone_step_{self.n_calls}.zip"
        self.model.save(model_path)
        print(f"모델 저장: {model_path}")

def create_optimized_env():
    """최적화된 환경 생성"""
    return OptimizedDroneEnv()

def train_optimized_model():
    """최적화된 모델 훈련"""
    print("🚀 최적화된 드론 모델 훈련 시작!")
    
    # 병렬 환경 설정
    num_cpu = min(4, mp.cpu_count())
    env = SubprocVecEnv([create_optimized_env for _ in range(num_cpu)])
    env = VecNormalize(env, norm_obs=True, norm_reward=True)
    
    # 최적화된 모델 설정
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=1024,        # 배치 크기 증가
        batch_size=64,       # 미니배치 크기
        n_epochs=10,         # 에포크 수
        gamma=0.99,          # 할인 인자
        gae_lambda=0.95,     # GAE 람다
        clip_range=0.2,      # PPO 클리핑
        ent_coef=0.01,       # 엔트로피 계수
        vf_coef=0.5,         # 값 함수 계수
        max_grad_norm=0.5,   # 그래디언트 클리핑
        verbose=1,
        device="auto",       # GPU 자동 감지
        tensorboard_log="./tensorboard_logs/"
    )
    
    # 콜백 설정
    callback = OptimizedTrainingCallback(eval_freq=2000, save_freq=10000)
    
    # 훈련 시작
    print(f"🔥 {num_cpu}개 CPU 코어로 병렬 훈련")
    model.learn(
        total_timesteps=100000,  # 빠른 학습을 위해 축소
        callback=callback,
        tb_log_name="optimized_drone_training"
    )
    
    # 최종 모델 저장
    model.save("models/optimized_drone_final.zip")
    env.save("models/optimized_drone_env.pkl")
    
    print("✅ 최적화된 모델 훈련 완료!")
    return model, env

def benchmark_training():
    """훈련 성능 벤치마크"""
    import time
    
    print("📊 훈련 성능 벤치마크")
    print("=" * 40)
    
    start_time = time.time()
    
    # 간단한 테스트 훈련
    env = create_optimized_env()
    model = PPO("MlpPolicy", env, verbose=0)
    model.learn(total_timesteps=1000)
    
    training_time = time.time() - start_time
    
    print(f"⚡ 1000 스텝 훈련 시간: {training_time:.2f}초")
    print(f"🏃 초당 스텝 수: {1000/training_time:.1f} steps/sec")
    
    # 메모리 사용량 체크
    import psutil
    process = psutil.Process()
    memory_mb = process.memory_info().rss / 1024 / 1024
    print(f"💾 메모리 사용량: {memory_mb:.1f}MB")

if __name__ == "__main__":
    # 벤치마크 실행
    benchmark_training()
    
    # 실제 훈련 (옵션)
    # train_optimized_model() 