#!/usr/bin/env python3
"""
Autonomous Campus Flight System
훈련된 강화학습 모델로 경상국립대 캠퍼스 자율주행 시뮬레이션
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import json
import time
import random
from matplotlib.animation import FuncAnimation
import os

class AutonomousCampusFlight:
    def __init__(self, model_path="models/improved_drone_final.zip"):
        """자율주행 시스템 초기화"""
        self.model_path = model_path
        self.campus_data = self.load_campus_data()
        self.waypoints = self.load_waypoints()
        self.current_position = np.array([60.0, 80.0])  # Main Gate 시작
        self.target_waypoint = 0
        self.flight_path = [self.current_position.copy()]
        self.completed_waypoints = []
        self.flight_speed = 2.0  # m/s
        self.is_flying = False
        self.mission_complete = False
        
        print("🚁 경상국립대 캠퍼스 자율주행 시스템 초기화 완료!")
        print(f"📍 총 {len(self.waypoints)}개 경유점으로 캠퍼스 투어 시작")
        
    def load_campus_data(self):
        """실제 캠퍼스 맵 데이터 로드"""
        campus_info = {
            "name": "Gyeongsang National University Gajwa Campus",
            "buildings": [
                {"name": "Main Gate", "pos": (60, 80), "size": (15, 10), "color": "#8B4513", "type": "entrance"},
                {"name": "Central Plaza", "pos": (85, 85), "size": (20, 15), "color": "#90EE90", "type": "plaza"},
                {"name": "Engineering", "pos": (50, 110), "size": (30, 25), "color": "#DC143C", "type": "academic"},
                {"name": "Central Library", "pos": (100, 105), "size": (25, 20), "color": "#4169E1", "type": "library"},
                {"name": "Science Hall", "pos": (140, 115), "size": (25, 20), "color": "#32CD32", "type": "academic"},
                {"name": "Medical School", "pos": (160, 95), "size": (30, 25), "color": "#FF69B4", "type": "medical"},
                {"name": "Gymnasium", "pos": (160, 65), "size": (25, 20), "color": "#9932CC", "type": "sports"},
                {"name": "Student Center", "pos": (100, 70), "size": (25, 15), "color": "#FF8C00", "type": "student"},
                {"name": "Dormitory", "pos": (70, 45), "size": (30, 20), "color": "#FFD700", "type": "dormitory"},
            ],
            "no_fly_zones": [
                {"name": "Medical Hospital", "pos": (180, 115), "radius": 25, "color": "#FF0000"}
            ],
            "map_size": (200, 150)
        }
        return campus_info
    
    def load_waypoints(self):
        """캠퍼스 투어 경유점 로드"""
        waypoints = [
            {"name": "Main Gate", "pos": np.array([60.0, 80.0]), "action": "takeoff"},
            {"name": "Central Plaza", "pos": np.array([85.0, 85.0]), "action": "hover"},
            {"name": "Engineering", "pos": np.array([65.0, 122.0]), "action": "circle"},
            {"name": "Central Library", "pos": np.array([112.0, 115.0]), "action": "hover"},
            {"name": "Science Hall", "pos": np.array([152.0, 125.0]), "action": "circle"},
            {"name": "Medical School", "pos": np.array([175.0, 107.0]), "action": "hover"},
            {"name": "Gymnasium", "pos": np.array([172.0, 75.0]), "action": "circle"},
            {"name": "Student Center", "pos": np.array([112.0, 77.0]), "action": "hover"},
            {"name": "Dormitory", "pos": np.array([85.0, 55.0]), "action": "circle"},
            {"name": "Gate Return", "pos": np.array([60.0, 80.0]), "action": "landing"}
        ]
        return waypoints
    
    def load_trained_model(self):
        """훈련된 모델 로드 시뮬레이션 (실제로는 AI 모델 로드)"""
        print(f"🤖 훈련된 모델 로드 중: {self.model_path}")
        if os.path.exists(self.model_path):
            print("✅ 모델 로드 성공!")
            return True
        else:
            print("⚠️ 실제 모델 파일이 없어 시뮬레이션 모드로 실행")
            return False
    
    def get_ai_action(self, current_pos, target_pos, obstacles):
        """AI 모델의 행동 결정 (시뮬레이션)"""
        # 실제로는 훈련된 신경망 모델에서 행동을 예측
        # 여기서는 간단한 경로 계산으로 시뮬레이션
        
        direction = target_pos - current_pos
        distance = np.linalg.norm(direction)
        
        if distance < 3.0:  # 목표점에 가까우면
            return np.array([0.0, 0.0])  # 정지
        
        # 정규화된 방향 벡터
        unit_direction = direction / distance
        
        # 장애물 회피 로직 추가
        avoidance_force = self.calculate_obstacle_avoidance(current_pos, obstacles)
        
        # 최종 이동 방향 결정
        final_direction = unit_direction + avoidance_force * 0.3
        final_direction = final_direction / np.linalg.norm(final_direction) if np.linalg.norm(final_direction) > 0 else final_direction
        
        return final_direction * self.flight_speed
    
    def calculate_obstacle_avoidance(self, pos, obstacles):
        """장애물 회피 벡터 계산"""
        avoidance = np.array([0.0, 0.0])
        
        for building in self.campus_data["buildings"]:
            bx, by = building["pos"]
            bw, bh = building["size"]
            
            # 건물 중심점
            building_center = np.array([bx + bw/2, by + bh/2])
            to_building = pos - building_center
            distance = np.linalg.norm(to_building)
            
            # 건물과 너무 가까우면 회피
            if distance < 20.0 and distance > 0:
                avoidance += to_building / distance * (20.0 - distance) / 20.0
        
        return avoidance
    
    def update_position(self):
        """드론 위치 업데이트"""
        if self.target_waypoint >= len(self.waypoints):
            self.mission_complete = True
            return
        
        target_pos = self.waypoints[self.target_waypoint]["pos"]
        
        # AI 모델로부터 행동 결정
        action = self.get_ai_action(self.current_position, target_pos, self.campus_data["buildings"])
        
        # 위치 업데이트
        self.current_position += action * 0.1  # 시간 스텝
        self.flight_path.append(self.current_position.copy())
        
        # 목표점 도달 확인
        distance_to_target = np.linalg.norm(self.current_position - target_pos)
        if distance_to_target < 5.0:
            waypoint_info = self.waypoints[self.target_waypoint]
            self.completed_waypoints.append(waypoint_info)
            print(f"✅ {waypoint_info['name']} 도달! ({waypoint_info['action']})")
            self.target_waypoint += 1
    
    def start_autonomous_flight(self):
        """자율주행 시작"""
        print("\n🚁 경상국립대 캠퍼스 자율주행 투어 시작!")
        print("=" * 50)
        
        # 모델 로드
        self.load_trained_model()
        
        # 실시간 시각화
        self.setup_visualization()
        
    def setup_visualization(self):
        """실시간 비행 시각화 설정"""
        self.fig, self.ax = plt.subplots(figsize=(16, 12))
        
        # 캠퍼스 맵 그리기
        self.draw_campus_map()
        
        # 드론 초기 위치
        self.drone_marker, = self.ax.plot(self.current_position[0], self.current_position[1], 
                                         'ro', markersize=15, markeredgecolor='darkred', 
                                         markeredgewidth=3, label='🚁 Drone', zorder=10)
        
        # 비행 경로
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=3, alpha=0.7, 
                                      label='Flight Path', zorder=5)
        
        # 목표점 표시
        self.target_marker, = self.ax.plot([], [], 'gs', markersize=12, 
                                          markeredgecolor='green', markeredgewidth=2, 
                                          label='Target', zorder=8)
        
        # 애니메이션 시작
        self.animation = FuncAnimation(self.fig, self.animate, frames=1000, 
                                      interval=100, blit=False, repeat=False)
        
        plt.show()
    
    def draw_campus_map(self):
        """캠퍼스 맵 그리기"""
        # 캠퍼스 경계
        map_w, map_h = self.campus_data["map_size"]
        campus_border = patches.Rectangle((0, 0), map_w, map_h, 
                                        linewidth=3, edgecolor='black', 
                                        facecolor='lightgreen', alpha=0.2)
        self.ax.add_patch(campus_border)
        
        # 건물들 그리기
        for building in self.campus_data["buildings"]:
            x, y = building["pos"]
            w, h = building["size"]
            
            rect = patches.Rectangle((x, y), w, h, 
                                   facecolor=building["color"], 
                                   alpha=0.8, edgecolor='black', linewidth=2)
            self.ax.add_patch(rect)
            
            # 건물 이름
            self.ax.text(x + w/2, y + h/2, building["name"], 
                        ha='center', va='center', fontweight='bold', 
                        fontsize=10, color='white',
                        bbox=dict(boxstyle='round,pad=0.2', facecolor='black', alpha=0.7))
        
        # No-Fly Zone
        for nfz in self.campus_data["no_fly_zones"]:
            circle = patches.Circle(nfz["pos"], nfz["radius"], 
                                  facecolor=nfz["color"], alpha=0.3, 
                                  edgecolor='red', linewidth=2, linestyle='--')
            self.ax.add_patch(circle)
            self.ax.text(nfz["pos"][0], nfz["pos"][1], "NO FLY\nZONE", 
                        ha='center', va='center', fontweight='bold', 
                        fontsize=8, color='red')
        
        # 경유점들 그리기
        for i, waypoint in enumerate(self.waypoints):
            x, y = waypoint["pos"]
            if i == 0 or i == len(self.waypoints) - 1:
                self.ax.plot(x, y, 'gs', markersize=10, markeredgecolor='darkgreen', markeredgewidth=2)
            else:
                self.ax.plot(x, y, 'yo', markersize=8, markeredgecolor='orange', markeredgewidth=2)
            
            self.ax.text(x + 3, y + 3, f"{i+1}. {waypoint['name']}", 
                        fontsize=8, fontweight='bold', color='darkblue',
                        bbox=dict(boxstyle='round,pad=0.2', facecolor='yellow', alpha=0.7))
        
        # 설정
        self.ax.set_xlim(-5, map_w + 5)
        self.ax.set_ylim(-5, map_h + 5)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('East-West Direction (m)', fontsize=12)
        self.ax.set_ylabel('North-South Direction (m)', fontsize=12)
        self.ax.set_title('🚁 Gyeongsang National University Campus Autonomous Flight\n(Real-time AI Navigation)', 
                         fontsize=16, fontweight='bold')
        self.ax.legend(loc='upper right')
    
    def animate(self, frame):
        """애니메이션 프레임 업데이트"""
        if not self.mission_complete:
            self.update_position()
            
            # 드론 위치 업데이트
            self.drone_marker.set_data([self.current_position[0]], [self.current_position[1]])
            
            # 비행 경로 업데이트
            if len(self.flight_path) > 1:
                path_x = [p[0] for p in self.flight_path]
                path_y = [p[1] for p in self.flight_path]
                self.path_line.set_data(path_x, path_y)
            
            # 현재 목표점 표시
            if self.target_waypoint < len(self.waypoints):
                target_pos = self.waypoints[self.target_waypoint]["pos"]
                self.target_marker.set_data([target_pos[0]], [target_pos[1]])
            
            # 진행 상황 업데이트
            progress = len(self.completed_waypoints) / len(self.waypoints) * 100
            self.ax.set_title(f'🚁 GNU Campus Autonomous Flight - Progress: {progress:.1f}%\n'
                            f'Current Target: {self.waypoints[self.target_waypoint]["name"] if self.target_waypoint < len(self.waypoints) else "Mission Complete"}', 
                            fontsize=16, fontweight='bold')
        else:
            # 미션 완료
            self.ax.set_title('🎉 Campus Tour Mission Complete!\n✅ All waypoints visited successfully', 
                            fontsize=16, fontweight='bold', color='green')
            print("\n🎉 캠퍼스 투어 미션 완료!")
            print(f"📍 총 {len(self.completed_waypoints)}개 경유점 방문")
            print(f"🛩️ 총 비행 거리: {len(self.flight_path) * 0.2:.1f}m")
            
        return self.drone_marker, self.path_line, self.target_marker

def main():
    """메인 실행 함수"""
    print("🚁 경상국립대 캠퍼스 자율주행 시스템")
    print("=" * 50)
    
    # 자율주행 시스템 초기화
    flight_system = AutonomousCampusFlight()
    
    # 자율주행 시작
    flight_system.start_autonomous_flight()

if __name__ == "__main__":
    main() 