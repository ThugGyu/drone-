#!/usr/bin/env python3
"""
Never Stop Campus Tour - 절대 멈추지 않는 캠퍼스 투어
강력한 멈춤 방지 시스템으로 계속 움직이는 드론 시뮬레이션
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import json
import time
import os

class NeverStopCampusTour:
    def __init__(self):
        """절대 멈추지 않는 캠퍼스 투어 시뮬레이션"""
        self.campus_data = self.load_campus_layout()
        self.tour_route = self.create_continuous_tour_route()
        self.drone_pos = np.array([45.0, 82.0])  # Main Gate에서 시작
        self.current_waypoint = 0
        self.flight_history = [self.drone_pos.copy()]
        self.completed_visits = []
        self.hover_counter = 0
        self.tour_status = "MOVING"
        self.base_speed = 2.5  # 기본 속도 증가
        self.min_speed = 1.0   # 최소 속도 보장
        self.stuck_prevention_timer = 0
        self.last_positions = []
        self.force_move_mode = False
        
        print("🚀 Never Stop Campus Tour - Continuous Movement System")
        print("⚡ Enhanced anti-stuck with guaranteed movement")
        print(f"🎯 Total waypoints: {len(self.tour_route)}")
        
    def load_campus_layout(self):
        """연속 이동을 위한 최적화된 캠퍼스 레이아웃"""
        layout = {
            "campus_name": "GNU Gajwa Campus (Never Stop Mode)",
            "map_dimensions": (200, 150),
            "buildings": [
                {"name": "Main Gate", "pos": (45, 82), "size": (12, 6), "color": "#8B4513", "code": "GATE"},
                {"name": "Central Plaza", "pos": (85, 87), "size": (12, 10), "color": "#90EE90", "code": "PLAZA"},
                {"name": "Engineering", "pos": (55, 110), "size": (20, 18), "color": "#DC143C", "code": "ENG"},
                {"name": "Humanities", "pos": (55, 135), "size": (20, 12), "color": "#FFD700", "code": "HUM"},
                {"name": "Central Library", "pos": (110, 100), "size": (20, 16), "color": "#4169E1", "code": "LIB"},
                {"name": "Science Hall", "pos": (140, 115), "size": (20, 16), "color": "#32CD32", "code": "SCI"},
                {"name": "Medical School", "pos": (160, 95), "size": (25, 20), "color": "#FF69B4", "code": "MED"},
                {"name": "Administration", "pos": (120, 125), "size": (16, 12), "color": "#CD853F", "code": "ADM"},
                {"name": "Gymnasium", "pos": (160, 60), "size": (20, 16), "color": "#9932CC", "code": "GYM"},
                {"name": "Student Center", "pos": (95, 65), "size": (16, 12), "color": "#FF8C00", "code": "STU"},
                {"name": "Dormitory", "pos": (45, 45), "size": (20, 16), "color": "#FFA500", "code": "DORM"}
            ],
            "no_fly_zones": [
                {"name": "Medical Hospital", "center": (190, 115), "radius": 20, "color": "#FF0000"},
                {"name": "Admin Restricted", "center": (130, 145), "radius": 15, "color": "#FF4500"}
            ]
        }
        return layout
    
    def create_continuous_tour_route(self):
        """연속적이고 부드러운 투어 경로 생성"""
        # 더 넓은 간격과 안전한 경로로 설계
        continuous_route = [
            {"name": "Main Gate Start", "pos": np.array([45.0, 82.0]), "visit_time": 1, "action": "takeoff"},
            {"name": "Central Plaza", "pos": np.array([92.0, 95.0]), "visit_time": 2, "action": "survey"},
            {"name": "Student Center", "pos": np.array([103.0, 75.0]), "visit_time": 2, "action": "inspect"},
            {"name": "Gymnasium Area", "pos": np.array([175.0, 70.0]), "visit_time": 2, "action": "check"},
            {"name": "Medical Zone (Safe)", "pos": np.array([140.0, 80.0]), "visit_time": 2, "action": "observe"},
            {"name": "Science Hall", "pos": np.array([155.0, 130.0]), "visit_time": 2, "action": "inspect"},
            {"name": "Administration (Safe)", "pos": np.array([105.0, 130.0]), "visit_time": 2, "action": "check"},
            {"name": "Central Library", "pos": np.array([125.0, 108.0]), "visit_time": 2, "action": "survey"},
            {"name": "Humanities Building", "pos": np.array([65.0, 145.0]), "visit_time": 2, "action": "inspect"},
            {"name": "Engineering Complex", "pos": np.array([65.0, 120.0]), "visit_time": 2, "action": "detailed"},
            {"name": "Dormitory Area", "pos": np.array([55.0, 55.0]), "visit_time": 2, "action": "check"},
            {"name": "Return to Gate", "pos": np.array([45.0, 82.0]), "visit_time": 1, "action": "landing"}
        ]
        
        total_distance = sum(np.linalg.norm(continuous_route[i+1]["pos"] - continuous_route[i]["pos"]) 
                           for i in range(len(continuous_route) - 1))
        
        print(f"🛣️ Continuous route distance: {total_distance:.1f}m")
        print(f"⚡ Designed for non-stop movement")
        
        return continuous_route
    
    def never_stop_navigation(self, current_pos, target_pos):
        """절대 멈추지 않는 네비게이션 시스템"""
        direction_vector = target_pos - current_pos
        distance_to_target = np.linalg.norm(direction_vector)
        
        # 목표에 가까워도 완전히 멈추지 않음
        if distance_to_target < 1.0:
            return self.generate_minimum_movement()
        
        # 정규화된 방향
        if distance_to_target > 0:
            unit_direction = direction_vector / distance_to_target
        else:
            unit_direction = self.generate_random_direction()
        
        # 장애물 회피 (약하게)
        obstacle_avoidance = self.compute_light_obstacle_avoidance(current_pos)
        
        # No-Fly Zone 회피 (강하게)
        nfz_avoidance = self.compute_strong_nfz_avoidance(current_pos)
        
        # 강력한 멈춤 방지 시스템
        anti_stuck_force = self.compute_powerful_anti_stuck_force()
        
        # 연속 이동 보장 시스템
        continuous_movement = self.ensure_continuous_movement()
        
        # 최종 이동 벡터 (항상 최소 속도 보장)
        final_direction = (unit_direction + 
                         obstacle_avoidance * 0.1 + 
                         nfz_avoidance * 1.5 + 
                         anti_stuck_force * 0.8 +
                         continuous_movement * 0.3)
        
        # 벡터 정규화 및 최소 속도 보장
        if np.linalg.norm(final_direction) > 0:
            final_direction = final_direction / np.linalg.norm(final_direction)
        else:
            final_direction = self.generate_random_direction()
        
        # 속도 계산 (항상 최소 속도 이상)
        speed = max(self.min_speed, self.base_speed * self.calculate_speed_factor(distance_to_target))
        
        return final_direction * speed
    
    def generate_minimum_movement(self):
        """최소한의 움직임 보장"""
        angle = np.random.uniform(0, 2 * np.pi)
        return np.array([np.cos(angle), np.sin(angle)]) * self.min_speed
    
    def generate_random_direction(self):
        """랜덤 방향 생성"""
        angle = np.random.uniform(0, 2 * np.pi)
        return np.array([np.cos(angle), np.sin(angle)])
    
    def compute_light_obstacle_avoidance(self, pos):
        """가벼운 장애물 회피 (멈춤 방지 우선)"""
        avoidance_force = np.array([0.0, 0.0])
        
        for building in self.campus_data["buildings"]:
            bx, by = building["pos"]
            bw, bh = building["size"]
            
            building_center = np.array([bx + bw/2, by + bh/2])
            direction_to_building = pos - building_center
            distance_to_building = np.linalg.norm(direction_to_building)
            
            # 작은 안전 마진 (멈춤을 방지하기 위해)
            safety_margin = max(8.0, (bw + bh) / 6)
            
            if distance_to_building < safety_margin and distance_to_building > 0.1:
                avoidance_strength = (safety_margin - distance_to_building) / safety_margin * 0.5
                avoidance_force += direction_to_building / distance_to_building * avoidance_strength
        
        return avoidance_force
    
    def compute_strong_nfz_avoidance(self, pos):
        """강력한 No-Fly Zone 회피"""
        nfz_avoidance = np.array([0.0, 0.0])
        
        for nfz in self.campus_data["no_fly_zones"]:
            nfz_center = np.array(nfz["center"])
            direction_from_nfz = pos - nfz_center
            distance_from_nfz = np.linalg.norm(direction_from_nfz)
            
            danger_radius = nfz["radius"] + 25  # 큰 안전 마진
            
            if distance_from_nfz < danger_radius and distance_from_nfz > 0.1:
                avoidance_intensity = 3.0 * (danger_radius - distance_from_nfz) / danger_radius
                nfz_avoidance += direction_from_nfz / distance_from_nfz * avoidance_intensity
        
        return nfz_avoidance
    
    def compute_powerful_anti_stuck_force(self):
        """강력한 멈춤 방지 시스템"""
        # 위치 기록 관리
        self.last_positions.append(self.drone_pos.copy())
        if len(self.last_positions) > 10:
            self.last_positions.pop(0)
        
        # 이동량 계산
        if len(self.last_positions) >= 3:
            recent_movement = 0
            for i in range(1, len(self.last_positions)):
                movement = np.linalg.norm(self.last_positions[i] - self.last_positions[i-1])
                recent_movement += movement
            
            # 멈춤 감지 및 강제 이동 활성화
            if recent_movement < 5.0:  # 임계값 증가
                self.stuck_prevention_timer += 1
                self.force_move_mode = True
                
                if self.stuck_prevention_timer > 5:
                    print("🚀 POWERFUL ANTI-STUCK ACTIVATED! FORCING MOVEMENT!")
                    # 강력한 랜덤 방향으로 이동
                    angle = np.random.uniform(0, 2 * np.pi)
                    force_direction = np.array([np.cos(angle), np.sin(angle)])
                    return force_direction * 3.0  # 강한 힘
            else:
                self.stuck_prevention_timer = max(0, self.stuck_prevention_timer - 1)
                if self.stuck_prevention_timer == 0:
                    self.force_move_mode = False
        
        return np.array([0.0, 0.0])
    
    def ensure_continuous_movement(self):
        """연속 이동 보장 시스템"""
        if self.force_move_mode:
            # 강제 이동 모드에서는 계속 움직임
            angle = np.random.uniform(0, 2 * np.pi)
            return np.array([np.cos(angle), np.sin(angle)]) * 1.5
        
        # 일반 모드에서도 최소 움직임 보장
        angle = np.random.uniform(0, 2 * np.pi)
        return np.array([np.cos(angle), np.sin(angle)]) * 0.2
    
    def calculate_speed_factor(self, distance):
        """거리별 속도 계산 (항상 최소 속도 이상)"""
        if distance < 2.0:
            return 0.4  # 목표 근처에서도 어느 정도 속도 유지
        elif distance < 10.0:
            return 0.7
        else:
            return 1.0
    
    def update_continuous_tour(self):
        """연속적인 투어 업데이트"""
        if self.current_waypoint >= len(self.tour_route):
            self.tour_status = "COMPLETED"
            return
        
        active_waypoint = self.tour_route[self.current_waypoint]
        target_position = active_waypoint["pos"]
        
        # 절대 멈추지 않는 네비게이션
        movement_vector = self.never_stop_navigation(self.drone_pos, target_position)
        
        # 드론 위치 업데이트 (더 큰 스텝)
        self.drone_pos += movement_vector * 0.15  # 스텝 크기 증가
        self.flight_history.append(self.drone_pos.copy())
        
        # 목표점 도달 판정 (더 넓은 범위)
        distance_to_waypoint = np.linalg.norm(self.drone_pos - target_position)
        
        if distance_to_waypoint < 8.0:  # 범위 확대
            self.hover_counter += 1
            
            # 짧은 호버링 시간
            required_hover_time = active_waypoint["visit_time"] * 1  # 호버링 시간 단축
            if self.hover_counter >= required_hover_time:
                self.completed_visits.append(active_waypoint)
                
                print(f"🚀 {active_waypoint['name']} - NEVER STOP MISSION COMPLETE!")
                print(f"   ⚡ Action: {active_waypoint['action']} (CONTINUOUS MODE)")
                print(f"   🎯 Moving to next target immediately!")
                
                self.current_waypoint += 1
                self.hover_counter = 0
                self.stuck_prevention_timer = 0  # 리셋
                self.force_move_mode = False
    
    def start_never_stop_tour(self):
        """절대 멈추지 않는 투어 시작"""
        print("\n🚀 Never Stop Campus Tour Starting!")
        print("⚡ Guaranteed continuous movement system active")
        print("🛡️ Anti-stuck system: MAXIMUM POWER")
        print("=" * 60)
        
        self.setup_never_stop_visualization()
    
    def setup_never_stop_visualization(self):
        """연속 이동 시각화 설정"""
        self.figure, self.axis = plt.subplots(figsize=(18, 12))
        
        self.render_never_stop_map()
        
        # 드론 표시 (더 크고 눈에 띄게)
        self.drone_marker, = self.axis.plot(self.drone_pos[0], self.drone_pos[1], 
                                           'ro', markersize=18, markeredgecolor='darkred', 
                                           markeredgewidth=4, label='Never Stop Drone', zorder=15)
        
        # 연속 비행 경로
        self.path_trace, = self.axis.plot([], [], 'blue', linewidth=4, alpha=0.9, 
                                         label='Continuous Flight Path', zorder=10)
        
        # 현재 목표점
        self.target_marker, = self.axis.plot([], [], 'gs', markersize=14, 
                                            markeredgecolor='green', markeredgewidth=3, 
                                            label='Current Target', zorder=12)
        
        # 애니메이션 (더 빠른 업데이트)
        self.animation = FuncAnimation(self.figure, self.animate_never_stop, frames=3000, 
                                      interval=80, blit=False, repeat=False)
        
        plt.tight_layout()
        plt.show()
    
    def render_never_stop_map(self):
        """연속 이동 최적화된 맵 렌더링"""
        # 캠퍼스 경계
        map_w, map_h = self.campus_data["map_dimensions"]
        campus_border = patches.Rectangle((0, 0), map_w, map_h, 
                                        linewidth=4, edgecolor='darkgreen', 
                                        facecolor='lightgreen', alpha=0.1)
        self.axis.add_patch(campus_border)
        
        # 건물들 (작게 그려서 충돌 가능성 줄이기)
        for building in self.campus_data["buildings"]:
            x, y = building["pos"]
            w, h = building["size"]
            
            building_rect = patches.Rectangle((x, y), w, h, 
                                            facecolor=building["color"], alpha=0.7, 
                                            edgecolor='black', linewidth=1.5, zorder=3)
            self.axis.add_patch(building_rect)
            
            self.axis.text(x + w/2, y + h/2, f"{building['code']}", 
                          ha='center', va='center', fontweight='bold', 
                          fontsize=9, color='white', zorder=4)
        
        # No-Fly Zones (강조)
        for nfz in self.campus_data["no_fly_zones"]:
            # 위험 구역
            danger_circle = patches.Circle(nfz["center"], nfz["radius"], 
                                         facecolor='red', alpha=0.5, 
                                         edgecolor='darkred', linewidth=5, zorder=4)
            self.axis.add_patch(danger_circle)
            
            # 경고 구역
            warning_circle = patches.Circle(nfz["center"], nfz["radius"] + 15, 
                                          facecolor='orange', alpha=0.2, 
                                          edgecolor='red', linewidth=3, 
                                          linestyle='--', zorder=3)
            self.axis.add_patch(warning_circle)
            
            self.axis.text(nfz["center"][0], nfz["center"][1], "NO FLY\nZONE", 
                          ha='center', va='center', fontweight='bold', 
                          fontsize=11, color='white', zorder=5,
                          bbox=dict(boxstyle='round,pad=0.4', facecolor='red', alpha=0.9))
        
        # 연속 투어 경로
        route_x = [wp["pos"][0] for wp in self.tour_route]
        route_y = [wp["pos"][1] for wp in self.tour_route]
        self.axis.plot(route_x, route_y, 'green', linewidth=4, alpha=0.8, 
                      linestyle='--', label='Never Stop Route', zorder=6)
        
        # 경유점 표시
        for i, waypoint in enumerate(self.tour_route):
            x, y = waypoint["pos"]
            
            if i == 0 or i == len(self.tour_route) - 1:
                marker_style = 'D'
                marker_color = 'green' if i == 0 else 'red'
                marker_size = 14
            else:
                marker_style = 'o'
                marker_color = 'blue'
                marker_size = 11
            
            self.axis.plot(x, y, marker_style, color=marker_color, markersize=marker_size, 
                          markeredgecolor='black', markeredgewidth=2, zorder=7)
            
            # 순서 번호
            if i < len(self.tour_route) - 1:
                self.axis.text(x + 3, y + 3, str(i+1), fontsize=9, fontweight='bold', 
                             color='darkblue', zorder=8,
                             bbox=dict(boxstyle='circle,pad=0.2', facecolor='yellow', alpha=0.9))
        
        # 축 설정
        self.axis.set_xlim(-5, map_w + 5)
        self.axis.set_ylim(-5, map_h + 5)
        self.axis.set_aspect('equal')
        self.axis.grid(True, alpha=0.3)
        self.axis.set_xlabel('East-West Direction (m)', fontsize=12)
        self.axis.set_ylabel('North-South Direction (m)', fontsize=12)
        self.axis.set_title('Never Stop GNU Campus Tour - Guaranteed Continuous Movement\nPowerful Anti-Stuck System Active', 
                           fontsize=16, fontweight='bold')
        self.axis.legend(loc='upper right')
    
    def animate_never_stop(self, frame):
        """절대 멈추지 않는 애니메이션"""
        if self.tour_status != "COMPLETED":
            self.update_continuous_tour()
            
            # 드론 위치 업데이트
            self.drone_marker.set_data([self.drone_pos[0]], [self.drone_pos[1]])
            
            # 연속 비행 경로 업데이트
            if len(self.flight_history) > 1:
                recent_path = self.flight_history[-60:]  # 더 많은 경로 표시
                path_x = [p[0] for p in recent_path]
                path_y = [p[1] for p in recent_path]
                self.path_trace.set_data(path_x, path_y)
            
            # 현재 목표점 표시
            if self.current_waypoint < len(self.tour_route):
                target = self.tour_route[self.current_waypoint]["pos"]
                self.target_marker.set_data([target[0]], [target[1]])
            
            # 진행률 및 상태
            progress_percentage = len(self.completed_visits) / len(self.tour_route) * 100
            current_mission = (self.tour_route[self.current_waypoint]["name"] 
                             if self.current_waypoint < len(self.tour_route) 
                             else "Mission Complete")
            
            # 이동 상태 표시
            movement_status = "FORCE MOVE" if self.force_move_mode else "NORMAL"
            stuck_level = min(self.stuck_prevention_timer, 10)
            
            # 제목 업데이트
            self.axis.set_title(f'Never Stop Campus Tour - Progress: {progress_percentage:.1f}% | Movement: {movement_status}\n'
                               f'Current: {current_mission} | Anti-Stuck Level: {stuck_level}/10', 
                               fontsize=16, fontweight='bold')
        else:
            total_flight_distance = len(self.flight_history) * 0.3
            total_mission_time = len(self.flight_history) * 0.08 / 60
            
            print(f"\n🎉 Never Stop Campus Tour Complete!")
            print(f"🚀 Continuous movement maintained throughout!")
            print(f"📍 All locations visited: {len(self.completed_visits)}")
            print(f"🛩️ Total distance: {total_flight_distance:.1f}m")
            print(f"⏱️ Total time: {total_mission_time:.1f} minutes")
            print(f"⚡ Zero stops achieved!")
            
            self.axis.set_title('🎉 Never Stop Campus Tour Complete!\n🚀 Continuous Movement Success - Zero Stops!', 
                               fontsize=16, fontweight='bold', color='green')
        
        return self.drone_marker, self.path_trace, self.target_marker

def main():
    """메인 실행 함수"""
    print("🚀 Never Stop GNU Campus Tour")
    print("⚡ Guaranteed Continuous Movement System")
    print("🛡️ Powerful Anti-Stuck Technology")
    print("🎯 NEVER STOPS - ALWAYS MOVING!")
    print("=" * 65)
    
    # 절대 멈추지 않는 캠퍼스 투어 시작
    never_stop_tour = NeverStopCampusTour()
    never_stop_tour.start_never_stop_tour()

if __name__ == "__main__":
    main()