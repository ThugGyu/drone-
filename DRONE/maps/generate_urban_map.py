#!/usr/bin/env python3
"""
Urban City Map Generator
Creates a realistic urban environment map for drone simulation
"""

import numpy as np
import cv2
import random
from PIL import Image

class UrbanMapGenerator:
    def __init__(self, width=2000, height=2000, resolution=0.05):
        """
        Initialize urban map generator
        Args:
            width: Map width in pixels
            height: Map height in pixels  
            resolution: Meters per pixel (0.05 = 5cm per pixel)
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.map_data = np.full((height, width), 255, dtype=np.uint8)  # Start with free space
        
        # Color codes for different areas
        self.OCCUPIED = 0      # Black - buildings/obstacles
        self.FREE = 255        # White - free space
        self.UNKNOWN = 128     # Gray - unknown areas
        
    def add_building_block(self, x, y, width, height, building_type='normal'):
        """Add a building block to the map"""
        x1, y1 = max(0, x), max(0, y)
        x2, y2 = min(self.width, x + width), min(self.height, y + height)
        
        if building_type == 'highrise':
            # High-rise buildings (completely solid)
            self.map_data[y1:y2, x1:x2] = self.OCCUPIED
        else:
            # Regular buildings with some internal structure
            self.map_data[y1:y2, x1:x2] = self.OCCUPIED
            # Add some internal courtyards for larger buildings
            if width > 100 and height > 100:
                courtyard_x = x1 + width//3
                courtyard_y = y1 + height//3
                courtyard_w = width//3
                courtyard_h = height//3
                self.map_data[courtyard_y:courtyard_y+courtyard_h, 
                             courtyard_x:courtyard_x+courtyard_w] = self.FREE
    
    def add_road(self, x1, y1, x2, y2, width=40):
        """Add a road between two points"""
        # Calculate road direction and create perpendicular width
        dx = x2 - x1
        dy = y2 - y1
        length = np.sqrt(dx*dx + dy*dy)
        
        if length == 0:
            return
            
        # Normalize direction
        dx_norm = dx / length
        dy_norm = dy / length
        
        # Perpendicular direction for road width
        perp_x = -dy_norm * width // 2
        perp_y = dx_norm * width // 2
        
        # Create road polygon
        points = np.array([
            [x1 + perp_x, y1 + perp_y],
            [x1 - perp_x, y1 - perp_y],
            [x2 - perp_x, y2 - perp_y],
            [x2 + perp_x, y2 + perp_y]
        ], dtype=np.int32)
        
        cv2.fillPoly(self.map_data, [points], self.FREE)
    
    def add_intersection(self, x, y, size=60):
        """Add a road intersection"""
        x1, y1 = max(0, x - size//2), max(0, y - size//2)
        x2, y2 = min(self.width, x + size//2), min(self.height, y + size//2)
        self.map_data[y1:y2, x1:x2] = self.FREE
    
    def add_park(self, x, y, width, height):
        """Add a park area (partially free space with some trees)"""
        x1, y1 = max(0, x), max(0, y)
        x2, y2 = min(self.width, x + width), min(self.height, y + height)
        
        # Park is mostly free space
        self.map_data[y1:y2, x1:x2] = self.FREE
        
        # Add some trees/obstacles randomly in the park
        for _ in range((width * height) // 5000):  # Tree density
            tree_x = random.randint(x1 + 10, x2 - 10)
            tree_y = random.randint(y1 + 10, y2 - 10)
            tree_size = random.randint(8, 15)
            cv2.circle(self.map_data, (tree_x, tree_y), tree_size, self.OCCUPIED, -1)
    
    def add_parking_lot(self, x, y, width, height):
        """Add a parking lot with some cars"""
        x1, y1 = max(0, x), max(0, y)
        x2, y2 = min(self.width, x + width), min(self.height, y + height)
        
        # Parking lot is free space
        self.map_data[y1:y2, x1:x2] = self.FREE
        
        # Add parked cars
        car_width, car_height = 8, 4  # Car size in pixels
        for car_y in range(y1 + 10, y2 - 10, 20):
            for car_x in range(x1 + 10, x2 - 10, 15):
                if random.random() < 0.6:  # 60% occupancy
                    self.map_data[car_y:car_y+car_height, car_x:car_x+car_width] = self.OCCUPIED
    
    def add_landing_zone(self, x, y, size=30):
        """Add a designated landing zone"""
        x1, y1 = max(0, x - size//2), max(0, y - size//2)
        x2, y2 = min(self.width, x + size//2), min(self.height, y + size//2)
        
        # Clear the area
        self.map_data[y1:y2, x1:x2] = self.FREE
        
        # Add landing pad border
        cv2.rectangle(self.map_data, (x1, y1), (x2, y2), self.OCCUPIED, 2)
        
        # Add cross marker in center
        cv2.line(self.map_data, (x-10, y), (x+10, y), self.OCCUPIED, 2)
        cv2.line(self.map_data, (x, y-10), (x, y+10), self.OCCUPIED, 2)
    
    def generate_urban_layout(self):
        """Generate a realistic urban layout"""
        print("Generating urban city layout...")
        
        # 1. Create main road grid
        print("Adding road network...")
        
        # Vertical main roads
        for x in [400, 800, 1200, 1600]:
            self.add_road(x, 0, x, self.height, 50)  # Main roads
        
        # Horizontal main roads  
        for y in [400, 800, 1200, 1600]:
            self.add_road(0, y, self.width, y, 50)
            
        # Secondary roads
        for x in [200, 600, 1000, 1400, 1800]:
            self.add_road(x, 0, x, self.height, 30)
            
        for y in [200, 600, 1000, 1400, 1800]:
            self.add_road(0, y, self.width, y, 30)
        
        # Add intersections
        print("Adding intersections...")
        road_positions = [200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800]
        for x in road_positions:
            for y in road_positions:
                self.add_intersection(x, y, 80)
        
        # 2. Add downtown high-rise district
        print("Adding downtown district...")
        for x in range(300, 700, 80):
            for y in range(300, 700, 80):
                if random.random() < 0.8:  # 80% building density
                    building_w = random.randint(60, 120)
                    building_h = random.randint(60, 120)
                    self.add_building_block(x, y, building_w, building_h, 'highrise')
        
        # 3. Add residential areas
        print("Adding residential areas...")
        # Top-left residential
        for x in range(50, 350, 40):
            for y in range(50, 350, 40):
                if random.random() < 0.6:
                    building_w = random.randint(25, 50)
                    building_h = random.randint(25, 50)
                    self.add_building_block(x, y, building_w, building_h)
        
        # Top-right residential
        for x in range(1650, 1950, 40):
            for y in range(50, 350, 40):
                if random.random() < 0.6:
                    building_w = random.randint(25, 50)
                    building_h = random.randint(25, 50)
                    self.add_building_block(x, y, building_w, building_h)
        
        # Bottom areas
        for x in range(50, 1950, 60):
            for y in range(1650, 1950, 60):
                if random.random() < 0.4:
                    building_w = random.randint(30, 80)
                    building_h = random.randint(30, 80)
                    self.add_building_block(x, y, building_w, building_h)
        
        # 4. Add commercial/industrial areas
        print("Adding commercial areas...")
        for x in range(900, 1500, 100):
            for y in range(50, 300, 100):
                if random.random() < 0.7:
                    building_w = random.randint(80, 150)
                    building_h = random.randint(60, 100)
                    self.add_building_block(x, y, building_w, building_h)
        
        # 5. Add parks and green spaces
        print("Adding parks...")
        # Central park
        self.add_park(850, 850, 300, 300)
        
        # Smaller parks
        self.add_park(50, 450, 120, 120)
        self.add_park(1650, 450, 120, 120)
        self.add_park(450, 1650, 120, 120)
        
        # 6. Add parking lots
        print("Adding parking areas...")
        self.add_parking_lot(750, 300, 100, 80)
        self.add_parking_lot(300, 750, 80, 100)
        self.add_parking_lot(1300, 1300, 120, 100)
        self.add_parking_lot(1500, 500, 100, 80)
        
        # 7. Add landing zones
        print("Adding landing zones...")
        # Hospital landing pad
        self.add_landing_zone(1000, 1000, 40)
        
        # Emergency landing zones
        self.add_landing_zone(100, 100, 30)
        self.add_landing_zone(1900, 100, 30)
        self.add_landing_zone(100, 1900, 30)
        self.add_landing_zone(1900, 1900, 30)
        
        # Police station landing
        self.add_landing_zone(500, 500, 35)
        
        print("Urban layout generation completed!")
    
    def save_pgm(self, filename):
        """Save map as PGM file"""
        print(f"Saving map as {filename}...")
        
        # Create PIL image and save as PGM
        img = Image.fromarray(self.map_data, mode='L')
        img.save(filename)
        
        print(f"Map saved successfully: {filename}")
        print(f"Map size: {self.width}x{self.height} pixels")
        print(f"Real world size: {self.width*self.resolution:.1f}m x {self.height*self.resolution:.1f}m")
        print(f"Resolution: {self.resolution*100:.1f}cm per pixel")

def main():
    """Generate urban city map"""
    print("=== Urban City Map Generator ===")
    
    # Create map generator
    generator = UrbanMapGenerator(width=2000, height=2000, resolution=0.05)
    
    # Generate urban layout
    generator.generate_urban_layout()
    
    # Save map
    generator.save_pgm("urban_city.pgm")
    
    print("\n=== Generation Complete ===")
    print("Files created:")
    print("- urban_city.pgm (map data)")
    print("- urban_city.yaml (map configuration)")
    print("- urban_city_preview.txt (map description)")
    print("\nTo use this map:")
    print("1. Copy files to your ROS2 maps directory")
    print("2. Update your launch file to use 'urban_city.yaml'")
    print("3. Launch your navigation stack")

if __name__ == "__main__":
    main() 