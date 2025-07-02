function generate_urban_map()
% Urban City Map Generator for Drone Navigation
% Creates a realistic urban environment using MATLAB
% Output: PGM format map file for ROS navigation

    % Map parameters
    map_width = 2000;   % pixels (100m at 5cm resolution)
    map_height = 2000;  % pixels
    resolution = 0.05;  % meters per pixel
    
    % Initialize map with free space (white = 255)
    city_map = 255 * ones(map_height, map_width, 'uint8');
    
    fprintf('Generating Urban City Map...\n');
    fprintf('Map size: %dx%d pixels (%.1fm x %.1fm)\n', ...
            map_width, map_height, map_width*resolution, map_height*resolution);
    
    % Add main roads (horizontal and vertical grid)
    city_map = add_road_network(city_map, map_width, map_height);
    
    % Add downtown high-rise district (center)
    city_map = add_downtown_district(city_map, map_width, map_height);
    
    % Add residential areas (corners)
    city_map = add_residential_areas(city_map, map_width, map_height);
    
    % Add parks and green spaces
    city_map = add_parks(city_map, map_width, map_height);
    
    % Add parking lots
    city_map = add_parking_areas(city_map, map_width, map_height);
    
    % Add emergency landing zones
    city_map = add_landing_zones(city_map, map_width, map_height);
    
    % Save as PGM file
    save_pgm_file(city_map, 'urban_city_improved.pgm');
    
    % Create visualization
    create_visualization(city_map);
    
    fprintf('Urban city map generated successfully!\n');
    fprintf('Files created:\n');
    fprintf('- urban_city_improved.pgm (map data)\n');
    fprintf('- urban_city_visualization.png (preview)\n');
end

function map = add_road_network(map, width, height)
    % Add main roads in a grid pattern
    road_width = 40;  % pixels (2m wide roads)
    
    % Vertical roads every 400 pixels (20m apart)
    for x = 200:400:width-200
        x_start = max(1, x - road_width/2);
        x_end = min(width, x + road_width/2);
        map(:, x_start:x_end) = 128;  % Gray for roads
    end
    
    % Horizontal roads every 400 pixels (20m apart)
    for y = 200:400:height-200
        y_start = max(1, y - road_width/2);
        y_end = min(height, y + road_width/2);
        map(y_start:y_end, :) = 128;  % Gray for roads
    end
    
    % Add intersections (darker gray)
    for x = 200:400:width-200
        for y = 200:400:height-200
            x_start = max(1, x - road_width/2);
            x_end = min(width, x + road_width/2);
            y_start = max(1, y - road_width/2);
            y_end = min(height, y + road_width/2);
            map(y_start:y_end, x_start:x_end) = 100;  % Darker gray for intersections
        end
    end
end

function map = add_downtown_district(map, width, height)
    % Add high-rise buildings in the center area
    center_x = width/2;
    center_y = height/2;
    district_size = 600;  % 30m radius
    
    % Create several large buildings
    buildings = [
        center_x-200, center_y-200, 120, 120;  % Building 1
        center_x-50,  center_y-180, 100, 140;  % Building 2
        center_x+100, center_y-150, 80,  100;  % Building 3
        center_x-180, center_y+50,  140, 100;  % Building 4
        center_x+50,  center_y+80,  120, 120;  % Building 5
    ];
    
    for i = 1:size(buildings, 1)
        x = round(buildings(i, 1));
        y = round(buildings(i, 2));
        w = buildings(i, 3);
        h = buildings(i, 4);
        
        x_start = max(1, x);
        x_end = min(width, x + w);
        y_start = max(1, y);
        y_end = min(height, y + h);
        
        map(y_start:y_end, x_start:x_end) = 0;  % Black for buildings
    end
end

function map = add_residential_areas(map, width, height)
    % Add smaller residential buildings in corners
    building_size = 60;  % 3m x 3m buildings
    spacing = 120;       % 6m spacing
    
    % Top-left residential area
    for x = 50:spacing:400
        for y = 50:spacing:400
            if map(y, x) == 255  % Only place on free space
                x_end = min(width, x + building_size);
                y_end = min(height, y + building_size);
                map(y:y_end, x:x_end) = 50;  % Dark gray for residential
            end
        end
    end
    
    % Top-right residential area
    for x = width-400:spacing:width-50
        for y = 50:spacing:400
            if x <= width && y <= height && map(y, x) == 255
                x_end = min(width, x + building_size);
                y_end = min(height, y + building_size);
                map(y:y_end, x:x_end) = 50;
            end
        end
    end
    
    % Bottom areas (similar pattern)
    for x = 50:spacing:400
        for y = height-400:spacing:height-50
            if y <= height && map(y, x) == 255
                x_end = min(width, x + building_size);
                y_end = min(height, y + building_size);
                map(y:y_end, x:x_end) = 50;
            end
        end
    end
    
    for x = width-400:spacing:width-50
        for y = height-400:spacing:height-50
            if x <= width && y <= height && map(y, x) == 255
                x_end = min(width, x + building_size);
                y_end = min(height, y + building_size);
                map(y:y_end, x:x_end) = 50;
            end
        end
    end
end

function map = add_parks(map, width, height)
    % Add park areas (lighter gray for navigation)
    parks = [
        width*0.25, height*0.75, 200, 150;  % Park 1
        width*0.75, height*0.25, 180, 160;  % Park 2
        width*0.15, height*0.15, 120, 100;  % Small park 3
        width*0.85, height*0.85, 140, 120;  % Small park 4
    ];
    
    for i = 1:size(parks, 1)
        x = round(parks(i, 1));
        y = round(parks(i, 2));
        w = parks(i, 3);
        h = parks(i, 4);
        
        x_start = max(1, x);
        x_end = min(width, x + w);
        y_start = max(1, y);
        y_end = min(height, y + h);
        
        % Only place parks on free space
        for py = y_start:y_end
            for px = x_start:x_end
                if map(py, px) == 255
                    map(py, px) = 200;  % Light gray for parks
                end
            end
        end
    end
end

function map = add_parking_areas(map, width, height)
    % Add parking lots near buildings
    parking_areas = [
        width*0.3, height*0.4, 100, 80;
        width*0.7, height*0.6, 120, 90;
        width*0.2, height*0.8, 80, 70;
        width*0.8, height*0.2, 90, 85;
    ];
    
    for i = 1:size(parking_areas, 1)
        x = round(parking_areas(i, 1));
        y = round(parking_areas(i, 2));
        w = parking_areas(i, 3);
        h = parking_areas(i, 4);
        
        x_start = max(1, x);
        x_end = min(width, x + w);
        y_start = max(1, y);
        y_end = min(height, y + h);
        
        for py = y_start:y_end
            for px = x_start:x_end
                if map(py, px) == 255
                    map(py, px) = 180;  % Medium gray for parking
                end
            end
        end
    end
end

function map = add_landing_zones(map, width, height)
    % Add emergency landing zones (keep as free space but mark)
    landing_zones = [
        width*0.1, height*0.5, 60, 60;   % Emergency zone 1
        width*0.9, height*0.5, 60, 60;   % Emergency zone 2
        width*0.5, height*0.1, 80, 80;   % Main landing zone
        width*0.5, height*0.9, 80, 80;   % Secondary landing zone
    ];
    
    for i = 1:size(landing_zones, 1)
        x = round(landing_zones(i, 1));
        y = round(landing_zones(i, 2));
        w = landing_zones(i, 3);
        h = landing_zones(i, 4);
        
        x_start = max(1, x);
        x_end = min(width, x + w);
        y_start = max(1, y);
        y_end = min(height, y + h);
        
        % Clear area for landing
        for py = y_start:y_end
            for px = x_start:x_end
                if map(py, px) > 128  % Don't override buildings/roads
                    map(py, px) = 255;  % Keep as free space
                end
            end
        end
        
        % Add border markers
        border_width = 4;
        map(y_start:y_start+border_width, x_start:x_end) = 150;
        map(y_end-border_width:y_end, x_start:x_end) = 150;
        map(y_start:y_end, x_start:x_start+border_width) = 150;
        map(y_start:y_end, x_end-border_width:x_end) = 150;
    end
end

function save_pgm_file(map_data, filename)
    % Save map as PGM file format for ROS
    fid = fopen(filename, 'w');
    if fid == -1
        error('Could not create file %s', filename);
    end
    
    [height, width] = size(map_data);
    
    % Write PGM header
    fprintf(fid, 'P2\n');
    fprintf(fid, '# Urban City Map for Drone Navigation\n');
    fprintf(fid, '# Generated by MATLAB Urban Map Generator\n');
    fprintf(fid, '%d %d\n', width, height);
    fprintf(fid, '255\n');
    
    % Write pixel data
    for y = 1:height
        for x = 1:width
            fprintf(fid, '%d ', map_data(y, x));
        end
        fprintf(fid, '\n');
    end
    
    fclose(fid);
    fprintf('Saved PGM file: %s\n', filename);
end

function create_visualization(map_data)
    % Create and save visualization
    figure('Name', 'Urban City Map', 'Position', [100, 100, 800, 600]);
    
    % Create colormap
    colormap_custom = [
        0, 0, 0;        % Black - Buildings (value 0)
        0.2, 0.2, 0.2;  % Dark gray - Residential (value 50)
        0.4, 0.4, 0.4;  % Gray - Intersections (value 100)
        0.5, 0.5, 0.5;  % Roads (value 128)
        0.6, 0.6, 0.6;  % Landing zone borders (value 150)
        0.7, 0.7, 0.7;  % Parking (value 180)
        0.8, 0.8, 0.8;  % Parks (value 200)
        1, 1, 1;        % White - Free space (value 255)
    ];
    
    imagesc(map_data);
    colormap(gray);
    axis equal;
    axis tight;
    
    title('Urban City Environment Map for Drone Navigation', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X coordinate (pixels)', 'FontSize', 12);
    ylabel('Y coordinate (pixels)', 'FontSize', 12);
    
    % Add legend
    text(100, 100, {'Legend:', 'Black = Buildings', 'Dark Gray = Residential', ...
                    'Gray = Roads', 'Light Gray = Parks', 'White = Free Space'}, ...
         'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black');
    
    % Save visualization
    print('urban_city_visualization.png', '-dpng', '-r300');
    fprintf('Saved visualization: urban_city_visualization.png\n');
end 