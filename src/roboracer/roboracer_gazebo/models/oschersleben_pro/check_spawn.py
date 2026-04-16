
import numpy as np
from PIL import Image

def check_spawn(image_path, yaml_info, world_x, world_y):
    img = Image.open(image_path).convert('L')
    data = np.array(img)
    height, width = data.shape
    resolution = yaml_info['resolution']
    origin = yaml_info['origin']
    
    px = int((world_x - origin[0]) / resolution)
    py = int(height - (world_y - origin[1]) / resolution)
    
    if 0 <= px < width and 0 <= py < height:
        val = data[py, px]
        print(f"Pixel at world ({world_x}, {world_y}) is {px}, {py} with value {val}")
        if val > 128:
            print("SAFE: This is free space.")
        else:
            print("WARNING: This is a WALL.")
            # Find closest free pixel
            for r in range(1, 100):
                for dy in range(-r, r+1):
                    for dx in range(-r, r+1):
                        ny, nx = py+dy, px+dx
                        if 0 <= nx < width and 0 <= ny < height:
                            if data[ny, nx] > 128:
                                nx_w = nx * resolution + origin[0]
                                ny_w = (height - ny) * resolution + origin[1]
                                print(f"Suggested safe spawn: world ({nx_w:.3f}, {ny_w:.3f})")
                                return
    else:
        print("World coordinates are outside the map bounds!")

if __name__ == "__main__":
    yaml_data = {
        'resolution': 0.04295,
        'origin': [-55.07650228661655, -33.57884064395765, 0.0]
    }
    check_spawn("src/roboracer/roboracer_gazebo/models/oschersleben_pro/Oschersleben_map.png", yaml_data, 0, 0)
