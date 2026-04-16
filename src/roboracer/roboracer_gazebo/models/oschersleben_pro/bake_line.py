
import numpy as np
from PIL import Image, ImageDraw
import os

def paint_line_on_map(image_path, yaml_info, csv_path, output_path):
    # Load image
    img = Image.open(image_path).convert('L')
    width, height = img.size
    
    # We need to draw on it
    draw = ImageDraw.Draw(img)
    
    # Load CSV points
    data = np.genfromtxt(csv_path, delimiter=',', skip_header=1)
    points = data[:, :2] # x,y in meters
    
    resolution = yaml_info['resolution']
    origin = yaml_info['origin']
    
    # Convert meters to pixels
    pixel_points = []
    for p in points:
        px = (p[0] - origin[0]) / resolution
        py = height - (p[1] - origin[1]) / resolution
        pixel_points.append((px, py))
    
    # Draw the line as GRAY (value 180)
    # Width 5 pixels approx 20cm at 0.04 resolution
    draw.line(pixel_points, fill=180, width=5)
    
    img.save(output_path)
    print(f"Painted map saved to {output_path}")

if __name__ == "__main__":
    base = "/home/alfonsd/Documents/Assesment-Auto/src/roboracer/roboracer_gazebo/models/oschersleben_pro"
    yaml_data = {
        'resolution': 0.04295,
        'origin': [-55.07650228661655, -33.57884064395765, 0.0]
    }
    paint_line_on_map(
        os.path.join(base, "Oschersleben_map.png"),
        yaml_data,
        os.path.join(base, "Oschersleben_centerline.csv"),
        os.path.join(base, "Oschersleben_map_with_line.png")
    )
