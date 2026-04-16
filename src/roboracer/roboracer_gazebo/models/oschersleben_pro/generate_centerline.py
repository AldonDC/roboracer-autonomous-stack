
import numpy as np
import os

def generate_line_obj(csv_path, output_path):
    # Load CSV
    data = np.genfromtxt(csv_path, delimiter=',', skip_header=1)
    # x_m, y_m, w_tr_right_m, w_tr_left_m
    points = data[:, :2]
    
    vertices = []
    faces = []
    
    width = 0.15 # 15cm wide line
    z = 0.005 # 5mm above tracks to avoid flickering (z-fighting)
    
    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i+1]
        
        # Calculate direction and normal (perpendicular)
        dir_v = p2 - p1
        dist = np.linalg.norm(dir_v)
        if dist < 1e-6: continue
        
        dir_v /= dist
        norm_v = np.array([-dir_v[1], dir_v[0]]) # Rotate 90 deg
        
        # 4 vertices for this segment
        v1 = p1 - norm_v * (width/2)
        v2 = p1 + norm_v * (width/2)
        v3 = p2 + norm_v * (width/2)
        v4 = p2 - norm_v * (width/2)
        
        v_idx = len(vertices) + 1
        vertices.extend([
            (v1[0], v1[1], z), (v2[0], v2[1], z), (v3[0], v3[1], z), (v4[0], v4[1], z)
        ])
        
        # 2 triangles
        faces.extend([
            (v_idx, v_idx+1, v_idx+2), (v_idx, v_idx+2, v_idx+3)
        ])

    print(f"Saving Centerline OBJ with {len(vertices)} vertices...")
    with open(output_path, 'w') as f:
        for v in vertices:
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
        f.write("vn 0.0000 0.0000 1.0000\n")
        for face in faces:
            f.write(f"f {face[0]}//1 {face[1]}//1 {face[2]}//1\n")

if __name__ == "__main__":
    base_path = "/home/alfonsd/Documents/Assesment-Auto/src/roboracer/roboracer_gazebo/models/oschersleben_pro"
    csv_p = os.path.join(base_path, "Oschersleben_centerline.csv")
    out_p = os.path.join(base_path, "meshes/centerline.obj")
    generate_line_obj(csv_p, out_p)
    print("Done!")
