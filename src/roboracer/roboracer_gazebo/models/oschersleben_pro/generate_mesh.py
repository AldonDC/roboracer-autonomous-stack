
import numpy as np
from PIL import Image
import os

def generate_obj(image_path, yaml_info, output_path):
    img = Image.open(image_path).convert('L')

    # DOWNSAMPLE to 50%
    scale = 0.5
    new_w = int(img.width * scale)
    new_h = int(img.height * scale)
    img = img.resize((new_w, new_h), Image.NEAREST)

    data = np.array(img)
    height, width = data.shape
    resolution = yaml_info['resolution'] / scale
    origin = yaml_info['origin']

    is_wall = data < 128
    wall_height = 0.4

    vertices = []
    v_map = {}
    faces = []
    normals = [
        (0,0,-1), # 1: bottom
        (0,0,1),  # 2: top
        (0,-1,0), # 3: front
        (1,0,0),  # 4: right
        (0,1,0),  # 5: back
        (-1,0,0)  # 6: left
    ]

    def get_v(x_w, y_w, z_w):
        key = (round(x_w, 4), round(y_w, 4), round(z_w, 4))
        if key not in v_map:
            v_map[key] = len(vertices) + 1
            vertices.append(key)
        return v_map[key]

    def add_rect(x_p, y_p, w_p, h_p, z_bottom, z_top):
        x0 = x_p * resolution + origin[0]
        y0 = (height - (y_p + h_p)) * resolution + origin[1]
        x1 = (x_p + w_p) * resolution + origin[0]
        y1 = (height - y_p) * resolution + origin[1]

        v1 = get_v(x0, y0, z_bottom)
        v2 = get_v(x1, y0, z_bottom)
        v3 = get_v(x1, y1, z_bottom)
        v4 = get_v(x0, y1, z_bottom)
        v5 = get_v(x0, y0, z_top)
        v6 = get_v(x1, y0, z_top)
        v7 = get_v(x1, y1, z_top)
        v8 = get_v(x0, y1, z_top)

        # We also define the normal index for each face
        # bottom
        faces.extend([
            ((v1, 1), (v2, 1), (v3, 1)), ((v1, 1), (v3, 1), (v4, 1))
        ])
        # top
        faces.extend([
            ((v5, 2), (v6, 2), (v7, 2)), ((v5, 2), (v7, 2), (v8, 2))
        ])
        # side front (y0)
        faces.extend([
            ((v1, 3), (v2, 3), (v6, 3)), ((v1, 3), (v6, 3), (v5, 3))
        ])
        # side right (x1)
        faces.extend([
            ((v2, 4), (v3, 4), (v7, 4)), ((v2, 4), (v7, 4), (v6, 4))
        ])
        # side back (y1)
        faces.extend([
            ((v3, 5), (v4, 5), (v8, 5)), ((v3, 5), (v8, 5), (v7, 5))
        ])
        # side left (x0)
        faces.extend([
            ((v4, 6), (v1, 6), (v5, 6)), ((v4, 6), (v5, 6), (v8, 6))
        ])

    visited = np.zeros_like(is_wall)

    print(f"Extracting wall geometry at {scale*100}% scale...")
    for y in range(height):
        for x in range(width):
            if is_wall[y, x] and not visited[y, x]:
                w = 1
                while x + w < width and is_wall[y, x + w] and not visited[y, x + w]:
                    visited[y, x+w] = True
                    w += 1
                visited[y, x] = True
                add_rect(x, y, w, 1, 0, wall_height)

    # Floor: Just 1 big box for the whole area
    add_rect(0, 0, width, height, -0.05, 0)

    print(f"Saving optimized OBJ with {len(vertices)} vertices and {len(faces)} faces...")
    with open(output_path, 'w') as f:
        f.write("# Oschersleben Pro with Normals\n")
        # Write vertices
        for v in vertices:
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
        # Write normals
        for n in normals:
            f.write(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n")
        # Write faces (v//vn format)
        for face in faces:
            f.write(f"f {face[0][0]}//{face[0][1]} {face[1][0]}//{face[1][1]} {face[2][0]}//{face[2][1]}\n")

if __name__ == "__main__":
    yaml_data = {
        'resolution': 0.04295,
        'origin': [-55.07650228661655, -33.57884064395765, 0.0]
    }
    script_dir = os.path.dirname(os.path.abspath(__file__))
    img_p = os.path.join(script_dir, "Oschersleben_map.png")
    out_p = os.path.join(script_dir, "meshes/track.obj")
    generate_obj(img_p, yaml_data, out_p)
    print("Done!")
