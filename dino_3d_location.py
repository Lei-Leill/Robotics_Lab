import argparse
import torch
import cv2
import numpy as np
from PIL import Image, ImageDraw
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
import json
import os

def load_model(model_id="IDEA-Research/grounding-dino-tiny"):
    device = "cuda" if torch.cuda.is_available() else "cpu"
    processor = AutoProcessor.from_pretrained(model_id)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(device)
    return processor, model, device

def camera_to_world(H, point_c):
    col_vec = point_c.reshape(-1, 1)
    result = np.dot(H, col_vec)
    return result[:3].flatten()

def run_detection(rgb_path, depth_path, label_list, data, output_path, transformation_matrix):
    image = Image.open(rgb_path).convert("RGB")
    depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

    if depth_image is None:
        raise FileNotFoundError(f"Could not load depth image from: {depth_path}")
    
    if depth_image.shape[0] != image.size[1] or depth_image.shape[1] != image.size[0]:
        raise ValueError("Depth and RGB image dimensions do not match!")

    processor, model, device = load_model()

    text_labels = [label_list]
    inputs = processor(images=image, text=text_labels, return_tensors="pt").to(device)

    with torch.no_grad():
        outputs = model(**inputs)

    results = processor.post_process_grounded_object_detection(
        outputs,
        inputs.input_ids,
        box_threshold=0.4,
        text_threshold=0.3,
        target_sizes=[image.size[::-1]]
    )

    draw = ImageDraw.Draw(image)
    result = results[0]

    world_points = []
    for box, score, label in zip(result["boxes"], result["scores"], result["labels"]):
        box = [round(x, 2) for x in box.tolist()]
        x_min, y_min, x_max, y_max = map(int, box)
        u = int((x_min + x_max) / 2)
        v = int((y_min + y_max) / 2)

        if 0 <= v < depth_image.shape[0] and 0 <= u < depth_image.shape[1]:
            z_mm = depth_image[v, u]
            z_m = z_mm / 1000.0
            depth_str = f"{z_m:.2f} m"
        else:
            z_m = None
            depth_str = "no depth"

        x, y = calculate_x_y(u, v, data['cx'], data['cy'], data['fx'], data['fy'], z_m)
        
        if z_m is not None and transformation_matrix:
            H = np.array(transformation_matrix)
            camera_point = np.array([x, y, z_m, 1])
            world_point = camera_to_world(H, camera_point)
            print(f"Detected {label} at world point: {world_point}")
            world_points.append(world_point)

        draw.rectangle([x_min, y_min, x_max, y_max], outline="red", width=3)
        draw.text((x_min, y_min - 10), f"{label} ({depth_str})", fill="yellow")

    image_path = os.path.join(output_path, "image_bouding_box.png")
    image.save(image_path)

    if world_points:
        return world_points[0]
    else:
        return None

def calculate_x_y(x_prime, y_prime, c_x, c_y, f_x, f_y, z):
    x = (x_prime - c_x) * z / f_x
    y = (y_prime - c_y) * z / f_y
    return x, y

def find_latest_file(folder, prefix):
    files = [
        os.path.join(folder, f) for f in os.listdir(folder)
        if f.startswith(prefix)
    ]
    if not files:
        return None
    return max(files, key=os.path.getmtime)

def main():
    parser = argparse.ArgumentParser(description="Grounding DINO + Depth")
    parser.add_argument("--folder", type=str, required=True, help="Folder path to camera info, rbg, and depth images of left and right camera")
    parser.add_argument("--label", type=str, required=True, help="Text file that contains comma-separated list of labels (e.g. a green cube,a blue cylinder)")

    args = parser.parse_args()
    file = open(args.label, 'r')
    content = file.read()
    label_list = [label.strip() for label in content.split(',')]

    transformation_matrix = [[[0.99016408, -0.00400559, -0.13985367,  0.41902076],
                            [ 0.10285259, -0.65681041,  0.74700832, -0.79932346],
                            [-0.09484956, -0.75404512, -0.64993809,  0.45859295],
                            [ 0. ,         0. ,         0.,          1.        ]],

                            [[-0.99950439, -0.03119223, -0.00424504,  0.4710198 ],
                            [-0.01747422,  0.66191747, -0.74937301,  0.8043515 ],
                            [ 0.02618447, -0.74892744, -0.66213447,  0.40551513],
                            [ 0. ,         0.  ,        0. ,         1.        ]]]

    i = 0
    det_result = []
    for name in ['left_camera', 'right_camera']:
        camera_path = os.path.join(args.folder, name)
        if not os.path.isdir(camera_path):
            print(f"Camera folder {camera_path} not found.")
            continue
        rgb_path = find_latest_file(camera_path, "rgb_")
        depth_path = find_latest_file(camera_path, "depth_")
        info_path = os.path.join(camera_path, "camera_info.json")
        try:
            with open(info_path, 'r') as file:
                data = json.load(file)
        except FileNotFoundError:
            print("The file was not found.")
            continue
        except json.JSONDecodeError:
            print("Error decoding JSON.")
            continue
        
        print(f"\nObject Detection Running for {name}!!")
        output_dir = f"output/{name}"
        os.makedirs(output_dir, exist_ok=True)
        world_p = run_detection(rgb_path, depth_path, label_list, data, output_dir, transformation_matrix[i])
        if world_p is not None:
            print(f'{name} world point: {world_p}')
            det_result.append(world_p)
        i += 1

    if not det_result:
        print("No detections were made from either camera.")
        return None

    avg_arr = []
    length = len(det_result)
    if length == 2:
        avg_arr = [float((det_result[0][i] + det_result[1][i])/2) for i in range(3)]
    elif length == 1:
        avg_arr = det_result[0]
    else:
        print("No valid detections.")
        return None

    print(f"DETECTION_RESULT:{avg_arr}")    
    return avg_arr

if __name__ == "__main__":
    main()

