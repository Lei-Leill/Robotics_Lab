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
    """Transform camera coordinates to world coordinates using homogeneous transformation matrix"""
    col_vec = point_c.reshape(-1, 1) # change the array to a column vector of shape (4*1)
    result = np.dot(H, col_vec)
    return result[:3].flatten()

def run_detection(rgb_path, depth_path, label_list, data, output_path, transformation_matrix):
    # Load images
    image = Image.open(rgb_path).convert("RGB")
    depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)  # 16-bit depth

    if depth_image is None:
        raise FileNotFoundError(f"Could not load depth image from: {depth_path}")
    
    if depth_image.shape[0] != image.size[1] or depth_image.shape[1] != image.size[0]:
        raise ValueError("Depth and RGB image dimensions do not match!")

    processor, model, device = load_model()

    # Prepare input
    text_labels = [label_list]
    inputs = processor(images=image, text=text_labels, return_tensors="pt").to(device)

    with torch.no_grad():
        outputs = model(**inputs)

    results = processor.post_process_grounded_object_detection(
        outputs,
        inputs.input_ids,
        box_threshold=0.4,
        text_threshold=0.3,
        target_sizes=[image.size[::-1]]  # (H, W)
    )

    # Annotate and extract (x, y, z)
    draw = ImageDraw.Draw(image)
    result = results[0]

    # Store all detection results here
    detection_results = []
    world_point = [0,0,0]
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

        print(f"Detected {label} at (x={u}, y={v})")
        x, y = calculate_x_y(u,v, data['cx'], data['cy'], data['fx'], data['fy'], z_m)
        print(f"Transformed to meter unit/Camera coordinates: x={x:.4f}m, y={y:.4f}m, z={depth_str}")
        
        if z_m is not None and transformation_matrix:
            H = np.array(transformation_matrix)
            camera_point = np.array([x, y, z_m,1])
            world_point = camera_to_world(H, camera_point)

            '''detection_results.append({
                "label": label,
                "coords_in_meter": [round(x, 4), round(y, 4), round(z_m, 4)],
                "coords_in_world": coords_world
            })'''

    
        # Draw and label
        draw.rectangle([x_min, y_min, x_max, y_max], outline="red", width=3)
        draw.text((x_min, y_min - 10), f"{label} ({depth_str})", fill="yellow")

    # Save annotated image
    image_path = os.path.join(output_path, "image_bouding_box.png")
    image.save(image_path)
    '''
    # Save detection results to JSON
    json_path = os.path.join(output_path, "location.json")
    with open(json_path, "w") as f:
        json.dump(detection_results, f, indent=4)'''
    #print(f"Annotated image saved to {output_path}")
    return world_point


#Transformation code
def calculate_x_y(x_prime,y_prime,c_x, c_y, f_x, f_y,z):
  """
  Calculates the (x), and (y) given the image distance(x_ prime) and (y_ prime) ,
  and the principal points (c_x) and (c_y) ,object distance from lens (z), and focal length (f).

  Returns: x, y
  """
  x = (x_prime - c_x)* z/f_x
  y = (y_prime - c_y)* z/f_y

  return x,y

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

    transformation_matrix = [[[ 0.99009612, -0.01594327, -0.13948292,  0.42709209],
                            [ 0.09441481, -0.65968152,  0.74559113, -0.75780215],
                            [-0.10390147, -0.75137614, -0.65164283,  0.50147743],
                            [ 0.,          0. ,         0.   ,       1.        ]], 

                            [[-0.99950439, -0.03119223, -0.00424504,  0.4710198 ],
                            [-0.01747422,  0.66191747, -0.74937301,  0.8043515 ],
                            [ 0.02618447, -0.74892744, -0.66213447,  0.40551513],
                            [ 0. ,         0. ,         0. ,         1.        ]]]
    i = 0
    left_right_point = []
    for name in ['left_camera', 'right_camera']:
        camera_path = os.path.join(args.folder, name)
        #print(camera_path)
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
        except json.JSONDecodeError:
            print("Error decoding JSON.")
        
        print(f"\nObject Detection Running for {name}!!")
        output_dir = f"output/{name}"
        os.makedirs(output_dir, exist_ok=True)
        world_p = run_detection(rgb_path, depth_path, label_list, data, output_dir, transformation_matrix[i])
        print(f'index {i}: {world_p}')
        left_right_point.append(world_p)
        i += 1
    
    print("World Coordinate for the detected object")
    for array in left_right_point:
        print(array)


if __name__ == "__main__":
    main()
