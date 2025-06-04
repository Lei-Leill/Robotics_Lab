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

def run_detection(rgb_path, depth_path, label_list, data, output_path):
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
        print(f"Transformed to meter unit: x={x:.4f}m, y={y:.4f}m, z={depth_str}")

        # Append to result list
        detection_results.append({
            "label": label,
            "coords_in_meter": [round(x, 4), round(y, 4), round(z_m, 4)],
        })
        # Draw and label
        draw.rectangle([x_min, y_min, x_max, y_max], outline="red", width=3)
        draw.text((x_min, y_min - 10), f"{label} ({depth_str})", fill="yellow")

    # Save annotated image
    image_path = os.path.join(output_path, "image_bouding_box.png")
    image.save(image_path)
    
    # Save detection results to JSON
    json_path = os.path.join(output_path, "location.json")
    with open(json_path, "w") as f:
        json.dump(detection_results, f, indent=4)
    print(f"Annotated image and location data saved to {output_path}")


#Transformation code
def calculate_x_y(x_prime,y_prime,c_x, c_y, f_x, f_y,z):
  """
  Calculates the (x), and (y) given the image distance(x_ prime) and (y_ prime) ,
   and the principal points (c_x) and (c_y) ,object distance from lens (z), and focal length (f).

  Args:
    x: The object distance (x).
    y: The object distance (y).
    z: The object distance from the lens (z).
    f_x: The x-coordinate of the focal length of the lens (f).
    f_y = The y-coordinate of the focal length of the lens (f).
    x_prime: The image distance (x_prime).
    y_prime: The image distance (y_prime).
    c_x: The x-coordinate of the center of the lens (c_x),.
    c_y: The y-coordinate of the center of the lens (c_y).

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
        run_detection(rgb_path, depth_path, label_list, data, output_dir)

if __name__ == "__main__":
    main()
