import requests
import torch
from PIL import Image, ImageDraw
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
import os
model_id = "IDEA-Research/grounding-dino-tiny"
device = "cuda"
processor = AutoProcessor.from_pretrained(model_id)
model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(device)

image_url = "http://images.cocodataset.org/val2017/000000039769.jpg"
image = Image.open(requests.get(image_url, stream=True).raw)
# Check for cats and remote controls
text_labels = [["a cat", "a remote control"]]

folder_path = "rgb_detection"
for filename in os.listdir(folder_path):
    full = folder_path + "/" + filename
    image = Image.open(full)
    #text_labels = [["a cube", "a square","a cylinder","a circle","a star","a block"]]
    text_labels = [["a blue clover shape"]]


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

    # Retrieve the first image result
    result = results[0]
    if(len(result["labels"])==0):
        print("Detection failed for " + filename)
    for box, score, labels in zip(result["boxes"], result["scores"], result["labels"]):
        box = [round(x, 2) for x in box.tolist()]
        print(f"Detection success for {filename}, detected {labels} with confidence {round(score.item(), 3)} at location {box}")

        # Draw the bounding box (xmin, ymin, xmax, ymax)
        draw.rectangle(box, outline="red", width=3)

        # Optionally, add the label text near the bounding box
        text_position = (box[0], box[1] - 10)  # Position the text above the box
        draw.text(text_position, labels, fill="yellow")

    # Show the image with bounding boxes
    #image.show()
    savename = "Bounding_boxes_" + filename
    image.save(savename)


