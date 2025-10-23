import os
import cv2
import torch

# Define the path to the custom YOLOv5 model
model_path = '/home/daniel/work/ICS_Current/resources/best.pt'
# Load the custom YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)

# Define the kernel size for the blur effect
kernel_size = (25, 25)

# Define the blur function
def blur(img, bbox):
    # Convert bbox coordinates from (x1,y1,x2,y2) to (x,y,w,h)
    if len(bbox) > 4:
        bbox = bbox[:4]
    x, y, w, h = bbox
    # Extract the region of interest from the image
    roi = img[y:y+h, x:x+w]
    # Apply a blur effect to the roi
    blurred_roi = cv2.GaussianBlur(roi, kernel_size, 0)
    # Replace the original roi with the blurred roi
    img[y:y+h, x:x+w] = blurred_roi
    return img


def blur_privacy(image_dir):

    # Loop over the input images and apply the blur effect
    for filename in os.listdir(image_dir):
        # Load the input image
        img_path = os.path.join(image_dir, filename)
        img = cv2.imread(img_path)

        # Check if the image was loaded successfully
        if img is None:
            print(f'Error: Failed to load image {img_path}')
            continue

        # Use the custom YOLOv5 model to detect objects in the image
        results = model(img)

        # Check if any objects were detected in the image
        if len(results.xyxy[0]) == 0:
            # Remove the input image
            os.remove(img_path)
            print(f'Removed image {img_path} because no objects were detected.')
            continue

        # Extract the bounding boxes and class labels for the detected objects
        bboxes = results.xyxy[0].cpu().numpy()
        class_indices = results.xyxy[0][:, -1].long().cpu()
        class_labels = [results.names[int(x)] for x in class_indices]

        # Loop over the detected objects and blur them in the image
        for bbox, class_label in zip(bboxes, class_labels):
            # Blur objects with class labels of "person" or "car"
            if class_label in ["Tree"]:
                img = blur(img, bbox.astype(int))

        # Save the output image and delete the original
        os.remove(img_path)
        cv2.imwrite(img_path, img)
