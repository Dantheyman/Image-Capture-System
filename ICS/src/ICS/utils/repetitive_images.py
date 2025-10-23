import os
import cv2
import numpy as np
from PIL import Image

# Define a threshold NCC value
threshold_ncc = 0.9
# Define a threshold MSE value
threshold_mse = 45

# Remove all duplicate/similar images
def remove_similar_images(image_dir):

    similarity_detection_gps(image_dir)
    similarity_detection_ncc(image_dir)
    similarity_detection_mse(image_dir)



def similarity_detection_gps(image_dir):
    # Loop through all files in the directory
    for files in os.listdir(image_dir):
        
        image_info = []

        # Loop through all files in the current directory
        for file in files:
                # Check if the file is an image 
                if file.endswith(".jpg") or file.endswith(".jpeg") or file.endswith(".png"):
                    tokens = file.split('_')
                    lat, lon = extract_lat_lon(tokens)
                    if lat is not None and lon is not None and lat <= 1000.0 and lon <= 1000.0:
                        image_info.append((file, lat, lon))

    images_to_remove = set()
    # Compare each image with the others
    for i, (file1, lat1, lon1) in enumerate(image_info):
        for j, (file2, lat2, lon2) in enumerate(image_info):
            if i == j:
                continue             
            if ((abs(lat1-lat2)<=0.000001) and (abs(lon1-lon2)<=0.000001)):
                images_to_remove.add(file2)

    for image in images_to_remove:
        os.remove(os.path.join(image_dir,image))

# Helper to extract lat/lon from filename
def extract_lat_lon(filename_tokens):
    try:
        lat_idx = filename_tokens.index("lat")
        lon_idx = filename_tokens.index("lon")
        lat = float(filename_tokens[lat_idx + 1])
        lon = float(filename_tokens[lon_idx + 1])
        return lat, lon
    except (ValueError, IndexError):
        print("error parsing image name")
        exit(1)

def similarity_detection_ncc(image_dir):

    # Load and preprocess images
    image_filenames = os.listdir(image_dir)
    images = []

    for filename in image_filenames:
        image_path = os.path.join(image_dir, filename)
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue  # Skip unreadable images
        resized_image = cv2.resize(image, (256, 146))
        images.append(resized_image)

    # Compute NCC between unique image pairs
    n_images = len(images)
    ncc_values = np.zeros((n_images, n_images))
    for i in range(n_images):
        for j in range(i + 1, n_images):  # Only compare each pair once
            result = cv2.matchTemplate(images[i], images[j], cv2.TM_CCOEFF_NORMED)
            ncc = np.max(result)
            ncc_values[i, j] = ncc

    # Track images to remove
    images_to_remove = set()

    for i in range(n_images):
        for j in range(i + 1, n_images):
            if ncc_values[i, j] > threshold_ncc:
                if j not in images_to_remove and i not in images_to_remove:
                    images_to_remove.add(j)

    # delete images 
    for idx in images_to_remove:
        os.remove(os.path.join(image_dir,image_filenames[idx]))

# Load and preprocess image
def preprocess_image(path, size=(256, 146)):
    img = Image.open(path).convert("L").resize(size)
    return np.array(img)

def similarity_detection_mse(image_dir):
    # Load all images
    images = []
    for filename in os.listdir(image_dir):
        if filename.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
            full_path = os.path.join(image_dir, filename)
            try:
                pixel_array = preprocess_image(full_path)
                images.append((filename, pixel_array))
            except Exception as e:
                print(f"Failed to load {filename}: {e}")

    # Track duplicates
    duplicates = set()

    # Compare images
    for i in range(len(images)):
        filename_i, img_i = images[i]
        if filename_i in duplicates:
            continue
        for j in range(i + 1, len(images)):
            filename_j, img_j = images[j]
            if filename_j in duplicates:
                continue
            mse = np.mean((img_i - img_j) ** 2)
            if mse < threshold_mse:
                duplicates.add(filename_j)

    # Delete duplicate files
    for dup_filename in duplicates:
        image_path = os.path.join(image_dir, dup_filename)
        os.remove(image_path)