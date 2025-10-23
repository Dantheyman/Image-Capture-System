
from ICS.utils import yolo_remover, blur_detection, repetitive_images, photo_upload

def run_pre_processing_and_uploading(image_directory):
    # remove blurry and repetitive images
    blur_detection.remove_blurry_photos(image_directory)
    repetitive_images.remove_similar_images(image_directory)

    #blur images for privacy
    yolo_remover.blur_privacy(image_directory)

    #upload remaining images to database
    photo_upload.upload_directory(image_directory)

