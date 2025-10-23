import os
import cv2
import matplotlib.pyplot as plt
import numpy as np

#removes all blurry photos from a directory
def remove_blurry_photos(image_dir):

    # Supported image extensions
    image_extensions = ('.png', '.jpg', '.jpeg')

    # Loop through files in the directory
    for files in os.listdir(image_dir):  
        for file in files:
            if file.lower().endswith(image_extensions):
                image_path = os.path.join(root, file)

                # Load the image in grayscale
                image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

                if image is None:
                    print(f"Warning: Could not read {image_path}")
                    continue
                
                # delete any images that are blurry
                if detect_blur_fft(image) or detect_blur_laplacian(image):
                    os.remove(image_path)
            

#uses the fourier transform method to determine if an image is blurry
def detect_blur_fft(image, size = 100, thresh = 10, vis = False):
	# derive the center (x, y)-coordinates
	(h, w) = image.shape
	(cX, cY) = (int(w / 2.0), int(h / 2.0))

	fft = np.fft.fft2(image)
	fftShift = np.fft.fftshift(fft)
	if vis:
		# compute the magnitude spectrum of the transform
		magnitude = 20 * np.log(np.abs(fftShift))

	# the inverse FFT
	fftShift[cY - size:cY + size, cX - size:cX + size] = 0
	fftShift = np.fft.ifftshift(fftShift)
	recon = np.fft.ifft2(fftShift)
	
	# compute the magnitude spectrum of the reconstructed image,
	# then compute the mean of the magnitude values
	magnitude = 20 * np.log(np.abs(recon))
	mean = np.mean(magnitude)
     
	# the image will be considered "blurry" if the mean value of the
	# magnitudes is less than the threshold value
	return mean <= thresh

#uses the laplacian method to determine if an image is blurry
def detect_blur_laplacian(image):

    laplacian_threshold = 90

    # Converting the image to grayscale
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Calculate laplacian variance
    laplacian_var = cv2.Laplacian(image_gray, cv2.CV_64F).var()

    # if less than threshold we consider image blurry 
    return laplacian_var < laplacian_threshold
         
