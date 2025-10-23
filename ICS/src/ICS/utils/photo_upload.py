import os
import requests

url = "http://localhost:8000/uploadTreeImage"  


#uploads all images in the given directory to the database
def upload_directory(image_directory):

    for filename in os.listdir(image_directory):
            file_path = os.path.join(image_directory, filename)
            with open(file_path, "rb") as image_file:
                file = {
                    "file": (filename, image_file, "image/jpeg")  
                }
                #uploads and waits for response 
                response = requests.post(url, files=file)
                data = response.json()
                if data["success"] != "True":
                    print("Error Uploading File: "+file_path)
    
    print("Uploading Complete")
