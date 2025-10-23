#!/usr/bin/env python3.8
import csv
import datetime
import os
import re
import rospy
import cv2 as cv
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import numpy as np
import pandas as pd
from pypylon import pylon

from ICS import config



latest_gps_data = ''
old_gps = ""
cap_id = 0
store_photos_path = config.store_photos_path
database_url = ""






# process the gps data
def process_gps_data(data):
    global old_gps
    global latest_gps_data
    # read latest data
    gps_data = data.data
    gps_data = str(gps_data)
    # if the gps data has been updated/changed
    if old_gps != gps_data:
        # update latest gps information
        latest_gps_data = gps_data
        old_gps = gps_data
        # split gps into lat and lon
        coord = latest_gps_data.split("#")
        rospy.loginfo("Latest gps data: %s", coord[0])
        

# save a frame from a camera
def save_frame1(data):

    global cap_id

    br = CvBridge()
    frame = br.imgmsg_to_cv2(data)

    # get data and time
    current_date = datetime.date.today()
    t = datetime.datetime.now()

    #print("step 1")
    time_current = str(t.hour) + ':' + str(t.minute) + ':' + str(t.second)
   
    
   
    coordinates = str(latest_gps_data)
    list = coordinates.split("#")
    print(f"the cords are {coordinates}")
    rospy.loginfo("coordinates: %s", coordinates)

    #print("step 2")
    name = "capture_" + str(cap_id) + "_lat_" + list[0] + "_lon_" + list[1] + "_date_" + str(current_date) + ".jpeg"
    save_location = os.path.join(store_photos_path, name)
    result = cv.imwrite(save_location, frame)
    print(f"Image saved: {result} at {save_location}")

    
    cap_id+=1



# save a frame from a camera
def save_frame2(data):

    global cap_id


    br = CvBridge()
    frame = br.imgmsg_to_cv2(data)

    # get data and time
    current_date = datetime.date.today()
    t = datetime.datetime.now()
    time_current = str(t.hour) + ':' + str(t.minute) + ':' + str(t.second)
    
    
    
    coordinates = str(latest_gps_data)

    name = "capture_" + str(cap_id) + "_lat_" + list_coord[0] + "_lon_" + list_coord[1] + ".jpeg"
    save_location = os.path.join(store_photos_path, name)
    # cv.imwrite(save_location, frame)
    print(os.path.join(store_photos_path, name))
    cap_id += 1



def ros_shutdown(msg):
        rospy.loginfo("shutting down main node")
        rospy.signal_shutdown("shutdown called ")


def run_main():
    rospy.init_node("main_node", anonymous=True)
    rospy.loginfo("main node started")
    rospy.Subscriber('gps_coordinates', String, process_gps_data)
    rospy.Subscriber('frames', Image, save_frame1)
    rospy.Subscriber('frames2', Image, save_frame2)
    rospy.Subscriber('shutdown', Bool, ros_shutdown)


if __name__ == '__main__':
    try:
        run_main()
        rospy.spin()
        # destroy all cv windows on completion
        cv.destroyAllWindows()
    except rospy.ROSException:
        pass