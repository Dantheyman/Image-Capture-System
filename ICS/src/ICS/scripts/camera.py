#!/usr/bin/env python3.8

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from pypylon import pylon
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import threading

# Define missing constants
MIN_EXPOSURE = 100
MAX_EXPOSURE = 50000

class CameraNode:
    def __init__(self):
        self.cameraID = 1
        self.exposure_val = 8000
        self.grabbing = False
        self.camera = None
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        self.br = CvBridge()
        self.pub = rospy.Publisher('frames', Image, queue_size=10)
        self.capture_thread = None
        self.shutdown_requested = False

        rospy.Subscriber('toggle_capture', Bool, self.start_capture_callback)
        rospy.Subscriber('shutdown', Bool, self.shutdown_callback)
    
    def shutdown_callback(self, msg):
        if msg.data:
            rospy.loginfo("Shutdown requested")
            self.shutdown_requested = True
            self.stop_capture_internal()
    
    def start_capture_callback(self, msg):
        if msg.data and not self.grabbing:
            rospy.loginfo("Starting camera capture")
            self.grabbing = True
            # Start capture in separate thread to avoid blocking
            self.capture_thread = threading.Thread(target=self.start_camera)
            self.capture_thread.daemon = True
            self.capture_thread.start()
        elif not msg.data and self.grabbing:
            rospy.loginfo("Stopping camera capture")
            self.stop_capture_internal()

    def stop_capture_internal(self):
        """Internal method to stop capture without external message"""
        self.grabbing = False
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)  # Wait max 2 seconds

    def start_camera(self):
        try:
            # Open and configure camera
            tlFactory = pylon.TlFactory.GetInstance()
            devices = tlFactory.EnumerateDevices()

            if len(devices) < self.cameraID:
                rospy.logerr("Camera not found")
                return

            self.camera = pylon.InstantCamera()
            self.camera.Attach(tlFactory.CreateDevice(devices[self.cameraID-1]))
            self.camera.Open()
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            self.camera.ExposureAuto.SetValue("Once")
            self.camera.ExposureAuto.SetValue("Continuous")
            self.camera.AutoTargetBrightness.SetValue(0.6)
            self.camera.Gamma.SetValue(1)

            # Capture loop
            while self.grabbing and not rospy.is_shutdown() and not self.shutdown_requested:
                try:
                    grabResult = self.camera.RetrieveResult(500, pylon.TimeoutHandling_ThrowException)
                    if grabResult.GrabSucceeded():
                        image_converted = self.converter.Convert(grabResult)
                        frame = image_converted.GetArray()

                        # Exposure adjustment
                        Exposure = self.GradientScore(frame, self.exposure_val)
                        print("Final Exposure", Exposure)
                        self.camera.ExposureTime.SetValue(Exposure)
                        self.exposure_val = Exposure  # Update stored exposure

                        # Publish frame
                        if not self.shutdown_requested:
                            self.pub.publish(self.br.cv2_to_imgmsg(frame))
                        
                        grabResult.Release()
                    else:
                        rospy.logwarn("Grab failed")
                except Exception as e:
                    rospy.logerr(f"Error during capture: {e}")
                    break

        except Exception as e:
            rospy.logerr(f"Camera initialization failed: {e}")
        finally:
            self.stop_camera()

    def stop_camera(self):
        """Clean camera shutdown"""
        try:
            if self.camera:
                if self.camera.IsGrabbing():
                    self.camera.StopGrabbing()
                if self.camera.IsOpen():
                    self.camera.Close()
                self.camera = None
            rospy.loginfo("Camera stopped cleanly")
        except Exception as e:
            rospy.logerr(f"Error stopping camera: {e}")

    def MeanIntensity_Test(self, frame, Exposure, *varargin):
        desired_msv = 2
        k_p = 100
        LENHIST = 3
        MAX_COUNTER = 10
        counter = 0
        bComplete = False
        
        while not bComplete and not self.shutdown_requested:
            counter += 1
            
            ImageY = cv.cvtColor(frame, cv.COLOR_BGR2HSV)[:,:,2]
            cols = ImageY.shape[1]
            rows = ImageY.shape[0]
            hist = cv.calcHist([ImageY], [0], None, [LENHIST], [0, 256])
            
            mean_sample_value = np.matmul(np.linspace(1, LENHIST, LENHIST), hist)/(rows*cols)
            err_p = float(np.log2(desired_msv / mean_sample_value))
            
            if np.abs(err_p) > 2:
                err_p = np.sign(err_p)*2
            
            Exposure += k_p*err_p
            print("Exposure Calculating (MeanIntensity):", Exposure)
            
            if abs(err_p) < 0.2:
                bComplete = True
            if (counter > MAX_COUNTER) or (Exposure < MIN_EXPOSURE) or (Exposure > MAX_EXPOSURE):
                print('Tuning exceeded camera setting or maximum number of iteration has been exceeded.')
                break
        
        print('MeanIntensity iteration count =', counter)
        return Exposure

    def GradientScore(self, frame, Exposure, *varargin):
        MAX_COUNTER = varargin[0] if len(varargin) > 0 else 10
        
        delta = 0.05
        lambd = 1e3
        Kp = 800
        edgewidth = 5
        compress = 16
        gamma = np.array([0.50, 0.67, 0.85, 1.00, 1.20, 1.50, 2.00])
        lengamma = len(gamma)
        
        cols = int(5320/compress)
        rows = int(3032/compress)
        dim = (cols, rows)
        frame = cv.resize(frame, dim, interpolation=cv.INTER_AREA)

        bContinue = True
        LoopCount = 0
        
        while bContinue and not self.shutdown_requested:
            LoopCount += 1
            ImageY = cv.cvtColor(frame, cv.COLOR_BGR2HSV)[:,:,2]

            m = np.zeros(lengamma)
            for idx, g in enumerate(gamma):
                Y = np.power(ImageY, g)
                sobelx = cv.Sobel(Y, cv.CV_64F, 1, 0, ksize=edgewidth)
                sobely = cv.Sobel(Y, cv.CV_64F, 0, 1, ksize=edgewidth)
                GradSq = np.array(sobelx*sobelx + sobely*sobely)
                ImageGrad = GradSq/np.amax(GradSq)

                bDenoise = ImageGrad > delta
                m[idx] = np.sum(np.log10(lambd*(ImageGrad[bDenoise]-delta)+1))
                m[idx] /= np.log10(lambd*(1-delta)+1)
                
            ptbest = np.argmax(m)
            logdE = Kp*self.NonlinearGain(gamma[ptbest])
            print("Adjust Value/Max Value:", logdE, "/", Kp)
            
            Exposure += logdE
            print("Realtime Exposure (GradientScore):", Exposure)
            
            if np.abs(logdE) < 10:
                bContinue = False
                    
            if (LoopCount > MAX_COUNTER) or (Exposure < MIN_EXPOSURE) or (Exposure > MAX_EXPOSURE):
                break
                
        return Exposure

    def GradientScore_SatuMask(self, frame, Exposure, *varargin):
        MAX_COUNTER = varargin[0] if len(varargin) > 0 else 1
        
        delta = 0.04
        lambd = 1e3
        Kp = 100
        edgewidth = 3
        gamma = [0.50, 0.67, 0.85, 1.00, 1.20, 1.50, 2.00]
        lengamma = len(gamma)

        bContinue = True
        LoopCount = 0
        
        while bContinue and not self.shutdown_requested:
            LoopCount += 1
            ImageY = cv.cvtColor(frame, cv.COLOR_BGR2HSV)[:,:,2]
            cols = ImageY.shape[1]
            rows = ImageY.shape[0]
                
            ret, bSatuMap = cv.threshold(ImageY, 250, 255, cv.THRESH_BINARY)
            bSatuMap = cv.dilate(bSatuMap, cv.getStructuringElement(cv.MORPH_RECT, [edgewidth, edgewidth]))
            
            m = np.zeros(lengamma)
            for idx, g in enumerate(gamma):
                Y = np.power(ImageY, g)
                sobelx = cv.Sobel(Y, cv.CV_64F, 1, 0, ksize=edgewidth)
                sobely = cv.Sobel(Y, cv.CV_64F, 0, 1, ksize=edgewidth)
                GradSq = np.array(sobelx*sobelx + sobely*sobely)
                ImageGrad = GradSq/np.amax(GradSq)

                ImageGrad[bSatuMap == 255] *= 0.25

                bDenoise = ImageGrad > delta
                m[idx] = np.sum(np.log10(lambd*(ImageGrad[bDenoise]-delta)+1))
                m[idx] /= np.log10(lambd*(1-delta)+1)

            ptbest = np.argmax(m)
            logdE = Kp*self.NonlinearGain(gamma[ptbest])
           
            Exposure += logdE
            print("Exposure Calculating (GradientScore):", Exposure)
            
            if np.abs(logdE) < 0.2:
                bContinue = False
                    
            if (LoopCount > MAX_COUNTER) or (Exposure < MIN_EXPOSURE) or (Exposure > MAX_EXPOSURE):
                print('Tuning exceeded camera setting or maximum number of iteration has been exceeded.')
                break
                
        print('GradientScore_SatuMask iteration count =', LoopCount)
        return Exposure

    def NonlinearGain(self, g):
        p = 1.5
        q = 2
        if g < 0.5:
            R = 2        
        elif g < 1:
            R = 1+(2*(1-g))**p
        elif g < 2:
            R = 1-0.5*(1-g)**q
        elif g >= 2:
            R = 0.5
        R = np.log2(R) 
        return R

    def cleanup(self):
        """Cleanup method to be called on shutdown"""
        self.shutdown_requested = True
        self.stop_capture_internal()
        self.stop_camera()

def ros_shutdown(msg):
    print("Camera shutting down")
    if hasattr(ros_shutdown, 'node'):
        ros_shutdown.node.cleanup()
    rospy.signal_shutdown("shutdown called")

if __name__ == '__main__':
    rospy.init_node('camera_node', anonymous=True)
    node = CameraNode()
    ros_shutdown.node = node  # Store reference for cleanup
    rospy.Subscriber('shutdown', Bool, ros_shutdown)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
