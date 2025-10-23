#!/usr/bin/env python3.8

import rospy
from kivy.lang import Builder
from kivymd.app import MDApp
from std_msgs.msg import Bool
from kivy.app import App
import threading
import time

from ICS.config import store_photos_path
from ICS.utils import photo_automation

class IcsGuiApp(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.publishers_ready = False
        self.ros_thread = None
        self.shutdown_requested = False
        
        try:
            self.screen = Builder.load_file('/home/daniel/work/ICS_Current/ICS/src/ICS/gui.kv')
        except Exception as e:
            print(f"[ERROR] Failed to load KV file: {e}")
    
    def build(self):
        return self.screen
    
    def on_start(self):
        # Give ROS time to initialize
        time.sleep(0.5)
        self.publishers_ready = True
    
    def start_capture(self, *args):
        if self.publishers_ready and not self.shutdown_requested:
            pub.publish(True)
            print("Start capture message sent")
    
    def stop_capture(self, *args):
        if self.publishers_ready and not self.shutdown_requested:
            pub.publish(False)
            print("Stop capture message sent")
    
    def stop(self, *args):
        print("Initiating shutdown...")
        self.shutdown_requested = True
        
        # Send shutdown message to ROS nodes
        if self.publishers_ready:
            try:
                shutdown_pub.publish(True)
                print("Shutdown message sent to ROS nodes")
                # Give ROS nodes time to shutdown
                time.sleep(1.0)
            except Exception as e:
                print(f"Error sending shutdown message: {e}")
        
        # Stop the Kivy app
        try:
            super().stop()
        except Exception as e:
            print(f"Error stopping Kivy app: {e}")
        
        # Shutdown ROS
        try:
            if not rospy.is_shutdown():
                rospy.signal_shutdown("GUI shutdown requested")
        except Exception as e:
            print(f"Error shutting down ROS: {e}")

    def store_photos(self,*args):
        photo_automation.run_pre_processing_and_uploading(store_photos_path)
        delete_all_files_in_folder(store_photos_path)
        
            
def ros_spin_thread():
    try:
        rospy.spin()
    except Exception as e:
        print(f"ROS spin error: {e}")

def delete_all_files_in_folder(folder_path):
    for filename in os.listdir(folder_path):
        file_path = os.path.join(folder_path, filename)
        if os.path.isfile(file_path):
            try:
                os.remove(file_path)
                print(f"Deleted file: {file_path}")
            except Exception as e:
                print(f"Error deleting {file_path}: {e}")


if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node("gui_node", anonymous=True)
        
        # Create publishers
        pub = rospy.Publisher("toggle_capture", Bool, queue_size=10)
        shutdown_pub = rospy.Publisher("shutdown", Bool, queue_size=10)
        
        # Start ROS spin in background thread
        ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
        ros_thread.start()
        
        # Run the GUI app
        app = IcsGuiApp()
        app.run()
        
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        # Ensure ROS is shutdown
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Main thread exiting")
        print("Application exited")
