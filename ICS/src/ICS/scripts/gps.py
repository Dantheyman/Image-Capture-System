#!/usr/bin/env python3.8
# The configuration assumes Python3.8 is being used. Enforce the correct version
import rospy
from std_msgs.msg import String,Bool
from ublox_gps import UbloxGps
import serial

node = None

class GpsNode:
    def __init__(self):
        self.gps_active = False
        self.node_active = True
        self.pub = rospy.Publisher('gps_coordinates', String, queue_size=10)
        rospy.init_node('gps_coordinates', anonymous=True)
        rospy.Subscriber('toggle_capture', Bool, self.toggle_gps)
        rospy.loginfo("GPS node started")

        try:
            self.port = serial.Serial('/dev/ttyACM1', baudrate=38400, timeout=10)
            self.gps = UbloxGps(self.port)
        except Exception as e:
            rospy.logerr(f"Failed to connect to GPS: {e}")
            return
        

    def toggle_gps(self,msg):
        print("help")
        print(msg.data)
        self.gps_active = msg.data  

    def run_gps(self):
        while not rospy.is_shutdown() and self.node_active:
            if self.gps_active:
                try:
                    geo = self.gps.geo_coords()
                    coord = f"{geo.lat}#{geo.lon}"
                    self.pub.publish(coord)
                    print(coord)
                except Exception as e:
                    rospy.logwarn(f"GPS read failed: {e}")
            rospy.sleep(0.1)

    
    def close(self):
        self.port.close()
        self.node_active = False



def ros_shutdown(msg):
    node.close()
    print("gps shutting down")
    rospy.signal_shutdown("shutdown called ")

if __name__ == '__main__':
    try:
        rospy.Subscriber('shutdown', Bool,ros_shutdown)
        node = GpsNode()
        node.run_gps()
    except rospy.ROSException:
        pass

