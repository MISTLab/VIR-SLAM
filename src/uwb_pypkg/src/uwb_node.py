#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
import numpy as np
import time
from uwbdriver import uwbdriver
import pickle


class uwbNode(object):
    """docstring for uwbNode"""
    def __init__(self, port, baudrate):
        super(uwbNode, self).__init__()
        self.uwbDevice = uwbdriver(port=port, baudrate=baudrate, goonMeas=False)
        self.pub_uwb1_raw1 = rospy.Publisher('/uwb/rawrange1', PointStamped, queue_size=10)
        self.pub_uwb1 = rospy.Publisher('/uwb/range1', PointStamped, queue_size=10)
        self.pub_uwb1_raw2 = rospy.Publisher('/uwb/rawrange2', PointStamped, queue_size=10)
        self.pub_uwb2 = rospy.Publisher('/uwb/range2', PointStamped, queue_size=10)


    #def getTrainedModel(self):
    #    with open('/home/nvidia/yanjun/uvi_slam/src/uwb_pypkg/src/uwbCalibPNmodel.pickle', 'rb') as handle:
    #        self.uwbPNmodel = pickle.load(handle)

    def getTrainedModel(self):
        with open('/home/jun/projects/uvi_slam/src/uwb_pypkg/src/uwbCalibPNmodel.pickle', 'rb') as handle:
            self.uwbPNmodel = pickle.load(handle)


    def run(self):
        while not rospy.is_shutdown():
            self.uwbDevice.getRange()
            msgnowRaw = PointStamped()
            msgnowRaw.header.stamp = rospy.Time.now()
            msgnowRaw.point.x = self.uwbDevice.rangeRaw01/1000.0
            self.pub_uwb1_raw1.publish(msgnowRaw)

            msgnow = PointStamped()
            msgnow.header.stamp = rospy.Time.now()
            msgnow.point.x = float(self.uwbPNmodel.predict(np.array([self.uwbDevice.rangeRaw01/1000.0]).reshape(-1,1)))# corrected by pre-trained model
            #print(self.uwbDevice.rangeRaw01/1000.0, " -> ", float(self.uwbPNmodel.predict(np.array([self.uwbDevice.rangeRaw01/1000.0]).reshape(-1,1))))
            #print("predict ", msgnow.point.x)
            self.pub_uwb1.publish(msgnow)

            msgnowRaw2 = PointStamped()
            msgnowRaw2.header.stamp = rospy.Time.now()
            msgnowRaw2.point.x = self.uwbDevice.rangeRaw02/1000.0
            self.pub_uwb1_raw2.publish(msgnowRaw2)

            msgnow2 = PointStamped()
            msgnow2.header.stamp = rospy.Time.now()
            msgnow2.point.x = float(self.uwbPNmodel.predict(np.array([self.uwbDevice.rangeRaw02/1000.0]).reshape(-1,1)))# corrected by pre-trained model
            #print(self.uwbDevice.rangeRaw01/1000.0, " -> ", float(self.uwbPNmodel.predict(np.array([self.uwbDevice.rangeRaw01/1000.0]).reshape(-1,1))))
            #print("predict ", msgnow.point.x)
            self.pub_uwb2.publish(msgnow2)


            self.uwbDevice.rangeGot = False

            



if __name__ == '__main__':
    try:
        port = "/dev/ttyACM0"
        baudrate = 115200
        uwbNodex = uwbNode(port,baudrate)
        uwbNodex.getTrainedModel()
        rospy.init_node('uwbNode', anonymous=True)
        rospy.loginfo("uwbNode initialized, start do ranging")
        #rate = rospy.Rate(100) # 10hz

        uwbNodex.run()  
        #rate.sleep()

    except rospy.ROSInterruptException:
        pass


