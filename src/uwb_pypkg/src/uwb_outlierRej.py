#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
import numpy as np
import time
from uwbdriver import uwbdriver
import pickle

class uwbOutlierRej(object):
    """docstring for uwbOutlierRej"""
    def __init__(self):
        super(uwbOutlierRej, self).__init__()        #self.pub_uwb_raw = rospy.Publisher('/uwb/rawrange', PointStamped, queue_size=10)
        self.pub_uwb = rospy.Publisher('/uwb/corrected_range', PointStamped, queue_size=10)
        self.sub_rawrange = rospy.Subscriber("/uwb/rawrange1", PointStamped, self.uwb_callback)
#        self.sub_rawrange = rospy.Subscriber("/uwb/range", PointStamped, self.uwb_callback)

        self.lastRanges = []
        self.lastRevRange = 0
        self.RANGE_SIZE = 10
        self.offset = []
        self.offsetTime = rospy.Time.now()
        self.OFFSET_THRESH = 0.2
        self.SDV = 0.4
        self.rangeUpdated = False


    
    def uwb_callback(self,data):
        revRange = data.point.x
        print("--- meas arrive --",len(self.lastRanges))
        if len(self.lastRanges)<self.RANGE_SIZE:
            self.lastRanges.append(revRange)
        else:
            print(revRange, self.lastRevRange, len(self.lastRanges))
            if revRange!=self.lastRevRange:
                print("new data")
                offset = revRange- np.mean(self.lastRanges)
                offsetToLast = revRange - self.lastRevRange

                if offset > self.OFFSET_THRESH and abs(offsetToLast) > self.OFFSET_THRESH: # Get postive outliers first time
                    self.offset = offset
                    self.offsetTime = rospy.Time.now()
                    self.rangeUpdated = True # Use last range as new range, course they are same.
                    print(" Up edge ", revRange, " offset ", offset, " offset to last ", offsetToLast, " ranges ", self.lastRanges)
                elif offset > self.OFFSET_THRESH: # Get postive outliers after first up edge.
                    self.lastRanges.append(revRange-self.offset)
                    self.lastRanges = self.lastRanges[1:]
                    self.rangeUpdated = True
                    print(" Normal on Up ", revRange, " offset ", offset, " offset to last ", offsetToLast, " ranges ", self.lastRanges)
                elif offsetToLast < -self.OFFSET_THRESH:
                    temp = self.lastRanges[:]
                    temp.append(revRange)
                    self.offset = 0
                    print(" Down edge without offset ", revRange, " offset ", offset, " offset to last ", offsetToLast, " ranges ", self.lastRanges)
                else:
                    print("normal case")
                    # Normal cases
                    self.lastRanges.append(revRange)
                    self.lastRanges = self.lastRanges[1:]
                    self.rangeUpdated = True
                    self.offset = 0
                
                if rospy.Time.now() - self.offsetTime > rospy.Duration(1):
                    self.offset = 0
                    self.rangeUpdated == False
                    print(" offset time bigger")

                if data.header.stamp - self.lastRangeHeader.stamp > rospy.Duration(1): # blanket duration without data arrived.
                    self.lastRanges = []
                    self.offset = 0
                    self.rangeUpdated == False
                    print( " blank duration")
                
                if self.rangeUpdated == True:
                    print("pub")
                    filteredRange = PointStamped()
                    filteredRange.header = data.header
                    filteredRange.point.x = self.lastRanges[-1]
                    self.pub_uwb.publish(filteredRange)

            else: 
                print("repetative range")
                self.lastRanges = self.lastRanges[:len(self.lastRanges)-1]
                pass
        self.lastRevRange = revRange
        self.lastRangeHeader = data.header
        print("end set lastRevRange",revRange, self.lastRevRange,)






    def getTrainedModel(self):
        with open('/home/nvidia/yanjun/uvi_slam/src/uwb_pypkg/src/uwbCalibPNmodel.pickle', 'rb') as handle:
            self.uwbPNmodel = pickle.load(handle)
       




if __name__ == '__main__':
    try:
        rospy.init_node('uwbOutlierRej', anonymous=True)
        uwbOutlierRejx = uwbOutlierRej()
        #uwbOutlierRejx.getTrainedModel()
        rospy.loginfo("uwbOutlierRej initialized, start correcting ranging")
        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass


