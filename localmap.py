import bresenham
from math import sin, cos, pi,tan, atan2,log
import math
from itertools import groupby
from operator import itemgetter
import tf
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped

class localmap:
    def __init__(self, height, width, resolution,morigin):
        self.height=height
        self.width=width
        self.resolution=resolution
        self.punknown=-1.0
        self.localmap=[self.punknown]*int(self.width/self.resolution)*int(self.height/self.resolution)
        self.logodds=[0.0]*int(self.width/self.resolution)*int(self.height/self.resolution)
        self.origin=int(math.ceil(morigin[0]/resolution))+int(math.ceil(width/resolution)*math.ceil(morigin[1]/resolution))
        self.pfree=log(0.3/0.7)
        self.pocc=log(0.9/0.1)
        self.prior=log(0.5/0.5)
        self.max_logodd=100.0
        self.max_logodd_belief=10.0
        self.max_scan_range=1.0
        self.map_origin=morigin



    def updatemap(self,scandata,angle_min,angle_max,angle_increment,range_min,range_max,pose):
        #print(scandata)
        #np.nan_to_num(scandata)
        #print(type(scandata))

        '''
        in pixel enviroment, we should know the point where robot is.
        and we can calculate that point with following code.
        pose[0] : column axis pixels
        self.width/self.resolution : the number of pixel in a line
        pose[1] : the number of rows
        '''
        robot_origin=int(pose[0])+int(math.ceil(self.width/self.resolution)*pose[1])

        # devide len(scandata) with 2 and plus 1.
        # it mean the center of the ray.
        centreray=len(scandata)/2+1
        # print("len(scandata) : ", len(scandata))
        # print("centreray : ",centreray)
        for i in range(len(scandata)):
            # there is 'inf' (infinite data, it means garbage data.) data in scandata sometimes.
            # and we can skip 'inf' data with following code.
            # isinf: if there is 'inf' data, return true.
            if not math.isinf(scandata[i]):
                # beta : angular distance from centreray
                # angle_increment : angular distance between two points nearby.
                # (i-centreray) : the number of layer from centreray
                beta=(i-centreray)*angle_increment
                # print("beta : ",beta)
                # print("scandata[i] : ", scandata[i])
                # print("float(scandata[i]) : ", float(scandata[i]))

                # pose[2] : it means odometry theta value.
                # px, py are the position of scandata.
                # it's simple geometry.
                # cos(beta-pose[2]) : it means the angle of scandata.
                # float(scandata[i]) : it means the distance of scandata.
                # self.resolution : finally we can resolute the distance.
                # then we can get the position of scandata.
                px=int(float(scandata[i])*cos(beta-pose[2])/self.resolution)
                #print("pose[0] : ", pose[0])
                print("px : ", px)

                py=int(float(scandata[i])*sin(beta-pose[2])/self.resolution)
                print("py : ", py)

                l = bresenham.bresenham([0,0],[px,py])

                # print("l.path : ",l.path)
                # print("len(l.path) of scandata[i] : ",len(l.path)," of ",scandata[i])

                # there are line in 'l' variable.
                # we're gonna
                for j in range(len(l.path)):
                    # lpx, lpy are all the pixel from start point to end point
                    lpx=self.map_origin[0]+pose[0]+l.path[j][0]*self.resolution
                    lpy=self.map_origin[1]+pose[1]+l.path[j][1]*self.resolution

                    # if lpx and lpy are out of boundary do nothing
                    if (0<=lpx<self.width and 0<=lpy<self.height):
                        # get data index
                        index=self.origin+int(l.path[j][0]+math.ceil(self.width/self.resolution)*l.path[j][1])
                        # check index is out of boundary
                        if (index < len(self.logodds)):
                            # check scandata is out of boundary
                            if scandata[i]<self.max_scan_range*range_max:
                                #print(type(l.path))
                                #print("type(self.logodds) : ",type(self.logodds))
                                #print("len(self.logodds) : ",len(self.logodds))
                                #print("index : ",index)

                                # it's about Occupancy grid map.
                                # pfree : it means that pixel is free.
                                # pocc : it means that pixel is occupancy.
                                if(j<len(l.path)-1):self.logodds[index]+=self.pfree
                                else:self.logodds[index]+=self.pocc

                            else:self.logodds[index]+=self.pfree

                            if self.logodds[index]>self.max_logodd:
                                self.logodds[index]=self.max_logodd
                            elif self.logodds[index]<-self.max_logodd:
                                self.logodds[index]=-self.max_logodd

                            if self.logodds[index]>self.max_logodd_belief:
                                self.localmap[index]=100
                            else:
                                self.localmap[index]=0

                            self.localmap[self.origin]=100.0