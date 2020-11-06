#!/usr/bin/python

from __future__ import print_function
import rospy
from std_msgs.msg import Bool
import numpy as np
from   rospy.numpy_msg import numpy_msg
from tf.transformations import rotation_matrix 

import tf2_ros
import tf_conversions

from geometry_msgs.msg import TransformStamped,Twist
from pid_control import controller

class control_node(object):
    def __init__(self):
        self.init_vars()

        self.rate=rospy.Rate(10)

        self.init_first_TM(np.deg2rad(-90))


        if (not self.initial_calibration()):
            return
        
        self.calc_odom_transf()

        while (not rospy.is_shutdown() and (np.linalg.norm(self.actual_pos-self.goal)>0.1 or len(self.point_set)>0)):
            self.get_odom()
            
            self.calc_odom_transf()
            
            if (np.linalg.norm(self.actual_pos-self.goal)<0.1 and len(self.point_set)>0):
                self.goal=np.array([self.point_set.pop(0)],dtype=np.float)
                self.goal=np.transpose(self.goal)
            
            pos_err=self.goal - self.actual_pos
            err_in_base=np.matmul(self.rot_origin_2_base,pos_err)

            print("error en base", err_in_base)

            ang_err=np.arctan2(err_in_base[1],err_in_base[0])[0]
            print("Posicion actual ",end="")
            print(self.actual_pos)
            print("Error de posicion ",end="")
            print(np.linalg.norm(pos_err))
            print("Error angulo ",end="")
            print(ang_err)
            print(self.original_fr)

            self.control_conditional_action(ang_err,pos_err)

            self.rate.sleep()

    def init_callback(self,msg):
        self.init_flag=msg.data

    def init_vars(self):
        self.init_flag=False
        self.actual_pos=np.zeros((2,1),dtype=np.float)
        rospy.Subscriber("tll2/begin_control",Bool,callback=self.init_callback,queue_size=10)
        self.controllers=controller(0.4,1.0,0.3,np.pi/3)
        self.point_set=[[0, 0], [-3.5, 0], [-3.5, 3.5], [1.5, 3.5], [1.5, -1.5], [3.5, -1.5]]
        self.point_set+=[[3.5, -8.0], [-2.5, -8.0], [-2.5, -5.5], [1.5, -5.5], [1.5, -3.5],[-1.0, -3.5]]
        # Para realizar un broadcast
        self.broadcts  = tf2_ros.TransformBroadcaster()
        self.transform = TransformStamped()

        # Para realizar la escucha "listener"
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.goal=np.array([self.point_set.pop(0)],dtype=np.float)
        self.goal=np.transpose(self.goal)

        self.velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        self.tw_msg=Twist()

    def init_first_TM(self,angle):
        self.original_fr=np.identity(4,dtype=np.float)
        rot=[[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]
        self.original_fr[0:2,0:2]=np.array(rot,dtype=np.float)
        print(self.original_fr)

    def initial_calibration(self):
        i=0
        while i<10:
            try:
                self.trans_odom = self.tfBuffer.lookup_transform("odom","base_footprint", rospy.Time())
                i=15
            except:
                i+=1
                self.rate.sleep()
                rospy.logwarn("Error trying to look for transform")
        if (i<15):
            return False
        return True

    def calc_odom_transf(self):
        self.quat = np.array([self.trans_odom.transform.rotation.x, \
                                    self.trans_odom.transform.rotation.y, \
                                    self.trans_odom.transform.rotation.z, \
                                    self.trans_odom.transform.rotation.w])
        self.rot_matr_base_odom = tf_conversions.transformations.quaternion_matrix(self.quat)
        self.rot_matr_base_odom = self.rot_matr_base_odom.copy()
        self.rot_matr_base_odom[0,3]=self.trans_odom.transform.translation.x
        self.rot_matr_base_odom[1,3]=self.trans_odom.transform.translation.y
        self.rot_matr_base_odom[2,3]=self.trans_odom.transform.translation.z

        self.base_2_origin= np.matmul(self.original_fr,self.rot_matr_base_odom)
        
        self.rot_origin_2_base = self.base_2_origin.copy()[0:2,0:2]
        self.rot_origin_2_base=np.transpose(self.rot_origin_2_base)

        self.actual_pos[:,0]=self.base_2_origin[0:2,3]

    def get_odom(self):
        try:
            self.trans_odom = self.tfBuffer.lookup_transform("odom","base_footprint", rospy.Time())
        except:
            rospy.logwarn("Error trying to look for transform")

    def control_conditional_action(self,ang_err,pos_err):
        self.tw_msg.linear.x=0
        self.tw_msg.angular.z=0
        ang_cmd=self.controllers.angular_control(ang_err)
        print("ang_cmd",ang_cmd)
        self.tw_msg.angular.z=ang_cmd
        if (abs(ang_err)<0.01):
            vel_cmd=self.controllers.linear_control(np.linalg.norm(pos_err))
            self.tw_msg.linear.x=vel_cmd
        self.velocity_pub.publish(self.tw_msg)

    def control_complete_action(self,ang_err,pos_err):
        self.tw_msg.linear.x=0
        self.tw_msg.angular.z=0
        ang_cmd=self.controllers.angular_control(ang_err)
        self.tw_msg.angular.z=ang_cmd
        vel_cmd=self.controllers.linear_control(np.linalg.norm(pos_err))
        self.tw_msg.linear.x=vel_cmd