#!/usr/bin/python

from __future__ import print_function
import rospy
import numpy as np
from   rospy.numpy_msg import numpy_msg
from tf.transformations import rotation_matrix 

import tf2_ros
import tf_conversions

from geometry_msgs.msg import TransformStamped,Twist
from pid_control import controller
#from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

class control_node(object):
    def __init__(self):
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

        i=0
        rate=rospy.Rate(10)

        self.original_fr=np.zeros((4,4),dtype=np.float)
        self.original_fr[3,3]=1
        self.original_fr[2,2]=1
        self.original_fr[1,0]=-1
        self.original_fr[0,1]=1
        print(self.original_fr)

        while i<10:
            try:
                trans_odom = self.tfBuffer.lookup_transform("base_footprint","odom", rospy.Time())
                i=15
            except:
                i+=1
                rate.sleep()
                rospy.logwarn("Error trying to look for transform")
        if (i<15):
            return

        self.quat = np.array([trans_odom.transform.rotation.x, \
                                    trans_odom.transform.rotation.y, \
                                    trans_odom.transform.rotation.z, \
                                    trans_odom.transform.rotation.w])
        self.rot_matr_base_odom = tf_conversions.transformations.quaternion_matrix(self.quat)
        self.rot_matr_base_odom = self.rot_matr_base_odom.copy()
        self.rot_matr_base_odom[0,3]=trans_odom.transform.translation.x
        self.rot_matr_base_odom[1,3]=trans_odom.transform.translation.y
        self.rot_matr_base_odom[2,3]=trans_odom.transform.translation.z

        self.base_2_origin= np.matmul(self.original_fr,self.rot_matr_base_odom)
        
        self.actual_pos=np.zeros((2,1),dtype=np.float)
        self.actual_pos[:,0]=self.base_2_origin[0:2,3]

        self.velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        self.tw_msg=Twist()
        pos_err_ext=np.zeros((4,1),dtype=np.float)
        pos_err_ext[3]=1
        while (not rospy.is_shutdown() and (np.linalg.norm(self.actual_pos-self.goal)>0.1 or len(self.point_set)>0)):
            #rospy.loginfo("bien")
            try:
                trans_odom = self.tfBuffer.lookup_transform("odom","base_footprint", rospy.Time())
            except:
                rospy.logwarn("Error trying to look for transform")
                #return 
            
            self.quat = np.array([trans_odom.transform.rotation.x, \
                                    trans_odom.transform.rotation.y, \
                                    trans_odom.transform.rotation.z, \
                                    trans_odom.transform.rotation.w])
            self.rot_matr_base_odom = tf_conversions.transformations.quaternion_matrix(self.quat)
            self.rot_matr_base_odom = self.rot_matr_base_odom.copy()
            self.rot_matr_base_odom[0,3]=trans_odom.transform.translation.x
            self.rot_matr_base_odom[1,3]=trans_odom.transform.translation.y
            self.rot_matr_base_odom[2,3]=trans_odom.transform.translation.z

            self.base_2_origin = np.matmul(self.original_fr,self.rot_matr_base_odom)
            rot_origin_2_base = self.base_2_origin.copy()[0:2,0:2]
            rot_origin_2_base=np.transpose(rot_origin_2_base)
            print(rot_origin_2_base)

            self.actual_pos[:,0]=self.base_2_origin[0:2,3]
            if (np.linalg.norm(self.actual_pos-self.goal)<0.1 and len(self.point_set)>0):
                #(np.linalg.norm(self.actual_pos-self.goal)<0.6 and len(self.point_set)>0):
                #(np.linalg.norm(self.actual_pos-self.goal)<0.1 and len(self.point_set)>0):
                self.goal=np.array([self.point_set.pop(0)],dtype=np.float)
                self.goal=np.transpose(self.goal)
            
            pos_err=self.goal - self.actual_pos
            #pos_err_ext[0:2,0]=pos_err[:,0]
            err_in_base=np.matmul(rot_origin_2_base,pos_err)

            ang_err=np.arctan2(err_in_base[1],err_in_base[0])[0]
            print("Posicion actual ",end="")
            print(self.actual_pos)
            print("Error de posicion ",end="")
            print(np.linalg.norm(pos_err))
            print("Error angulo ",end="")
            print(ang_err)
            self.tw_msg.linear.x=0
            self.tw_msg.angular.z=0
            ang_cmd=self.controllers.angular_control(ang_err)
            #self.tw_msg.linear.x=0
            self.tw_msg.angular.z=ang_cmd
            #self.velocity_pub.publish(self.tw_msg)
            if (abs(ang_err)<0.01):
                vel_cmd=self.controllers.linear_control(np.linalg.norm(pos_err))
                #print(vel_cmd)
                self.tw_msg.linear.x=vel_cmd
                #self.tw_msg.angular.z=0
                #self.velocity_pub.publish(self.tw_msg)
            self.velocity_pub.publish(self.tw_msg)
            rate.sleep()
            #if(i>=20):
            #    break
            #i+=1