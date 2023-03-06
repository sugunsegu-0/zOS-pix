#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$


import numpy as np
import math
import cv2
import rclpy
from rclpy.node import Node
from matplotlib import pyplot as plt
from std_msgs.msg import Header
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
k = 0.1  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle

show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)



# overall length of the vehicle (meters)
# length = 4.20
length = 4.88

# overall wheel base of the vehicle (meters)
#wheel_base = 2.75
wheel_base = 2.85
# overhang
overh = (length - wheel_base)/2

# overall width
#width = 1.920
width = 1.910

# maximum steering angle (radians)
beta_max = 0.6

# minimum turning radius (meters)
# R_min = overh/math.sin(beta_max)
R_min = 5

# inner turning radius - minimum
Ri_min = np.sqrt(R_min**2 - wheel_base**2) - (width/2)

# outer turning radius - minimum
Re_min = np.sqrt((Ri_min+width)**2 + (wheel_base+overh)**2)

# minimum length of the parking space required - 1 shot
L_min = overh + np.sqrt(Re_min**2 - Ri_min**2)

flag = 0

def main():
    rclpy.init()

    parallel_park = Parallel_Park()

    rclpy.spin(parallel_park)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    parallel_park.destroy_node()
    rclpy.shutdown()
cam_1_status = ""
cam_2_status = ""
cam_3_status = ""
cam_4_status = ""
cam_5_status = ""
cam_6_status = ""
flag = 0
class Parallel_Park(Node):
    def __init__(self):
        super().__init__('parallel_park')
        self.subscription_cam_1 = self.create_subscription(
            Image,
            'topic',
            self.pose_estimtation_cam_1,
            10)
        self.subscription_cam_1 = self.create_subscription(
            Image,
            'topic',
            self.pose_estimtation_cam_2,
            10)
        self.subscription_cam_2 = self.create_subscription(
            Image,
            'topic',
            self.pose_estimtation_cam_3,
            10)
        self.subscription_cam_3 = self.create_subscription(
            Image,
            'topic',
            self.pose_estimtation_cam_4,
            10)
        self.subscription_cam_4 = self.create_subscription(
            Image,
            'topic',
            self.pose_estimtation_cam_5,
            10)
        self.subscription_cam_5 = self.create_subscription(
            Image,
            'topic',
            self.pose_estimtation_cam_6,
            10)
        self.publisher_ = self.create_publisher(Odometry, '/cmd_vel', 10)
        
    def generate_waypoints(pose,mid_point):
        min_theta = math.asin(0)
        max_theta = math.asin(mid_point[0]-pose[0]/R_min)
        arr = []
        for i in range(min_theta,max_theta,1):
            x = R_min * math.sin(i)
            y = R_min * math.sin(i)
            arr.append([x,y])
        min_theta = math.asin(0)
        max_theta = math.asin(mid_point[0]+pose[0]/R_min)
        
        for i in range(min_theta,max_theta,1):
            x = R_min * math.sin(i)
            y = R_min * math.sin(i)
            arr.append([x,y])
        return arr
    def park_points(ri, current, goal, w):
        r_prime = ri + w/2
        c1 = np.array([goal[0], goal[1] + r_prime])

        x_c1 = c1[0]
        y_c1 = c1[1]

        x_i = current[0]
        y_i = current[1]

        y_s = y_i
        y_c2 = y_s - r_prime

        y_t = (y_c1 + y_c2)/2
        x_t = x_c1 + np.sqrt(r_prime**2 - (y_t - y_c1)**2)

        x_s = 2 * x_t - x_c1

        x_c2 = x_s

        c2 = np.array([x_c2, y_c2])
        i = np.array([x_i, y_i])
        s = np.array([x_s, y_s])
        pt = np.array([x_t, y_t])
        #print r_prime, c1, c2, i, s, pt
        return r_prime, c1, c2, i, s, pt


    def pose_estimtation_cam_1(self,msg): #right cam
        self.br = CvBridge()
        matrix_coefficients=np.load("/path")
        distortion_coefficients=np.load("/path")
        frame = self.br.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get("DICT_ARUCO_ORIGINAL")
        parameters = cv2.aruco.DetectorParameters_create()


        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
            cameraMatrix=matrix_coefficients,
            distCoeff=distortion_coefficients)

            # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                if ids[i]=="3":
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                            distortion_coefficients)
                    cam_1_status = "1"
                    
            # Draw a square around the markers
    def pose_estimtation_cam_2(self,msg):
        self.br = CvBridge()
        matrix_coefficients=np.load("/path")
        distortion_coefficients=np.load("/path")
        frame = self.br.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get("DICT_ARUCO_ORIGINAL")
        parameters = cv2.aruco.DetectorParameters_create()


        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
            cameraMatrix=matrix_coefficients,
            distCoeff=distortion_coefficients)

            # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                        distortion_coefficients)
                cam_2_status = "1"            
    def pose_estimtation_cam_3(self,msg):
        self.br = CvBridge()
        matrix_coefficients=np.load("/path")
        distortion_coefficients=np.load("/path")
        frame = self.br.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get("DICT_ARUCO_ORIGINAL")
        parameters = cv2.aruco.DetectorParameters_create()


        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
            cameraMatrix=matrix_coefficients,
            distCoeff=distortion_coefficients)

            # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                        distortion_coefficients)
                cam_3_status = "1"
    def pose_estimtation_cam_4(self,msg):
        self.br = CvBridge()
        matrix_coefficients=np.load("/path")
        distortion_coefficients=np.load("/path")
        frame = self.br.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get("DICT_ARUCO_ORIGINAL")
        parameters = cv2.aruco.DetectorParameters_create()


        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
            cameraMatrix=matrix_coefficients,
            distCoeff=distortion_coefficients)

            # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                        distortion_coefficients)
                cam_4_status = "1"
    def pose_estimtation_cam_5(self,msg):
        self.br = CvBridge()
        matrix_coefficients=np.load("/path")
        distortion_coefficients=np.load("/path")
        frame = self.br.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get("DICT_ARUCO_ORIGINAL")
        parameters = cv2.aruco.DetectorParameters_create()


        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
            cameraMatrix=matrix_coefficients,
            distCoeff=distortion_coefficients)
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                if ids[i]=="3":
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                            distortion_coefficients)
                    cam_5_status = "1"

    def pose_estimtation_cam_6(self,msg):
        self.br = CvBridge()
        matrix_coefficients=np.load("/path")
        distortion_coefficients=np.load("/path")
        frame = self.br.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get("DICT_ARUCO_ORIGINAL")
        parameters = cv2.aruco.DetectorParameters_create()


        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
            cameraMatrix=matrix_coefficients,
            distCoeff=distortion_coefficients)

            # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                        distortion_coefficients)
                
        
            if ids[i]=="4":
                cam_6_status = "1"
                self.transform = tvec
    def trajectory(center1, center2, initial, start, transition, goal, r):

        x1 = np.linspace(initial[0], start[0])
        y1 = np.linspace(initial[1], initial[1])

        x2 = np.linspace(start[0], transition[0])
        y2 = np.sqrt(r**2 - (x2 - center2[0])**2) + center2[1]

        x3 = np.linspace(transition[0], goal[0])
        y3 = - np.sqrt(r**2 - (x3 - center1[0])**2) + center1[1]

        x = np.append(x1, [x2, x3])
        y = np.append(y1, [y2, y3])
        plt.plot(x1, y1)
        plt.plot(x2, y2)
        plt.plot(x3, y3)
        #plt.plot(x, y)

        plt.show()
            
    

    def calc_distance(a, b):
        dist = np.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)
        return dist
        
        
    def veh_control(initial_pos, start_pos, transition_pos, goal_pos, radius, update_pos):
        
        print(initial_pos, start_pos, transition_pos, goal_pos, radius, update_pos)

        if update_pos[0]>start_pos[0]:
            vel = -12
            steer = 0
            
        elif update_pos[0]<=(start_pos[0]) and update_pos[0]>(transition_pos[0]):
            vel = -104
            steer = -0.49

        elif update_pos[0]<=(transition_pos[0]) and update_pos[0]>(goal_pos[0]-0.5) and flag == 0:
            steer = 0.49
            vel = -104
        elif update_pos[0]<=goal_pos[0]-0.5:
            steer = 0
            vel = 20
            global flag
            flag = 1
            
        elif flag == 1:
            steer = 0
            vel = -5

        return vel, steer    
        
        
    def veh_mission(self):
        
        status = ""
        if cam_1_status == "1" or cam_2_status == "1" or cam_3_status == "1" or cam_4_status == "1":
            status = "move_fow"
        if cam_1_status == "0" and cam_2_status == "0" and cam_3_status == "0" and cam_4_status == "0" and cam_5_status == "1":
            status = "start_manuver"
        pose = np.array[self.transform[0][0][0],self.transform[0][0][1]]
        mid_point = np.array[(pose[0]+0)/2,(pose[1]+0.75)/2]
        dist = self.calc_distance(pose,mid_point)
        if status == "move_fow":
            msg = Odometry()
            msg._header = Header()
            msg._twist._twist._linear._x=1
            msg._twist._twist._linear._y=0
            self.publisher_.publish(msg)
      
                
        if dist <= (R_min+1):
            msg = Odometry()
            msg._header = Header()
            msg._twist._twist._linear._x=1
            msg._twist._twist._linear._y=0
            self.publisher_.publish(msg)
            flag = 1
        if flag == 1:
            waypoint = self.generate_waypoints(pose,mid_point)
            target_speed = 0.2  # [m/s]

            T = 100.0  # max simulation time
            cx = waypoint[:][0]
            cy = waypoint[:][1]
            # initial state
            state = State(x=cx[0], y=cy[0], yaw=0.0, v=0.0)

            lastIndex = len(cx) - 1
            time = 0.0
            states = States()
            states.append(time, state)
            target_course = TargetCourse(cx, cy)
            target_ind, _ = target_course.search_target_index(state)

            while T >= time and lastIndex > target_ind:

                # Calc control input
                ai = proportional_control(target_speed, state.v)
                di, target_ind = pure_pursuit_steer_control(
                    state, target_course, target_ind)
                msg = Odometry()
                msg._header = Header()
                msg._twist._twist._linear._x=ai
                msg._twist._twist._angular._z=di
                self.publisher_.publish(msg)
                state.update(ai, di)  # Control vehicle

                time += dt
                states.append(time, state)

                if show_animation:  # pragma: no cover
                    plt.cla()
                    # for stopping simulation with the esc key.
                    plt.gcf().canvas.mpl_connect(
                        'key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                    plot_arrow(state.x, state.y, state.yaw)
                    plt.plot(cx, cy, "-r", label="course")
                    plt.plot(states.x, states.y, "-b", label="trajectory")
                    plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                    plt.axis("equal")
                    plt.grid(True)
                    plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                    plt.pause(0.001)

            # Test
            assert lastIndex >= target_ind, "Cannot goal"
        
        

        



        
        

if __name__ == "__main__":
    
    main()    
    
    
