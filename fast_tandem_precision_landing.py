#!/usr/bin/env python

########IMORTS#########

import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
import heapq
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from array import array
from vector_math import Vector2


##### MARKER #####

class Marker:

    def __init__(self, id, priority, dim, offset=Vector2()):

        self.id = id
        self.priority = priority
        self.dim = dim
        self.offset = offset


#######VARIABLES########

vehicle = connect('tcp:127.0.0.1:5763',wait_ready=True)
vehicle.parameters['PLND_ENABLED']=1
vehicle.parameters['PLND_TYPE']=1
vehicle.parameters['PLND_EST_TYPE']=0
vehicle.parameters['LAND_SPEED']=30 ##cms/s

velocity=-.5 #m/s
takeoff_height=4 #m

########################
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)


aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# Init ArUcos

aruco_markers = {
                    72 : Marker(72, 1, 0.02), # Center
                    138 : Marker(138, 2, 0.16, Vector2(0.5,0)), # Left Wing
                    48 : Marker(48, 2, 0.16, Vector2(-0.5,0)), # Right Wing
                    62 : Marker(62, 3, 0.1, Vector2(0, 0.7)) # Tail
                }

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180) ##62.2 for picam V2, 53.5 for V1
vertical_fov = 48.8 * (math.pi / 180) ##48.8 for V2, 41.41 for V1

found_count=0
notfound_count=0

#############CAMERA INTRINSICS#######

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

######################################:3
time_last=0
time_to_wait = .1 ##100 ms
################FUNCTIONS###############
def arm_and_takeoff(targetHeight):
    while vehicle.is_armable !=True:
        print('Waiting for vehicle to become armable')
        time.sleep(1)
    print('Vehicle is now armable')

    vehicle.mode = VehicleMode('GUIDED')

    while vehicle.mode !='GUIDED':
        print('Waiting for drone to enter GUIDED flight mode')
        time.sleep(1)
    print('Vehicle now in GUIDED mode. Have Fun!')

    vehicle.armed = True
    while vehicle.armed ==False:
        print('Waiting for vehicle to become armed.')
        time.sleep(1)
    print('Look out! Virtual props are spinning!')

    vehicle.simple_takeoff(targetHeight)

    while True:
        print('Current Altitude: %d'%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >=.95*targetHeight:
            break
        time.sleep(1)
    print('Target altitude reached!')

    return None


### Unneccesary for Real Implementation
##Send velocity command to drone
def send_local_ned_velocity(vx,vy,vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,0,0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

### Unneccsary for Real Implementation
def send_land_message(x,y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,0,0
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()



def msg_receiver(message):
    ### Unneccesary for Real Implementation
    global notfound_count, found_count, time_last, time_to_wait, aruco_markers, previous_priority, priority_buffer

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message) ##Deserialize image data into array
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        ### We need this part onward
        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

        try:
            if ids is not None:
                tracking_aruco = 0 # Index into ids of tracked marker
                #print("IDs: " + str(ids))
                for i in range(len(ids)):
                    old_p = aruco_markers[int(ids[tracking_aruco])].priority
                    p = aruco_markers[int(ids[i])].priority
                    if p < old_p:
                        tracking_aruco = int(i)

                follow_marker = aruco_markers[int(ids[tracking_aruco])]

                # ret = aruco.estimatePoseSingleMarkers(corners,(follow_marker.dim * 100),cameraMatrix=np_camera_matrix,distCoeffs=np_dist_coeff)
                # (rvec, tvec) = (ret[0][0,0,:],ret[1][0,0,:])
                #
                # x = '{:.2f}'.format(tvec[0]) ### Xerror/distance between camera and aruco in CM
                # y = '{:.2f}'.format(tvec[1]) ### Yerror/distance between camera and aruco in CM
                # z = '{:.2f}'.format(tvec[2]) ### Zerror/distance between camera and aruco in CM

                topLeft = Vector2(int(corners[tracking_aruco][0][0][0]), int(corners[tracking_aruco][0][0][1]))
                topRight = Vector2(int(corners[tracking_aruco][0][1][0]), int(corners[tracking_aruco][0][1][1]))
                bottomLeft = Vector2(int(corners[tracking_aruco][0][3][0]), int(corners[tracking_aruco][0][3][1]))
                bottomRight = Vector2(int(corners[tracking_aruco][0][2][0]), int(corners[tracking_aruco][0][2][1]))
                topCenter = (topLeft + topRight) / 2
                centerRight = (topRight + bottomRight) / 2

                sum = topLeft + topRight + bottomLeft + bottomRight

                center = sum / 4

                axes_x = (centerRight - center).normalize()
                axes_y = (topCenter - center).normalize()
                axes = axes_x + axes_y

                dist = Vector2((centerRight - center).magnitude, (topCenter - center).magnitude)
                scale_factor = dist / (follow_marker.dim / 2)

                offset_mag = follow_marker.offset * scale_factor

                offset_pos = center + (axes_x * offset_mag.x) + (axes_y * offset_mag.y)

                #if this is center, wh not just move the center_marker_72

                x_ang = (offset_pos.x - horizontal_res*.5)*horizontal_fov/horizontal_res   ### X angle error
                y_ang = (offset_pos.y - vertical_res*.5)*vertical_fov/vertical_res         ### Y angle error

                # we could camcel the script once the aruco is lost after having been IDed for so long.
                if vehicle.mode !='LAND':
                    vehicle.mode = VehicleMode('LAND')
                    while vehicle.mode !='LAND':
                        time.sleep(1)
                    print('Vehicle in LAND mode')
                    send_land_message(x_ang,y_ang)
                else:
                    send_land_message(x_ang,y_ang)

                #marker_position = 'MARKER POSITION: x='+x+' y='+y+' z='+z

                aruco.drawDetectedMarkers(np_data,corners)

                #aruco.drawAxis(np_data,np_camera_matrix,np_dist_coeff,rvec,tvec,10)
                cv2.line(np_data, (int(center.x), int(center.y)), (int(topCenter.x), int(topCenter.y)), (0, 255, 0), 2)
                cv2.line(np_data, (int(center.x), int(center.y)), (int(centerRight.x), int(centerRight.y)), (0, 0, 255), 2)
                cv2.circle(np_data, (int(offset_pos.x), int(offset_pos.y)), 4, (252, 186, 3), -1)
                cv2.putText(np_data, str(follow_marker.id),(int(offset_pos.x + 10), int(offset_pos.y + 10)),0,.7,(252, 186, 3),thickness=2)

                ##putText(image, text_to_draw,position,font_face,fontScale,color,thickness)
                #cv2.putText(np_data,marker_position,(10,50),0,.7,(255,0,0),thickness=2)
                cv2.putText(np_data,"Tracking ID: " + str(ids[tracking_aruco]),(10,50),0,.7,(255,0,0),thickness=2)

                # Console Printes
                #print(marker_position)
                print('FOUND COUNT: '+str(found_count)+ ' NOTFOUND COUNT: '+str(notfound_count))

                found_count = found_count + 1

            else:
                notfound_count=notfound_count+1
        except Exception as e:
            print('Target likely not found')
            print(e)
            notfound_count=notfound_count+1
        new_msg = rnp.msgify(Image, np_data,encoding='rgb8')
        newimg_pub.publish(new_msg)
        time_last = time.time()
    else:
        return None

# Not needed
def subscriber():
    rospy.init_node('drone_node',anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    rospy.spin()


if __name__=='__main__':
    try:
        arm_and_takeoff(takeoff_height)
        time.sleep(1)
        send_local_ned_velocity(0,velocity,0)
        time.sleep(10)
        subscriber()
    except rospy.ROSInterruptException:
        pass
