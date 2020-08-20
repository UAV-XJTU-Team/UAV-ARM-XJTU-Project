#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from my_flight_demo.msg import Visual_msg
from cv2 import aruco
import math

def Setcamera(cap):
    cap.set(6,cv2.VideoWriter_fourcc('M','J','P','G'))   #different from opencv4
    cap.set(5,100)
    cap.set(3,640)
    cap.set(4,480)

def rotateByZ(x,y,thetaz):
    rz=thetaz*math.pi/180
    outx=math.cos(rz)*x-math.sin(rz)*y
    outy=math.sin(rz)*x+math.cos(rz)*y
    return outx,outy

def rotateByY(x,z,thetay):
    ry=thetay*math.pi/180
    outx=math.cos(ry)*x+math.sin(ry)*z
    outz=math.cos(ry)*z-math.sin(ry)*x
    return outx,outz

def rotateByX(y,z,thetax):
    rx=thetax*math.pi/180
    outy=math.cos(rx)*y-math.sin(rx)*z
    outz=math.cos(rx)*z+math.sin(rx)*y
    return outy,outz

def xyz(r,t):
    t=t[0,0]
    rmat,_=cv2.Rodrigues(r)
    thetaz=math.atan2(rmat[1,0],rmat[0,0])/math.pi*180
    thetay=math.atan2(-1*rmat[2,0],math.sqrt(rmat[2,1]*rmat[2,1]+rmat[2,2]*rmat[2,2]))/math.pi*180
    thetax=math.atan2(rmat[2,1],rmat[2,2])/math.pi*180
    tx=t[0]
    ty=t[1]
    tz=t[2]

    x,y=rotateByZ(tx,ty,-1*thetaz)
    x,z=rotateByY(x,tz,-1*thetay)
    y,z=rotateByX(y,z,-1*thetax)

    x=-x
    y=-y
    z=-z
    return x,y,z

# cap=cv2.VideoCapture(2)
# Setcamera(cap)

# cameraMartix=np.array([[720.2120,2.0761,335.1827],
#                        [0,721.1528,268.9376],
#                        [0,0,1]])

# distCoeffs=np.array([-0.1622,0.1400,0.00234976,-0.0008216553])

# dict=aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

def detecter():
    cap=cv2.VideoCapture(2)
    Setcamera(cap)

    cameraMartix=np.array([[720.2120,2.0761,335.1827],
                            [0,721.1528,268.9376],
                            [0,0,1]])

    distCoeffs=np.array([-0.1622,0.1400,0.00234976,-0.0008216553])

    dict=aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    pub = rospy.Publisher('visual', Visual_msg, queue_size=10)
    rospy.init_node('detecter', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        ret,frame=cap.read()
        img=frame
        corners, ids, reject = aruco.detectMarkers(frame, dict)
        if ids is not None:
            img=aruco.drawDetectedMarkers(img,corners,ids)
            r,t=aruco.estimatePoseSingleMarkers(corners,0.95,cameraMartix,distCoeffs)
            position=xyz(r,t)
            # rospy.loginfo("r:\n",r)
            # rospy.loginfo("t:\n",t)
            for i,id in enumerate(ids):
                img=aruco.drawAxis(img,cameraMartix,distCoeffs,r[i],t[i],1)
                msg=Visual_msg()
                msg.id=id
                marker_r=r[i][0]
                marker_t=t[i][0]
                msg.rvec.x=marker_r[0]
                msg.rvec.y=marker_r[1]
                msg.rvec.z=marker_r[2]
                msg.tvec.x=position[0]
                msg.tvec.y=position[1]
                msg.tvec.z=position[2]
                rospy.loginfo("detect marker")
                pub.publish(msg)
                rate.sleep()
            cv2.imshow("img",img)
            cv2.waitKey(1)
        else:
            cv2.imshow("img",frame)
            cv2.waitKey(1)
            msg=Visual_msg()
            msg.id=100
            msg.rvec.x=0
            msg.rvec.y=0
            msg.rvec.z=0
            msg.tvec.x=0
            msg.tvec.y=0
            msg.tvec.z=0
            rospy.loginfo("detect nothing")
            rospy.loginfo(msg)
            pub.publish(msg)

if __name__ == '__main__':
    try:
        detecter()
    except rospy.ROSInterruptException:
        pass

# while(1):
#     ret,frame=cap.read()
#     img=frame
#     corners, ids, reject = aruco.detectMarkers(frame, dict)
#     if ids is not None:
#         img=aruco.drawDetectedMarkers(img,corners,ids)
#         r,t,obj=aruco.estimatePoseSingleMarkers(corners,0.95,cameraMartix,distCoeffs)
#         print("r:\n",r)
#         print("t:\n",t)
#         for i in range(ids.size):
#             img=aruco.drawAxis(img,cameraMartix,distCoeffs,r[i],t[i],1)
#     cv2.imshow("img",img)
#     if cv2.waitKey(1) & 0xFF == 27:
#         break