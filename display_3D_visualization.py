#!/usr/bin/env python
import rospy
from vpython import *
from math import *
import wx
from sensor_msgs.msg import Imu
rad2degrees = 180.0/pi
precision = 2 
yaw_offset = 0

def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)
    return [roll, pitch, yaw]

#def shutdown_hook():
#    wx.Exit()

def processIMU_message(imuMsg):
    global yaw_offset
    roll=0
    pitch=0
    yaw=0
    quaternion = (imuMsg.orientation.x,imuMsg.orientation.y,imuMsg.orientation.z,imuMsg.orientation.w)
    data= quaternion_to_euler(imuMsg.orientation.x,imuMsg.orientation.y,imuMsg.orientation.z,imuMsg.orientation.w)
    roll=data[0]
    pitch=data[1]
    yaw=data[2]
    yaw += yaw_offset
    print(roll)
    print(pitch)
    print(yaw)
    axis=vector(-cos(pitch)*cos(yaw),-cos(pitch)*sin(yaw),sin(pitch))
    up=vector(((sin(roll)*sin(yaw))+(cos(roll)*sin(pitch)*cos(yaw))),((-sin(roll)*cos(yaw))+(cos(roll)*sin(pitch)*sin(yaw))),(cos(roll)*cos(pitch)))
    platform.axis=axis
    platform.up=up
    platform.length=1.0
    plat_arrow_up.axis=up
    plat_arrow_up.up=axis
    plat_arrow_up.length=0.4
    plat_arrow.axis=axis
    plat_arrow.up=up
    plat_arrow.length=-0.8
    p_line.axis=axis
    p_line.up=up
    p_line.length=1.1
    cil_roll.axis=vector(-0.2*cos(roll),0.2*sin(roll),0)
    cil_roll2.axis=vector(0.2*cos(roll),-0.2*sin(roll),0)
    cil_pitch.axis=vector(0,-0.4*sin(pitch),-0.4*cos(pitch))
    arrow_course.axis=vector(-0.2*sin(yaw-yaw_offset),0.2*cos(yaw-yaw_offset),0)
    rollLabel.text = str(round(roll*rad2degrees, precision)) + " / " + str(round(roll,precision))
    pitchLabel.text = str(round(pitch*rad2degrees, precision)) + " / " + str(round(pitch, precision))
    yawLabel.text = str(round((yaw-yaw_offset)*rad2degrees, precision)) + " / " + str(round((yaw-yaw_offset), precision))
    linAccLabel.text = str(round(imuMsg.linear_acceleration.x, precision)) + " / " + str(round(imuMsg.linear_acceleration.y, precision)) + " / " + str(round(imuMsg.linear_acceleration.z, precision))
    angVelLabel.text = str(round(imuMsg.angular_velocity.x, precision)) + " / " + str(round(imuMsg.angular_velocity.y, precision)) + " / " + str(round(imuMsg.angular_velocity.z, precision))
#    if scene.waitfor('keydown'):
#            yaw_offset += -yaw

rospy.init_node("display_3D_visualization_node")
sub = rospy.Subscriber('imu', Imu, processIMU_message)
scene=canvas(title="9DOF Razor IMU Main Screen")
scene.range=1.3
scene.forward =vector(1,0,-0.25)
scene.up=vector(0,0,1)
scene.width=500
scene.height=500
scene2 = canvas(title='9DOF Razor IMU Roll, Pitch, Yaw',x=550, y=0, width=500, height=500,center=vector(0,0,0), background=color.black)
scene2.range=1
scene2.select()
cil_roll = arrow(pos=vector(-0.5,0.3,0),axis=vector(0.2,0,0),shaftwidth=0.1,color=color.red)
cil_roll2 = cylinder(pos=vector(-0.5,0.3,0),axis=vector(-0.2,0,0),radius=0.01,color=color.red)
cil_pitch = arrow(pos=vector(0.5,0.3,0),axis=vector(0,0,-0.4),shaftwidth=0.02,color=color.green)
arrow_course = arrow(pos=vector(0.0,-0.4,0),color=color.cyan,axis=vector(0,0.2,0), shaftwidth=0.02)
label(pos=vector(-0.5,0.6,0),text="Roll (degrees / radians)",box=0,opacity=0)
label(pos=vector(0.5,0.6,0),text="Pitch (degrees / radians)",box=0,opacity=0)
label(pos=vector(0.0,0.02,0),text="Yaw (degrees / radians)",box=0,opacity=0)
label(pos=vector(0.0,-0.16,0),text="N",box=0,opacity=0,color=color.yellow)
label(pos=vector(0.0,-0.64,0),text="S",box=0,opacity=0,color=color.yellow)
label(pos=vector(-0.24,-0.4,0),text="W",box=0,opacity=0,color=color.yellow)
label(pos=vector(0.24,-0.4,0),text="E",box=0,opacity=0,color=color.yellow)
label(pos=vector(0.18,-0.22,0),height=7,text="NE",box=0,color=color.yellow)
label(pos=vector(-0.18,-0.22,0),height=7,text="NW",box=0,color=color.yellow)
label(pos=vector(0.18,-0.58,0),height=7,text="SE",box=0,color=color.yellow)
label(pos=vector(-0.18,-0.58,0),height=7,text="SW",box=0,color=color.yellow)
rollLabel = label(pos=vector(-0.5,0.52,0),text="-",box=0,opacity=0,height=12)
pitchLabel = label(pos=vector(0.5,0.52,0),text="-",box=0,opacity=0,height=12)
yawLabel = label(pos=vector(0,-0.06,0),text="-",box=0,opacity=0,height=12)
label(pos=vector(0,0.9,0),text="Linear Acceleration x / y / z (m/s^2)",box=0,opacity=0)
label(pos=vector(0,-0.8,0),text="Angular Velocity x / y / z (rad/s)",box=0,opacity=0)
linAccLabel = label(pos=vector(0,0.82,0),text="-",box=0,opacity=0,height=12)
angVelLabel = label(pos=vector(0,-0.88,0),text="-",box=0,opacity=0,height=12)
scene.select()
arrow(color=color.green,axis=vector(1,0,0), shaftwidth=0.04, fixedwidth=1)
arrow(color=color.green,axis=vector(0,1,0), shaftwidth=0.04 , fixedwidth=1)
arrow(color=color.green,axis=vector(0,0,1), shaftwidth=0.04, fixedwidth=1)
label(pos=vector(0,0,-1.2),text="Press 'a' to align",box=0,opacity=0)
label(pos=vector(1,0.1,0),text="X",box=0,opacity=0)
label(pos=vector(0,1,-0.1),text="Y",box=0,opacity=0)
label(pos=vector(0,-0.1,1),text="Z",box=0,opacity=0)
platform = box(length=1.0, height=0.05, width=0.65, color=color.red,up=vector(0,0,1),axis=vector(-1,0,0))
p_line = box(length=1.1,height=0.08,width=0.1,color=color.yellow,up=vector(0,0,1),axis=vector(-1,0,0))
plat_arrow = arrow(length=-0.8,color=color.cyan,up=vector(0,0,1),axis=vector(-1,0,0), shaftwidth=0.06, fixedwidth=1)
plat_arrow_up = arrow(length=0.4,color=color.cyan,up=vector(-1,0,0),axis=vector(0,0,1), shaftwidth=0.06, fixedwidth=1)
if __name__ == "__main__":
	try:
		#rospy.on_shutdown(shutdown_hook)


		rospy.spin()
	except KeyboardInterrupt:
        	print ('\nShutdown requested. Exiting...')


