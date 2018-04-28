"""  
 * Team Id: EYRC<3921> 
 * Author List: G.Gautam,G.Rajinikanth,G.Tarun,K.Rithesh
 * Filename: Collector bot 
 * Theme: Collector bot implementation
 * Functions: map,pwm,pid,send_to_cb,setobj,choose_a_goal, set_reset_goal,motion, go_to_truck,track_truck
 * Global Variables:oe,ei,wheel_separation,wheel_radius
 */
""" 


import cv2
import sys
import numpy as np
import math
import cv2.aruco as aruco
import time
import vrep
import serial

#################################################################
#################    'u' for arm lift         ###################                
#################    'd' for arm drop         ###################
#################    'g' for grip or catch    ###################
#################    'l' for leave object     ###################
#################################################################
#####|WORKING|##################
#width of pixel=0.003351064
#height of pixel=0.003389831

#Define camera matrix
#k=np.array( [[ 137.03452147,0.  , 717.48581284],
#            [ 0.  ,189.51227963 , 181.45475449],
# 	     [ 0., 0., 1. ]])
k=np.array([[ 532.79548556 ,0., 342.4585458 ],
 	    [ 0.,532.91940012,  233.90077597],
 	    [ 0.,0.,1]])
#Distortion coefficient d
#d=np.array( [ 0.30197851, -0.03050747, -0.15104799,  0.00679967 , 0.00139066])
d=np.array( [ -2.81087759e-01,2.72740676e-02,1.21666147e-03,-1.34227508e-04,1.58476349e-01])
correct_time=0.0
count=0
oe=0
ei=0
wheel_seperation_bot=0.154
wheel_radius_bot=0.0325
###########################################################################################################################################
"""
* Function Name: Map
* Input        : angular velocity(x),minimum angular velocity recorded(in_min), maximum angular velocity recorded(in_max),min pwm(out_min(i.e 16), maximum pwm (out_max(i.e 255)))
*Output        : PWM
* Logic        : This function takes the angular velocity and converts it to the required pwm from range 0 to 255 in this case
*Example call  : map(0.081, 0.051, 8.687, 15,255)
"""
def map(x, in_min, in_max, out_min, out_max):
    return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

##############################################################################################################################################
"""
* Function Name: pwm (PULSE-WIDTH-MODULATION)
* Input        : angular velocity(w)
*Output        : required pwm to set
* Logic        : This function takes the angular velocity and converts it to the required pwm from range 0 to 255 in this case
*Example call  : pwm(0.081)
"""

def pwm(w):
	pwm_min=16
	if w<0:
		w=abs(w)
		pwm=map(w, 0.051, 8.687, 15,255)
		if pwm<16:
			return -1*pwm_min 

		r_pwm=round(pwm)
		r_pwm=-1*r_pwm
		return r_pwm
	if w==0:
		return 0
	if w>0:
		pwm=map(w, 0.051, 8.687,15,255)
		if pwm<16:
			return pwm_min 
		r_pwm=round(pwm)
		return r_pwm
		
###########################################################################################################################################
"""
* Function Name: pid controller
* Input        : e=error ei=integral error oe= old error
*Output        : sets angular velocity
* Logic        : This function gives angular velocity 
"""

def pid(e,oe,ei):
	Kp=0.5
	Ki=0.3
	Kd=0.3
	e_der=e-oe
	ei=ei+e
	w=Kp*e+Kd*e_der+Ki*ei
	oe=e
	return w 
	
############################################################################################################################################
"""
* Function Name: send_to_cb
* Input        : left wheel angular velocity (w_l), right wheel angular velocity(w_r),b and c just for debugging(now required) 
*Output        : sends the velocities through XIGBEE to the differential drive robot
* Logic        : This function send pwm for two seperate wheels to the robot 
*Example call  : pwm(0.081)
"""

data=serial.Serial('/dev/ttyUSB0',9600)
def send_to_cb(w_l,w_r,b,c):
	r=pwm(w_r)
	l=pwm(w_l)
	b.append(r)
	c.append(l)	
	r=round(r)
	l=round(l)
	r=int(r)
	l=int(l)
	
	r=str(r)
	l=str(l)	
	string=l+','+r+';'
	a=data.write(string)
	return
#############################################################################################################################################
"""
* Function Name: setobj-----seting object in the vrep scene
* Input        : flag
*Output        : change the image coordinates to the vrep scene coordinates 
* Logic        : This function takes flag as input . if flag =0 it sets all the objects when flag = 1 it cares only about the position of robot
*Example call  : setobj(1)
"""

def setobj(flag):
	cap=cv2.VideoCapture(1)
	w=cap.get(cv2.CAP_PROP_FRAME_WIDTH)
	h=cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
#	print w
#	print h
	time.sleep(0)
	while True:
		ret,frame=cap.read()

		framebw=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		aruco_dict=aruco.Dictionary_get(aruco.DICT_5X5_250)
		parameters=aruco.DetectorParameters_create()
		corners,ids,_=aruco.detectMarkers(framebw,aruco_dict,parameters=parameters)
#		print ids
		t=len(ids)  #gives total aruco markers
#		print t
		l=0
		l1=0
		for i in range (0,t):
			z=int(ids[i][0])
#			print'>>>>>>>'
			cv2.circle(frame,(corners[i][0][0][0],corners[i][0][0][1]),5,(125,125,125),-1)
			cv2.circle(frame,(corners[i][0][1][0],corners[i][0][1][1]),5,(0,255,0),-1)
			cv2.circle(frame,(corners[i][0][2][0],corners[i][0][2][1]),5,(180,105,255),-1)
			cv2.circle(frame,(corners[i][0][3][0],corners[i][0][3][1]),5,(255,255,255),-1)
			x=((corners[i][0][0][0]+corners[i][0][2][0])/2)
			y=((corners[i][0][0][1]+corners[i][0][2][1])/2)
			cv2.circle(frame,(int(x),int(y)),5,(255,0,0),-1) #center
			x1=(corners[i][0][0][0]+corners[i][0][1][0])/2
			y1=(corners[i][0][0][1]+corners[i][0][1][1])/2
			cv2.circle(frame,(int(x1),int(y1)),5,(0,0,255),-1) #midpoint of starting edge
			cv2.line(frame,(int(x),int(y)),(int(x1),int(y1)),(255,0,0),3)
			a=y1-y
			b=x1-x
			theta=3.14-math.atan2(a,b)#taking negative to the resultant angle .Angle lies between [-pi,pi]
#			print theta
#			print '........'
			font = cv2.FONT_HERSHEY_SIMPLEX
			cv2.putText(frame,str(theta),(int(x+100),int(y-20)),font,0.8,(0,255,0),2,cv2.LINE_AA)
			q= (x-320)*0.003351064#0.003351064 #(x-320)*0.00303077335822 #0.00353077335822ipx along x axis=0.003515625 m
			r= (y-240)*0.003351064#0.003389831 #(y-240)*0.00302814921311 #0.00352814921311ipx along y axis=0.0034375 m
			emptyBuff=bytearray()
			orientation=[0,0,theta]
			if z==0:
				position=[-q,r,+4.1000e-02]
				returnCode1,outInts1,path_pos,outStrings1,outBuffer1=vrep.simxCallScriptFunction(clientID,'script',vrep.sim_scripttype_childscript,'shift',[],position,[],emptyBuff,vrep.simx_opmode_blocking)
				returnCode1,outInts1,path_pos,outStrings1,outBuffer1=vrep.simxCallScriptFunction	(clientID,'script',vrep.sim_scripttype_childscript,'rotate',[],orientation,[],emptyBuff,vrep.simx_opmode_blocking)
#				print '0303030303'
			else:
				if flag==0:
					if z in freshfruits_id:
						position=[-q,r,+4.1000e-02]
						if position[0]>0:
							if position[1]>0:
								returnCode=vrep.simxSetObjectPosition(clientID,freshfruits[0],-1,position,vrep.simx_opmode_oneshot_wait)	
							if position[1]<0:
								returnCode=vrep.simxSetObjectPosition(clientID,freshfruits[1],-1,position,vrep.simx_opmode_oneshot_wait)
						if position[0]<0:
							returnCode=vrep.simxSetObjectPosition(clientID,freshfruits[2],-1,position,vrep.simx_opmode_oneshot_wait)
#						print returnCode
#						print r
#						print -q
#						print '03030330-------------'
					if z in damagedfruits_id:
						position=[-q,r,+4.1000e-02]	
						returnCode=vrep.simxSetObjectPosition(clientID,damagedfruits[l1],-1,position,vrep.simx_opmode_oneshot_wait)
#						print returnCode
#						print r
#						print -q
#						print '03030330-------------'
						l1=l1+1
				
								
		flag=flag+1
		cv2.imshow('frame',frame)
		if i==t-1:
			cap.release()
			cv2.destroyAllWindows()
			return
##########################################################################################################################################
"""
* Function Name: choose_a_goal()
* Input        : c_handle (takes cylinder handle as a parameter 
*Output        : Gives the cylinder position in vrep scene
* Logic        : This function takes cylinder handle and gives the vrep position 
"""

def choose_a_goal(c_handle):
	returnCode,cyl_position=vrep.simxGetObjectPosition(clientID,c_handle,-1,vrep.simx_opmode_blocking)
	return cyl_position
################################################################################################################################################
"""
* Function Name: set_reset_goal
* Input        : cyl_position (cylinder position)
*Output        : returns the desired goal position
* Logic        : This function sets the desired goal position for the path
"""

def set_reset_goal(cyl_position):
	returnCode=vrep.simxSetObjectPosition(clientID,goal_dummy_handle,-1,cyl_position,vrep.simx_opmode_blocking)
	time.sleep(0)
	return cyl_position
###############################################################################################################################################	
def path():
	emptyBuff=bytearray()	
	returnCode1,outInts1,path_pos,outStrings1,outBuffer1=vrep.simxCallScriptFunction(clientID,'script',vrep.sim_scripttype_childscript,'path',[],[],[],emptyBuff,vrep.simx_opmode_blocking)
	time.sleep(1)
##############################################################################################################################################
"""
* Function Name: motion()
* Input        : pos_on_path,distance,g_p
*Output        : makes the differential drive robot move
* Logic        : This function sets a point in the path accordingly and moves
"""

def motion(pos_on_path,distance,g_p,t):#count correct_time
	
	emptyBuff=bytearray()
	a=[]
	b=[]
	c=[]
	d=[]
	flag=0
	old_x=0
	old_y=0
	while (pos_on_path<0.92):#0------------------------------1
	
		returnCode,start_pos=vrep.simxGetObjectPosition(clientID,start_dummy_handle,-1,vrep.simx_opmode_blocking)

		returnCode1,outInts1,path_pos,outStrings1,outBuffer1=vrep.simxCallScriptFunction	(clientID,'script',vrep.sim_scripttype_childscript,'motion',[],[pos_on_path],[],emptyBuff,vrep.simx_opmode_blocking)
#		print path_pos
		distance=math.sqrt((path_pos[0]*path_pos[0])+(path_pos[1]*path_pos[1]))
		phi=math.atan2(path_pos[1],path_pos[0])
		returnCode,start_pos=vrep.simxGetObjectPosition(clientID,start_dummy_handle,-1,vrep.simx_opmode_blocking)
		returnCode,robot_pos=vrep.simxGetObjectPosition(clientID,robot_handle,-1,vrep.simx_opmode_blocking)
#		print 'distance='
#		print distance
#		print 'phi'
#		print phi
		v_des=0.05
	   	w_des=pid(phi,oe,ei)
		wheel_diameter=0.0701	
		wheel_radius=0.0325	#wheel_diameter/2	
		wheel_separation=0.154	#0.208
		v_r=v_des+(wheel_separation/2)*w_des
		v_l=v_des-(wheel_separation/2)*w_des
	  	w_r=(v_r/wheel_radius)
		w_l=(v_l/wheel_radius)
		a.append(w_r)
		d.append(w_l)		
		
		returnCode=vrep.simxSetJointTargetVelocity(clientID,leftjoint_handle,w_l,vrep.simx_opmode_streaming)
		returnCode=vrep.simxSetJointTargetVelocity(clientID,rightjoint_handle,w_r,vrep.simx_opmode_streaming)
		send_to_cb(w_l,w_r,b,c)
		#loop_time=time.clock()
		print time.time()-t
		if(distance<0.05):#distance<0.05
			#count=count+1#loop_time=time.clock()
			pos_on_path=pos_on_path+0.01
#			print time.time()-t#loop_time-correct_time
			if(time.time()-t)>1:#Taking time as a reference to skip some frames captured by overhead camera
				returnCode=vrep.simxSetJointTargetVelocity(clientID,leftjoint_handle,0,vrep.simx_opmode_streaming)
				returnCode=vrep.simxSetJointTargetVelocity(clientID,rightjoint_handle,0,vrep.simx_opmode_streaming)
                        	send_to_cb(0,0,b,c) 
				setobj(1)
				t=time.time()
						
			#correct_time=loop_time
			
				
	returnCode=vrep.simxSetJointTargetVelocity(clientID,leftjoint_handle,0,vrep.simx_opmode_streaming)
	returnCode=vrep.simxSetJointTargetVelocity(clientID,rightjoint_handle,0,vrep.simx_opmode_streaming)
	string='0'+','+'0'+';'
	a=data.write(string)#sends velocity to the real robot
	time.sleep(0)	
	return 
#########################################################################################################################################
"""
* Function Name: go_to_truck()
* Input        : Nil
*Output        : Makes the differential drive robot move towards the truck to drop the fresh fruit
* Logic        : It sets a stop where it drops the fresh fruit on the truck
"""

def go_to_truck():
	returnCode,g_p=vrep.simxGetObjectPosition(clientID,goal_dummy_handle,-1,vrep.simx_opmode_blocking)
	path() #creates a path
	pos_on_path=0 #the position on path moves from 0 to 1 where initial point is 0
	distance=0
	count=time.time()
	motion(pos_on_path,distance,g_p,count) #makes the robot move along the path toward the truck
#############################################################################################################################################

def set_cylinders_pos(x,y,cylinder_handle):
	position=[x,y,0.04100000113248825]
	returnCode=vrep.simxSetObjectPosition(clientID,cylinder_handle,-1,position,vrep.simx_opmode_oneshot_wait)
	return
###########################################################################################################################################
"""
* Function Name: track_truck()
* Input        : robot position (rp)
*Output        : Nil
* Logic        : Once the bot has reached the stop to drop fresh fruit it waits for the truck to come near it. Once it reached a threshold distance it returns
"""

def track_truck(rp):
	cap=cv2.VideoCapture(1)
	w=cap.get(cv2.CAP_PROP_FRAME_WIDTH)
	h=cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
#	print w
#	print h
	d_pos=1 #distance difference between cb and truck
	while True:
		ret,frame = cap.read()
		if ret==False:
			continue
		framebw=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		aruco_dict=aruco.Dictionary_get(aruco.DICT_5X5_250)
		parameters=aruco.DetectorParameters_create()
		corners,ids,_=aruco.detectMarkers(framebw,aruco_dict,parameters=parameters)
#		print ids
		i=ids.tolist().index([0])  #gives total aruco markers
#		print i
		l=0
		l1=0
		z=int(ids[i][0])
#		print z
		cv2.circle(frame,(corners[i][0][0][0],corners[i][0][0][1]),5,(125,125,125),-1)
		cv2.circle(frame,(corners[i][0][1][0],corners[i][0][1][1]),5,(0,255,0),-1)
		cv2.circle(frame,(corners[i][0][2][0],corners[i][0][2][1]),5,(180,105,255),-1)
		cv2.circle(frame,(corners[i][0][3][0],corners[i][0][3][1]),5,(255,255,255),-1)
		x=((corners[i][0][0][0]+corners[i][0][2][0])/2)
		y=((corners[i][0][0][1]+corners[i][0][2][1])/2)
		cv2.circle(frame,(int(x),int(y)),5,(255,0,0),-1) #center
		x1=(corners[i][0][0][0]+corners[i][0][1][0])/2
		y1=(corners[i][0][0][1]+corners[i][0][1][1])/2
		cv2.circle(frame,(int(x1),int(y1)),5,(0,0,255),-1) #midpoint of starting edge
		cv2.line(frame,(int(x),int(y)),(int(x1),int(y1)),(255,0,0),3)
		a=y1-y
		b=x1-x
		theta=3.14-math.atan2(a,b)#taking negative to the resultant angle .Angle lies between [-pi,pi]
#		print theta
#		print '........'
		font = cv2.FONT_HERSHEY_SIMPLEX
		q= (x-320)*0.003351064#0.003351064 #(x-320)*0.00303077335822 #0.00353077335822ipx along x axis=0.003515625 m
		r= (y-240)*0.003351064#0.003389831 #(y-240)*0.00302814921311 #0.00352814921311ipx along y axis=0.0034375 m
		emptyBuff=bytearray()
		position=[-q,r,+4.1000e-02]
		orientation=[0,0,theta]
#		print position
#		print orientation
		x1=position[0]-rp[0]
		y1=position[1]-rp[1]
		d_pos=math.sqrt((x1)**2+(y1)**2)
		if d_pos<0.015:
	               break
   
	cap.release()
	cv2.destroyAllWindows()
	return
	
	
	

#########################################################################################################################################
vrep.simxFinish(-1)

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
if clientID!=-1:
	print "connected to remote api server"
	
else:
	print 'connection not successful'
	sys.exit("could not connect")

returnCode,robot_handle=vrep.simxGetObjectHandle(clientID,'Collector_Bot',vrep.simx_opmode_oneshot_wait)
returnCode,truck_handle=vrep.simxGetObjectHandle(clientID,'TRUCK',vrep.simx_opmode_oneshot_wait)
returnCode,leftjoint_handle=vrep.simxGetObjectHandle(clientID,'left_joint',vrep.simx_opmode_oneshot_wait)
returnCode,rightjoint_handle=vrep.simxGetObjectHandle(clientID,'right_joint',vrep.simx_opmode_oneshot_wait)
returnCode,start_dummy_handle = vrep.simxGetObjectHandle(clientID,'start',vrep.simx_opmode_oneshot_wait)
returnCode,goal_dummy_handle = vrep.simxGetObjectHandle(clientID,'end',vrep.simx_opmode_oneshot_wait)
returnCode,cylinder_handle1=vrep.simxGetObjectHandle(clientID,'Cylinder',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle2=vrep.simxGetObjectHandle(clientID,'Cylinder0',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle3=vrep.simxGetObjectHandle(clientID,'Cylinder1',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle4=vrep.simxGetObjectHandle(clientID,'Cylinder2',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle5=vrep.simxGetObjectHandle(clientID,'Cylinder3',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle6=vrep.simxGetObjectHandle(clientID,'Cylinder4',vrep.simx_opmode_oneshot_wait )

cylinder_handles=[cylinder_handle1,cylinder_handle2,cylinder_handle3,cylinder_handle4,cylinder_handle5,cylinder_handle6]
freshfruits=[cylinder_handle1,cylinder_handle2,cylinder_handle3]
freshfruits_id=[2,3,4,5]
damagedfruits=[cylinder_handle4,cylinder_handle5,cylinder_handle6]
damagedfruits_id=[6,7,8,9]
location_of_stop_8 = [0.48590427999999997, -0.54622343200000001, 0.041]
orientation_8        = [0, 0, 4.710796326794897]

location_of_stop_3 = [0.41888300000000001, 0.61994684, 0.041]
orientation_2        = [0, 0, 6.238141758198262]

location_of_stop_6 = [-0.78582450800000003, -0.46244683199999997, 0.041]
orientation_6        = [0, 0, 3.5823742229767452]


vehicles=[0,1]
n=len(freshfruits)
flag=0
setobj(flag)

g=0
j=0


for k in range (0,n):		
	returnCode,robot_pos=vrep.simxGetObjectPosition(clientID,robot_handle,-1,vrep.simx_opmode_blocking)
	returnCode=vrep.simxSetObjectPosition(clientID,start_dummy_handle,-1,robot_pos,vrep.simx_opmode_oneshot_wait)
	returnCode,robo_orien=vrep.simxGetObjectOrientation(clientID,robot_handle,-1,vrep.simx_opmode_blocking)
	returnCode=vrep.simxSetObjectOrientation(clientID,start_dummy_handle,-1,robo_orien,vrep.simx_opmode_oneshot_wait)
	cyl_position=choose_a_goal(freshfruits[k])#finds the position of freshfruits
	g_p=set_reset_goal(cyl_position)#sets the goal near fruits, see that the id doesnt correspond to the truck/cb aruco id(use if condition)
	returnCode,fruit=vrep.simxGetObjectOrientation(clientID,freshfruits[k],-1,vrep.simx_opmode_blocking)
	returnCode=vrep.simxSetObjectOrientation(clientID,goal_dummy_handle,-1,fruit,vrep.simx_opmode_oneshot_wait)
	if cyl_position[0]<0:
		orientation11=[-6.2454e-02,6.4782e-04,1.7977e+02]
		returnCode=vrep.simxSetObjectOrientation(clientID,goal_dummy_handle,-1,orientation11,vrep.simx_opmode_oneshot_wait)
	path() #creates a path
	pos_on_path=0 #the position on path moves from 0 to 1 where initial point is 0
	distance=0
	count=time.time()
	motion(pos_on_path,distance,g_p,count) #makes the robot move along the path
	returnCode,start_pos=vrep.simxGetObjectPosition(clientID,start_dummy_handle,-1,vrep.simx_opmode_blocking)
#	print 'start_pos='
#	print start_pos
#	print 'g_p='
#	print g_p
	time.sleep(1)
#	print '_______________________'
	x=math.sqrt(((start_pos[0]-g_p[0])**2)+((start_pos[1]-g_p[1])**2))#to know the distance of the incomplete path
	if (x>0.5):
		returnCode,robot_pos=vrep.simxGetObjectPosition(clientID,robot_handle,-1,vrep.simx_opmode_blocking)
		returnCode=vrep.simxSetObjectPosition(clientID,start_dummy_handle,-1,robot_pos,vrep.simx_opmode_oneshot_wait)
		returnCode,robo_orien=vrep.simxGetObjectOrientation(clientID,robot_handle,-1,vrep.simx_opmode_blocking)
		returnCode=vrep.simxSetObjectOrientation(clientID,start_dummy_handle,-1,robo_orien,vrep.simx_opmode_oneshot_wait)
		path()
		pos_on_path=0
		distance=0
		motion(pos_on_path,distance,g_p)
		returnCode,start_pos=vrep.simxGetObjectPosition(clientID,start_dummy_handle,-1,vrep.simx_opmode_blocking)
		x=math.sqrt(((start_pos[0]-g_p[0])**2)+((start_pos[1]-g_p[1])**2))#to know the distance of the incomplete path
	data.write("g")
	data.write("u")
	if cyl_position[0]>0:
		if cyl_position[1]>0:
#			print "stop 2"
			returnCode=vrep.simxSetObjectPosition(clientID,goal_dummy_handle,-1,location_of_stop_2 ,vrep.simx_opmode_oneshot_wait)
			returnCode=vrep.simxSetObjectOrientation(clientID,goal_dummy_handle,-1,orientation_2,vrep.simx_opmode_oneshot_wait)
		if cyl_position[1]<0:
#			print "stop 8"
			returnCode=vrep.simxSetObjectPosition(clientID,goal_dummy_handle,-1,location_of_stop_8 ,vrep.simx_opmode_oneshot_wait)
			returnCode=vrep.simxSetObjectOrientation(clientID,goal_dummy_handle,-1,orientation_8,vrep.simx_opmode_oneshot_wait)
	if cyl_position[0]<0:
#			print "stop 6"
			returnCode=vrep.simxSetObjectPosition(clientID,goal_dummy_handle,-1,location_of_stop_6,vrep.simx_opmode_oneshot_wait)
			returnCode=vrep.simxSetObjectOrientation(clientID,goal_dummy_handle,-1,orientation_6,vrep.simx_opmode_oneshot_wait)
	posit=[1.1500e+00,-1.7250e+00,5.0000e-02]
	posit[0]=posit[0]+2*k
	returnCode=vrep.simxSetObjectPosition(clientID,freshfruits[k],-1,posit,vrep.simx_opmode_oneshot_wait)

	
	print "going:::::::::::::::::::::::::::::::::::::::::::::::::::::"
	go_to_truck()
	returnCode,robot_pos=vrep.simxGetObjectPosition(clientID,robot_handle,-1,vrep.simx_opmode_blocking)
#	print "robot position sent"
#        track_truck(robot_pos)
#	time.sleep(18)
	data.write("l")
	data.write("g")
	data.write("l")
#	time.sleep(10)
	data.write("d")		
	

returnCode=vrep.simxSetJointTargetVelocity(clientID,leftjoint_handle,0,vrep.simx_opmode_streaming)
returnCode=vrep.simxSetJointTargetVelocity(clientID,rightjoint_handle,0,vrep.simx_opmode_streaming)
w_r=0
w_l=0
w_l=str(w_l)
w_r=str(w_r)
string=w_l+','+w_r+';'
a=data.write(string)
#print a
#cap.release()
cv2.destroyAllWindows()




