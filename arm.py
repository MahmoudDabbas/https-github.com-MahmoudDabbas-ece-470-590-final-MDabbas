#This sofware is provide to control a Hubo-Ach Robot 
#the robot DRC-Hubo raise the left leg and balance on one foot and go up and down on right leg.
#then go on the left foot, left its right leg and hands into a position almost parallel to ground, 
#and and go up and down on right leg. then lower it self and balance on both legs again
#created 10 01 2015
#by Mahmoud Al Dabbas
 
 
import hubo_ach as ha
import ach
import sys
import time
import math
from time import sleep
from ctypes import *
from time import gmtime, strftime
import numpy as np

ROBOT_TIME_CHAN  = 'state.time'
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

freq = 10
period = 1/freq
PI = 3.141529
# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
s.flush()
r.flush()
# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()
# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()
# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)
print 'Starting the movement....'

shoulder_height=1.16
Zgoal=1-shoulder_height
Theta_init=np.array([0.0,0.0,0.0,0.0,0.0,0.0])#LSP,LSR,LSY,LEB,LWY,LWP
Goal=np.array([1.5,0.0,Zgoal]) #the goal position of the box
DeltaTheta = 0.06 #in radian
Error = 0.006 #equals 1% of the total length of arm

Length = np.array([0.24,0.31,0.31,0.13])#represent the lengths of the arm components. namely the shoulder, the arm, the forearm and the hand/fingers
l1 = Length[0]#the shoulder
l2 = Length[1]#the arm
l3 = Length[2]#the forearm
l4 = Length[3]#the hand/fingers
step = .01 #in meter



#new identity matrices to initiate the transformation matrices.
T01=np.identity(4)#for the shoulder pitch and translation
T12=np.identity(4)#for the shoulder roll
T23=np.identity(4)#for the shoulder yaw
T34=np.identity(4)#for the elbow pitch and translation 
T45=np.identity(4)#for the wrist yaw and translation
T56=np.identity(4)#for the wrist roll 
T67=np.identity(4)#for the hand translation


def 	GetFK(theta):
#for the shoulder pitch and translation
	T01[0,0]=np.cos(theta[0])
	T01[2,2]=T01[0,0]
	T01[0,2]=np.sin(theta[0])
	T01[2,0]=-1*T01[0,2]
	T01[1,3]=l1
#for the shoulder roll
	T12[1,1]=np.cos(theta[1])
	T12[2,2]=T12[1,1]
	T12[1,2]=-1*np.sin(theta[1])
	T12[2,1]=-1*T12[0,2]
	T12[1,3]=0
#for the shoulder yaw
	T23[0,0]=np.cos(theta[2])
	T23[1,1]=T23[0,0]
	T23[0,1]=-1*np.sin(theta[2])
	T23[1,0]=-1*T23[0,1]
	T23[1,3]=0
#for the elbow pitch and translation
	T34[0,0]=np.cos(theta[3])
	T34[2,2]=T34[0,0]
	T34[0,2]=np.sin(theta[3])
	T34[2,0]=-1*T34[0,2]
	T34[2,3]=-1*l2
#for the wrist yaw and translation
	T45[0,0]=np.cos(theta[4])
	T45[1,1]=T45[0,0]
	T45[0,1]=-1*np.sin(theta[4])
	T45[1,0]=-1*T45[0,1]
	T45[2,3]=-1*l3
#for the wrist roll
	T56[1,1]=np.cos(theta[5])
	T56[2,2]=T56[1,1]
	T56[1,2]=-1*np.sin(theta[5])
	T56[2,1]=-1*T56[1,2]
	T56[1,3]=0
#for the hand translation
	T67[2,3]=-1*l4
#cross multiplying the matrices with each other to get the final transformation matrix
	T=np.dot(T01,T12)
	T=np.dot(T,T23)
	T=np.dot(T,T34)
	T=np.dot(T,T45)
	T=np.dot(T,T56)
	T=np.dot(T,T67)
	
#calculating the x, y and z Angles of endeffector location 
	ThetaX= round(math.atan2(T[2,1],T[2,2]),1)*180/PI
	ThetaY= round(math.atan2(-1*T[2,0],math.pow(T[2,1],2)+math.pow(T[2,2],2)),1)*180/PI
	ThetaZ= round(math.atan2(T[1,0],T[0,0])*180/PI,1)
	Theta=np.array([ThetaX,ThetaY,ThetaZ])

#multipling the final coordination matrix with the coordinates of the hand effector in the last space to get the location vector T
	T=np.dot(T,np.array([[0],[0],[0],[1]]))
	x=round(T[0],3)#extracting x value of the end effector from the location vector
	y=round(T[1],3)#extracting y value of the end effector from the location vector
	z=round(T[2],3)#extracting z value of the end effector from the location vector
	e=np.array([x,y,z])
	return (e,Theta)	





#a function to calculate the distance between the end-effector and the origin
def 	GetDist(Endeffector,Goal):
	return np.sqrt((np.power((Endeffector[0]-Goal[0]),2)+np.power((Endeffector[1]-Goal[1]),2)+np.power((Endeffector[2]-Goal[2]),2)))







def 	tilt_right():
	print 'Tilting to the right....'
	b=0.16 #define the angle of body tiltig to shift the center of mass.
	a=0.01*b #some step
	c = 0
	while a<=b:
		ref.ref[ha.LEB] = 0
		ref.ref[ha.LSR] = 1.5*a #left arm swing
		ref.ref[ha.RSR] = -1.5*a #right arm swing
		ref.ref[ha.LAR] = -a
		ref.ref[ha.RAR] = -a
		ref.ref[ha.LHR] = a
		ref.ref[ha.RHR] = 1*a
		ref.ref[ha.RHP] = 0
		ref.ref[ha.LHP] = 0
		ref.ref[ha.RKN] = 0
		ref.ref[ha.LKN] = 0
		ref.ref[ha.LAP] = -0
		ref.ref[ha.RAP] = -0
		ref.ref[ha.RSP] = 0 
		ref.ref[ha.LSP] = 0
		time.sleep(0.2)
		a=a+0.02*b
		r.put(ref)
	return None



def rais_left_leg():
	b=PI/3
	a=0.01*b
	c = 0
	print 'Now lefting the left leg'
	while a<=b: # for movemnet smoothing.
	    ref.ref[ha.LHP] = -1*a 
	    ref.ref[ha.RSR] = -.5
	    ref.ref[ha.LAR] = 0
	    ref.ref[ha.LHR] = .2
	    ref.ref[ha.LKN] = 2*a 
	    ref.ref[ha.LAP] = -1*a 
	    x=time.time()
	    r.put(ref)
	    #print "%.20f" % time.time(),'   Now lefting the left leg'
	    c=c+1
	    a=a+0.01*b
	    time.sleep(.02)
	return None




def lower_right_leg(radians):
	print 'tilting down'
	b=radians
	a=0.01*b
	c = 0
	while a<=b: #going down
		ref.ref[ha.RHP] = -1.1*a 
		ref.ref[ha.RKN] = 2*a 
		ref.ref[ha.RAP] = -1*a 
		x=time.time()
		r.put(ref)
		#print "%.20f" % time.time(), c,'   Going down'
		c=c+1
		a=a+0.01*b
		time.sleep(.05)
	return None



def tilt_up(radians):
	print 'tilting up'
	b=radians
	a=b
	c = 0
	while a>=0: #going up
		ref.ref[ha.RSP] = 0
		ref.ref[ha.LSP] = 0 
		ref.ref[ha.RHP] = -1.1*a 
		ref.ref[ha.RKN] = 2*a 
		ref.ref[ha.RAP] = -1*a 
		x=time.time()
		r.put(ref)
		#print "%.20f" % time.time(), c,'    Going up'
		c=c-1
		a=a-0.01*b
		time.sleep(.05)
	return None


def relax():
	print 'Now relaxing the Robot'
	counter = 10
	LAR=ref.ref[ha.LAR] 
	RAR=ref.ref[ha.RAR] 
	LHR=ref.ref[ha.LHR] 
	RHR=ref.ref[ha.RHR] 
	LAR=ref.ref[ha.LAR] 
	RHP=ref.ref[ha.RHP] 
	LHP=ref.ref[ha.LHP] 
	RKN=ref.ref[ha.RKN] 
	LKN=ref.ref[ha.LKN] 
	LAP=ref.ref[ha.LAP]  
	RAP=ref.ref[ha.RAP]  
	RSP=ref.ref[ha.RSP]  
	LSP=ref.ref[ha.LSP] 

	while counter>10:
		ref.ref[ha.LAR] = ref.ref[ha.LAR]-0.1*LAR
		ref.ref[ha.RAR] = ref.ref[ha.RAR]-0.1*RAR
		ref.ref[ha.LHR] = ref.ref[ha.LHR]-0.1*LHR
		ref.ref[ha.RHR] = ref.ref[ha.RHR]-0.1*RHR
		ref.ref[ha.RHP] = ref.ref[ha.RHP]-0.1*RHP
		ref.ref[ha.LHP] = ref.ref[ha.LHP]-0.1*LHP
		ref.ref[ha.RKN] = 0
		ref.ref[ha.LKN] = 0
		ref.ref[ha.LAP] = 0 
		ref.ref[ha.RAP] = 0 
		ref.ref[ha.RSP] = 0.05 
		ref.ref[ha.LSP] = 0.05
		r.put(ref)
		counter = counter-1
	return None


def tilt_left():
	print 'going to the left leg...................'
	b=0.165 #define the angle of body tiltig to shift the center of mass.
	a=0.01*b #some step
	c = 0
	print 'Tilting to the left....'
	while a<=b:
		ref.ref[ha.LEB] = 0
		ref.ref[ha.LSR] = 1.5*a #left arm swing
		ref.ref[ha.RSR] = -1.5*a #right arm swing
		ref.ref[ha.LAR] = a
		ref.ref[ha.RAR] = a
		ref.ref[ha.LHR] = -a
		ref.ref[ha.RHR] = -a
		ref.ref[ha.RHP] = 0
		ref.ref[ha.LHP] = 0
		ref.ref[ha.RKN] = 0
		ref.ref[ha.LKN] = 0
		ref.ref[ha.LAP] = 0
		ref.ref[ha.RAP] = 0
		ref.ref[ha.RSP] = 0 
		ref.ref[ha.LSP] = 0
		time.sleep(0.2)
		a=a+0.02*b
		r.put(ref)
	return None

def rais_rigth_leg():
	b=PI/3
	a=0.01*b
	c = 0
	print 'Now lefting the right leg'

	while a<=b: # for movemnet smoothing.
		ref.ref[ha.RHP] = -1*a 
		ref.ref[ha.LSR] = .5
		ref.ref[ha.RAR] = 0
		ref.ref[ha.RHR] = -.2
		ref.ref[ha.RKN] = 2*a 
		ref.ref[ha.RAP] = -1*a 
		x=time.time()
		r.put(ref)
		#print "%.20f" % time.time(), c, '    Now lefting the right leg'
		c=c+1
		a=a+0.01*b
		time.sleep(.02)
		return None


def 	tilt_forward_left():
	return None

def 	tilt_right_leg_forward():
	return None

def 	tilt_forward_right():
	return None

def 	tilt_left_leg_forward():
	return None

def 	extend_left_leg():
	return None

def 	rais_right_leg():
	return None



def walk(number_steps):
	while number_steps>0:
		tilt_forward_left()
		time.sleep(1)
		tilt_right_leg_forward()
		time.sleep(1)
		tilt_forward_right()
		time.sleep(1)
		tilt_left_leg_forward()
		time.sleep(1)
		number_steps=number_steps-1

def 	GetJacobian (DeltaTheta,Theta):
	EndEffector = GetFK(Theta)
	for i in range(0,len(EndEffector)):
		for j in range(0,len(Theta)):
			ThetaNew = Theta 
			ThetaNew[j] = Theta[j] + DeltaTheta
			DeltaEndEffector = GetFK(ThetaNew)[i] - EndEffector[i]
			Jacobian[i,j]=DeltaEndEffector/DeltaTheta
	return Jacobian


def update_Hand_Location(current_theta):
	T0_actual = time.time()
	t.get(tim, wait=False, last=True)
	T0_sim=tim.sim[0]
	ref.ref[ha.LSP]=current_theta[0]
	ref.ref[ha.LSR]=current_theta[1]
	ref.ref[ha.LSY]=current_theta[2]
	ref.ref[ha.LEB]=current_theta[3]
	ref.ref[ha.LWY]=current_theta[4]
	ref.ref[ha.LWP]=current_theta[5]
	r.put(ref)
	Delta_t_sim=tim.sim[0]-T0_sim
	Delta_t_actual=time.time()-T0_actual
	ratio=Delta_t_sim / Delta_t_actual
	time.sleep(ratio*max(0,(period-(time.time()-T0_actual))))
	

def extend_right_hand(Target):
	while np.all(x>Error):
		Jacobian=GetJacobian(DeltaTheta,Theta)
		PseduInvertedJacobian=np.linalg.pinv(Jacobian)
		DeltaEndEffector=GetNextPointDelta(EndEffector,Target,step)
		DeltaTheta2=np.dot(DeltaEndEffector,PseduInvertedJacobian)
		Theta = Theta + DeltaTheta2
		update_Hand_Location(Theta)
		EndEffector = GetFK(Theta)
		x=GetDist(EndEffector,Target)
	return None




Jacobian = np.zeros(shape=(3,6))
Theta = Theta_init
print "Theta_init is : ",Theta_init
EndEffector = GetFK(Theta_init)#+np.array([0.0,0.0,1.16])
steps=3
x=GetDist(EndEffector,Goal)
while True:
	tilt_right()
	time.sleep(1)
	rais_left_leg()
	time.sleep(1)
	lower_right_leg(PI/3)
	time.sleep(1)
	extend_left_leg()
	time.sleep(1)
	walk(steps)	
	time.sleep(1)
	rais_right_leg()
	time.sleep(1)
	extend_right_hand(Goal)


# Close the connection to the channels
r.close()
s.close()
