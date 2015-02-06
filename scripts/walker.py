#!/usr/bin/env python

from threading import Thread
import rospy
import math
from darwin_gazebo.darwin import Darwin
from geometry_msgs.msg import Twist


class WJFunc:
    """Walk Joint Function CPG style"""
    def __init__(self):
        self.offset=0
        self.scale=1
        self.in_offset=0
        self.in_scale=1
        
    def get(self,x):
        """ x between 0 and 1"""
        f=math.sin(self.in_offset+self.in_scale*x)
        return self.offset+self.scale*f        
        
    def clone(self):
        z=WJFunc()
        z.offset=self.offset
        z.scale=self.scale
        z.in_offset=self.in_offset
        z.in_scale=self.in_scale
        return z
    
    def mirror(self):
        z=self.clone()
        z.offset*=-1
        z.scale*=-1
        return z
        
    def __str__(self):
        return "y=%f+%f*sin(%f+%f*x)"%(self.offset,self.scale,self.in_offset,self.in_scale)
        
class WFunc:
    """Multi-joint walk function for Darwin"""
    def __init__(self,**kwargs):
        self.parameters={}

        self.parameters["swing_scale"]=0
        self.parameters["step_scale"]=0.3
        self.parameters["step_offset"]=0.55
        self.parameters["ankle_offset"]=0
        self.parameters["vx_scale"]=0.5
        self.parameters["vy_scale"]=0.5
        self.parameters["vt_scale"]=0.4
        
        for k,v in kwargs.items():
            self.parameters[k]=v
        
        self.generate()
        
    def generate(self):
        # f1=THIGH1=ANKLE1=L=R in phase
        self.pfn={} # phase joint functions    
        self.afn={} # anti phase joint functions

        #~ print f        
        f1=WJFunc()
        f1.in_scale=math.pi
        f1.scale=-self.parameters["swing_scale"]
        self.pfn["j_ankle2_l"]=f1
        self.pfn["j_thigh1_l"]=f1
        
        # f2=mirror f1 in antiphase
        f2=f1.mirror()
        #~ f2=WJFunc()
        self.afn["j_ankle2_l"]=f2
        self.afn["j_thigh1_l"]=f2

        f3=WJFunc()
        f3.in_scale=math.pi
        f3.scale=self.parameters["step_scale"]
        f3.offset=self.parameters["step_offset"]
        self.pfn["j_thigh2_l"]=f3
        f33=f3.mirror()
        f33.offset+=self.parameters["ankle_offset"]
        self.pfn["j_ankle1_l"]=f33
        
        f4=f3.mirror()
        f4.offset*=2
        f4.scale*=2
        self.pfn["j_tibia_l"]=f4
        
        s2=0
        f5=f3.clone()
        f5.in_scale*=2
        f5.scale=s2
        self.afn["j_thigh2_l"]=f5
        
        
        f6=f3.mirror()
        f6.in_scale*=2
        f6.scale=f5.scale
        f6.offset+=self.parameters["ankle_offset"]
        self.afn["j_ankle1_l"]=f6
        
        f7=f4.clone()
        f7.scale=0
        self.afn["j_tibia_l"]=f7

        
        self.forward=[f5,f6]
        
        self.generate_right()
        self.joints=self.pfn.keys()
        
        self.show()
       
    def generate_right(self):
        # Mirror from left to right and antiphase right
        l=[ v[:-2] for v in self.pfn.keys()]
        for j in l:
            self.pfn[j+"_r"]=self.afn[j+"_l"].mirror()
            self.afn[j+"_r"]=self.pfn[j+"_l"].mirror()
        
    def get(self,phase,x,velocity):
        """ x between 0 and 1"""
        angles={}
        for j in self.pfn.keys():
            if phase:
                v=self.pfn[j].get(x)
                angles[j]=v
            else:
                angles[j]=self.afn[j].get(x)
        self.apply_velocity(angles,velocity,phase,x)
        return angles
            
            
        
        
    def show(self):
        for j in self.pfn.keys():
            print j,"p",self.pfn[j],"a",self.afn[j]
        
    
    def apply_velocity(self,angles,velocity,phase,x):
        
        # VX
        v=velocity[0]*self.parameters["vx_scale"]
        d=(x*2-1)*v
        if phase:
            angles["j_thigh2_l"]+=d
            angles["j_ankle1_l"]+=d
            angles["j_thigh2_r"]+=d
            angles["j_ankle1_r"]+=d
        else:
            angles["j_thigh2_l"]-=d
            angles["j_ankle1_l"]-=d
            angles["j_thigh2_r"]-=d
            angles["j_ankle1_r"]-=d

        # VY
        v=velocity[1]*self.parameters["vy_scale"]
        d=(x)*v
        d2=(1-x)*v
        if v>=0:
            if phase:
                angles["j_thigh1_l"]-=d
                angles["j_ankle2_l"]-=d
                angles["j_thigh1_r"]+=d
                angles["j_ankle2_r"]+=d
            else:
                angles["j_thigh1_l"]-=d2
                angles["j_ankle2_l"]-=d2
                angles["j_thigh1_r"]+=d2
                angles["j_ankle2_r"]+=d2
        else:
            if phase:
                angles["j_thigh1_l"]+=d2
                angles["j_ankle2_l"]+=d2
                angles["j_thigh1_r"]-=d2
                angles["j_ankle2_r"]-=d2
            else:
                angles["j_thigh1_l"]+=d
                angles["j_ankle2_l"]+=d
                angles["j_thigh1_r"]-=d
                angles["j_ankle2_r"]-=d
                
        # VT
        v=velocity[2]*self.parameters["vt_scale"]
        d=(x)*v
        d2=(1-x)*v
        if v>=0:
            if phase:
                angles["j_pelvis_l"]=-d
                angles["j_pelvis_r"]=d
            else:
                angles["j_pelvis_l"]=-d2
                angles["j_pelvis_r"]=d2
        else:
            if phase:
                angles["j_pelvis_l"]=d2
                angles["j_pelvis_r"]=-d2
            else:
                angles["j_pelvis_l"]=d
                angles["j_pelvis_r"]=-d



class Walker:
    """
    Class for making Darwin walk
    """
    def __init__(self,darwin):
        self.darwin=darwin
        self.running=False

        self.velocity=[0,0,0]
        self.walking=False
        self.func=WFunc()

        #~ self.ready_pos=get_walk_angles(10)
        self.ready_pos=self.func.get(True,0,[0,0,0])
        
        self._th_walk=None

        self._sub_cmd_vel=rospy.Subscriber(darwin.ns+"cmd_vel",Twist,self._cb_cmd_vel,queue_size=1)


    def _cb_cmd_vel(self,msg):
        print "cmdvel",msg
        vx=msg.linear.x
        vy=msg.linear.y
        vt=msg.angular.z
        self.start()
        self.set_velocity(vx,vy,vt)
        
    def init_walk(self):
        rospy.loginfo("Going to walk position")
        if self.get_dist_to_ready()>0.02:                    
            self.darwin.set_angles_slow(self.ready_pos)        

    def start(self):
        if not self.running:
            self.running=True                    
            self.init_walk()
            self._th_walk=Thread(target=self._do_walk)
            self._th_walk.start()
            self.walking=True
    
    def stop(self):
        if self.running:
            self.walking=False
            rospy.loginfo("Waiting for stopped")
            while not rospy.is_shutdown() and self._th_walk is not None:
                rospy.sleep(0.1)                
            rospy.loginfo("Stopped")
            self.running=False
            
    def set_velocity(self,x,y,t):
        self.velocity=[x,y,t]
        

    def _do_walk(self):
        r=rospy.Rate(100)
        rospy.loginfo("Started walking thread")
        func=self.func
        
        # Global walk loop
        n=50
        p=True
        i=0
        self.current_velocity=[0,0,0]
        while not rospy.is_shutdown() and (self.walking or i<n or self.is_walking()):
            if not self.walking:
                self.velocity=[0,0,0]
            if not self.is_walking() and i==0: # Do not move if nothing to do and already at 0
                self.update_velocity(self.velocity,n)
                r.sleep()
                continue
            x=float(i)/n            
            angles=func.get(p,x,self.current_velocity)
            self.update_velocity(self.velocity,n)
            self.darwin.set_angles(angles)
            i+=1
            if i>n:
                i=0
                p=not p
            r.sleep()
        rospy.loginfo("Finished walking thread")
        
        self._th_walk=None

    def is_walking(self):
        e=0.02
        for v in self.current_velocity:
            if abs(v)>e: return True
        return False
        
    def rescale(self,angles,coef):
        z={}
        for j,v in angles.items():            
            offset=self.ready_pos[j]
            v-=offset
            v*=coef
            v+=offset
            z[j]=v
        return z
            

    def update_velocity(self,target,n):
        a=3/float(n)
        b=1-a
        self.current_velocity=[a*t+b*v for (t,v) in zip(target,self.current_velocity)]
        
    def get_dist_to_ready(self):
        angles=self.darwin.get_angles()
        return get_distance(self.ready_pos,angles)
            
                

def interpolate(anglesa,anglesb,coefa):
    z={}
    joints=anglesa.keys()
    for j in joints:
        z[j]=anglesa[j]*coefa+anglesb[j]*(1-coefa)
    return z

def get_distance(anglesa,anglesb):
    d=0
    joints=anglesa.keys()
    if len(joints)==0: return 0
    for j in joints:
        d+=abs(anglesb[j]-anglesa[j])
    d/=len(joints)
    return d


if __name__=="__main__":
    rospy.init_node("walker")
    rospy.sleep(1)
    
    rospy.loginfo("Instantiating Darwin Client")
    darwin=Darwin()
    rospy.loginfo("Instantiating Darwin Walker")
    walker=Walker(darwin)
 
    rospy.loginfo("Darwin Walker Ready")
    while not rospy.is_shutdown():
        rospy.sleep(1)