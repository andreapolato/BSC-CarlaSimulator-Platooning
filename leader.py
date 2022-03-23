import carla
import math
import numpy as np

def sign(number):
  if number>=0: return 1
  else: return -1

class PlatoonMember:
    
    def __init__(self, vehicle: carla.Vehicle):
        self.vehicle = vehicle
        tr = self.vehicle.get_transform()
        con = self.vehicle.get_control()
        vel = self.vehicle.get_velocity()
        self.x = tr.location.x
        self.y = tr.location.y
        self.z = tr.location.z
        self.yaw = tr.rotation.yaw
        self.speed = 3.6*math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        self.throttle = con.throttle
        self.steer = con.steer
        self.brake = con.brake
    
    def update_position(self):
        tr = self.vehicle.get_transform()
        con = self.vehicle.get_control()
        vel = self.vehicle.get_velocity()
        self.x = tr.location.x
        self.y = tr.location.y
        self.z = tr.location.z
        self.yaw = tr.rotation.yaw
        self.speed = 3.6*math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        self.throttle = con.throttle
        self.steer = con.steer
        self.brake = con.brake

    def get_pos(self):
        return self.x, self.y
    
class Leader(PlatoonMember):
    
    followers = []
   
    def __init__(self, vehicle: carla.Vehicle):
        super().__init__(vehicle)
   
    def addFollower(self, f: carla.Vehicle):
        self.followers.append(f)
   
    def move(self):
        self.update_position()
        self.update_follower()

    def update_follower(self):
        for follower in self.followers:
            if self.speed>0.03:
                follower.addWaypoint([self.x, self.y, self.yaw, self.steer])
                follower.setSpeedGoal(self.speed)

class Follower(PlatoonMember):
  
    leader: PlatoonMember
    waypoints = []
  
    def __init__(self, vehicle:carla.Vehicle, lead:PlatoonMember):
        super().__init__(vehicle)
        self.leader = lead
        self.speedGoal = 0.0
        self.last_t = 0.0
        self.big_dist = False
   
    def addWaypoint(self, wp):
        self.waypoints.append(wp)
   
    def setSpeedGoal(self, s):
        self.speedGoal = s
   
    def defineThrottle(self):
        delta = self.speedGoal - self.speed
        t = b = 0.0
        if self.speedGoal>0.04:
            if (delta>0):
                boost = 0.2 if self.big_dist else 0.0
                t = (delta/10 + boost) if (delta/10 + boost) <= 1.0 else 1.0
            else:
                t = self.last_t
                if not self.big_dist:
                    b = -delta/10 if -delta/10 <= 1.0 else 1.0
        else:
            if self.big_dist:
                t = 0.2
            else:
                b = 0.8
        self.last_t=t
        return t, b

    def defineSteer(self):
        rl = self.waypoints
        fx = self.x
        fy = self.y
        yaw = self.yaw
        n=0
        limit=1000
        s=0.0
        delta_x = delta_y = 360
        rel_x = rel_y = 0
        for row in reversed(rl):
            
            lx = row[0]
            ly = row[1]
            lyaw = row[2]
            toll = 0.1
            if abs(lx-fx)<= toll and abs(ly-fy)<= toll:
                s=row[3]
                if lyaw<-90 and yaw>90:
                    corr=(lyaw-yaw+360)/60
                elif lyaw>90 and yaw<-90:
                    corr=(lyaw-yaw-360)/60
                else:
                    corr=(lyaw-yaw)/60
                if corr > 1.0: corr = 1.0
                elif corr < -1.0: corr = -1.0
                s+=corr
                if s>1.0:
                    s=1.0
                elif s<-1.0:
                    s=-1.0
                return s
            
            else:
                if abs(lx-fx) < delta_x:
                    rel_y = ly
                    delta_x = abs(lx-fx)
                    yaw_x = lyaw
                if abs(ly-fy) < delta_y:
                    rel_x = lx
                    delta_y = abs(ly-fy)
                    yaw_y=lyaw
                    
        lyaw = yaw_x if delta_x<delta_y else yaw_y
        #yaw_condition = sign(lyaw)==sign(yaw) or (abs(lyaw)>90 and abs(yaw)>90)
        #same_yaw = abs(lyaw-yaw)<3 if yaw_condition else abs(lyaw+yaw)<3
        
        corr = (lyaw-yaw)/60

        if abs(yaw)<=5: #going east
            s=(rel_y-fy)/10
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0
            if corr>1.0: corr = 1.0
            elif corr<-1.0: corr = -1.0
            s+=corr
        elif abs(yaw)>=175: #going ovest
            if lyaw>90 and yaw<-90:
                corr-=4
                s=(fy-rel_y)/10
                if s>1.0: s=1.0
                elif s<-1.0: s=-1.0
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr
            elif lyaw<-90 and yaw>90:
                corr+=4
                s=(fy-rel_y)/10
                if s>1.0: s=1.0
                elif s<-1.0: s=-1.0
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr
            else:
                s=(fy-rel_y)/10
                if s>1.0: s=1.0
                elif s<-1.0: s=-1.0
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr

        elif yaw<95 and yaw>85: #going south
            s=(fx-rel_x)/10
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0
            if corr>1.0: corr = 1.0
            elif corr<-1.0: corr = -1.0
            s+=corr
            
        elif yaw>-95 and yaw<-85: #going north
            s=(rel_x-fx)/10 
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0
            if corr>1.0: corr = 1.0
            elif corr<-1.0: corr = -1.0
            s+=corr

        else:
            if yaw>-180 and yaw<=-90:
                l_pos = -rel_y if delta_x<delta_y else rel_x
                f_pos = fy if delta_x<delta_y else -fx
                
            elif yaw>-90 and yaw<=0:
                l_pos = rel_y if delta_x<delta_y else rel_x
                f_pos = -fy if delta_x<delta_y else -fx
            
            elif yaw>0 and yaw<=90:
                l_pos = rel_y if delta_x<delta_y else -rel_x
                f_pos = -fy if delta_x<delta_y else fx
            
            elif yaw>90 and yaw<=180:
                l_pos = -rel_y if delta_x<delta_y else -rel_x
                f_pos = fy if delta_x<delta_y else fx
            
            if sign(l_pos)==sign(f_pos): l_pos = -l_pos

            s = (l_pos+f_pos)/40
            if s>1.0: s=1.0
            elif s<-1.0: s=-1.0

            if lyaw>90 and yaw<-90:
                corr=(lyaw-yaw-360)/60
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr
            
            elif lyaw<-90 and yaw>90:
                corr=(lyaw-yaw+360)/60
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr
            
            else:
                corr=(lyaw-yaw)/60
                if corr>1.0: corr = 1.0
                elif corr<-1.0: corr = -1.0
                s+=corr
        
        return s

    def checkDistance(self):
        x = self.x
        y = self.y
        tx,ty = self.leader.get_pos()
        if ((x-tx)**2 + (y-ty)**2)**(1/2) > 12.0:
            self.big_dist = True
        else: self.big_dist = False

    def move(self):
        self.update_position()
        wp = self.waypoints[0]
        self.checkDistance()
        s = 0.0
        t, b = self.defineThrottle()
        if self.speed>0.03:
            s = self.defineSteer()
        control = carla.VehicleControl(throttle=t, steer=s, brake=b)
        self.vehicle.apply_control(control)
    