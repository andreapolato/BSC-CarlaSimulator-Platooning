import carla
import math
import numpy as np

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
                follower.addWaypoint([self.x, self.y, self.steer])
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
            if (delta>0.03):
                boost = 0.2 if self.big_dist else 0.0
                t = delta/10 + boost if delta/10 + boost <= 1.0 else 1.0
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

    def defineSteer(self, tx, ty):
        s=0.0

        if self.speed>0.03:
            x = self.x
            y = self.y
            yaw = self.yaw
            ip = ((x-tx)**2 + (y-ty)**2)**(1/2)

            cat = abs(x-tx)
            alpha = np.degrees(np.arccos(cat/ip))
            
            if abs(tx-x)<0.5 or abs(ty-y)<0.5:
                alpha = self.yaw

            elif tx<x:
                alpha=180-alpha
            elif ty<=y:
                alpha = -alpha
            
            if alpha<-90 and yaw>90:
                s=(alpha-yaw+360)/60
            elif alpha>90 and yaw<-90:
                s=(alpha-yaw-360)/60
            else:
                s=(alpha-yaw)/60
            
            if s > 1.0: s = 1.0
            elif s < -1.0: s = -1.0
            
            print(self.yaw, alpha, s)
        
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
        target_x = wp[0]
        target_y = wp[1]
        s = wp[2]
        if abs(self.x-target_x)<0.05 and abs(self.y-target_y)<0.05:
            self.waypoints.pop(0)
        self.checkDistance()
        t, b = self.defineThrottle()
        control = carla.VehicleControl(throttle=t, steer=s, brake=b)
        self.vehicle.apply_control(control)
    