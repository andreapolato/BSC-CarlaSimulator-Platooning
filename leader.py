import carla
import math
import numpy as np

from start import leader_going_straight

class PlatoonMember:
    
    def __init__(self, vehicle: carla.Vehicle):
        self.vehicle = vehicle
        tr = self.vehicle.get_transform()
        vel = self.vehicle.get_velocity()
        self.x = tr.location.x
        self.y = tr.location.y
        self.z = tr.location.z
        self.yaw = tr.rotation.yaw
        self.speed = 3.6*math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        self.throttle = tr.throttle
        self.steer = tr.steer
        self.brake = tr.brake
    
    def update_position(self):
        tr = self.vehicle.get_transform()
        vel = self.vehicle.get_velocity()
        self.x = tr.location.x
        self.y = tr.location.y
        self.z = tr.location.z
        self.yaw = tr.rotation.yaw
        self.speed = 3.6*math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        self.throttle = tr.throttle
        self.steer = tr.steer
        self.brake = tr.brake

    
class Leader(PlatoonMember):
    
    followers = []

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def addFollower(self, f: carla.Vehicle):
        self.followers.append(f)

    def sendLocation(self):
        for follower in self.followers:
            follower.addWaypoint([self.x, self.y])

class Follower(PlatoonMember):

    leader: Leader
    waypoints = []

    def __init__(self, vehicle, lead):
        super().__init__(vehicle)
        self.leader = lead
        self.speedGoal = 0.0

    def addWaypoint(self, wp):
        self.waypoints.append(wp)

    def setSpeedGoal(self, s):
        self.speedGoal = s

    def updateControl(self):
        wp = self.waypoints[0]
        self.waypoints.pop(0)
        target_x = wp[0]
        target_y = wp[1]
        s = self.defineSteer(target_x,target_y)
        control = carla.VehicleControl(throttle=t, steer=s, brake=b)
        self.vehicle.apply_control(control)

    def defineSteer(self, tx, ty):
        x = self.x
        y = self.y
        yaw = self.yaw
        trajectory = ((x-tx)**2 + (y-ty)**2)**(1/2)
        m = (tx-x)/(ty-y)
        np.arctan(m)
        return s
