from turtle import pos, position, update


import carla

class Leader:
    vehicle: carla.Vehicle

    def __init__(self, vehicle):
        self.vehicle = vehicle
        tr = self.vehicle.get_transform()
        self.x = tr.location.x
        self.y = tr.location.y
        self.z = tr.location.z
        self.yaw = tr.rotation.yaw
        self.speed = 

    def update_position(self):
        tr = self.vehicle.get_transform()
        self.x = tr.location.x
        self.y = tr.location.y
        self.z = tr.location.z
        self.yaw = tr.rotation.yaw