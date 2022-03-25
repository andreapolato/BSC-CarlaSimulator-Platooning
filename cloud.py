
import platooning

class SafeCloud:
    
    def __init__(self):
        self.members =[]
        self.leader = None

    def set_leader(self, l):
        if not self.leader: 
            self.leader=l
            print("Leader set.")

    def add_members(self, m):
        self.members.append(m)
        print(len(self.members))
        self.leader.addFollower(m)

    def check_action(self,member,t,s,b):
        for elem in self.members:
            if member==elem:
                x = elem.vehicle.get_transform().location.x
                y = elem.vehicle.get_transform().location.y
