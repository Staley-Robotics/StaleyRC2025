from commands2 import Subsystem

from ntcore import *

class VisionCameraDetector(Subsystem):
    def __init__(self, id:int):
        self.id = f"limelight-one"

        self.table = NetworkTableInstance.getDefault().getTable(self.id)

        # self.sub = self.table.getDoubleArrayTopic('t2d').subscribe()

        self.data = {
            "valid_data":0,
            "hori_dist":0,
            "vert_dist":0,
            "target_area":0,
            "heartbeat":0,
            "latency":0
        }

        self.last_hb = 0
        self.cycles_since_data = 1000

        self.has_recieved_update = False

    def update_data(self) -> None:
        if self.table.getNumber('hb', 0) != self.last_hb: #check that new frame has been recieved (hb increases once each frame)
            self.has_recieved_update = True
            if self.table.getNumber('tv', 0) == 0: # check has valid data
                self.cycles_since_data += 1
            else:
                self.cycles_since_data = 0
            # self.data['valid_data'] = self.table.getNumber('tv', 0)
            self.data['hori_dist'] = self.table.getNumber('tx', 0) # crosshair to center of target (degrees)
            self.data['vert_dist'] = self.table.getNumber('ty', 0) # crosshair to center of target (degrees)
            self.data['target_area'] = self.table.getNumber('ta', 0) # percent of image area included in target area
            self.data['latency'] = (self.table.getNumber('tl', 0) + self.table.getNumber('cl', 0)) / 1000
        else:
            self.data = {
                "valid_data":0,
                "hori_dist":0,
                "vert_dist":0,
                "target_area":0,
                "heartbeat":0,
                "latency":0
            }
    
    def hasRecievedUpdate(self) -> bool:
        temp = self.has_recieved_update
        self.has_recieved_update = False
        return temp
    
    def hasTarget(self) -> bool:
        return self.cycles_since_data < 15 # ~=1.4 seconds, cam gets ~9 fps
