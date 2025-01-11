from commands2 import Subsystem

from ntcore import *

class VisionCameraDetector(Subsystem):
    def __init__(self, id:int):
        self.id = f"limelight{id}"

        self.table = NetworkTableInstance.getDefault().getTable(self.id)

    def update_data(self) -> dict[str, float]:
        has_target = bool(self.table.getNumber('tv'))
        if has_target:
            #NOTE: this is all referring to pixels in the image, not real space
            hori_dist = self.table.getNumber('tv') # crosshair to target (center?)
            vert_dist = self.table.getNumber('ty') # crosshair to target (center?)
            target_area = self.table.getNumber('ta') # percent of image area included in target area
        return {
            "hori_dist":hori_dist,
            "vert_dist":vert_dist,
            "target_area":target_area
        }