import typing

from ntcore.util import ntproperty

from subsystems import SwerveDrive
from subsystems import VisionCameraDetector
from commands.DriveByStick import DriveByStick

class DriveByNote(DriveByStick):
    def __init__(self,
                  swerve:SwerveDrive,
                  detectorCam:VisionCameraDetector,
                  frcFwd: typing.Callable[[], float] = lambda: 0.0,
                  frcLeft: typing.Callable[[], float] = lambda: 0.0,
                ) -> None:
        super().__init__(swerve, frcFwd, frcLeft, lambda: 0.0)
        self.cam = detectorCam

        self.hold_field_relative = False

        self.rotationMult = ntproperty('NoteTracking/Rotation Multiplier', 0.0, writeDefault=True)
    
    def initialize(self):
        self.hold_field_relative = self.__subsystem.getFieldRelative()
        self.__subsystem.setFieldRelative(False)
        return super().initialize()
    
    def end(self, interrupted):
        self.__subsystem.setFieldRelative(self.hold_field_relative)
        return super().end(interrupted)

    def getR(self) -> float:
        return self.cam.update_data()['hori_dist'] * self.rotationMult