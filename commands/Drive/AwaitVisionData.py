import typing

from wpimath.geometry import Pose2d
from wpilib import Timer

from commands2 import Command, Subsystem
from subsystems import Vision, SwerveDrive
from util import FalconLogger

class AwaitVisionData(Command):
    # Variable Declaration
    TRUST_VISION:float = 1.
    NO_TRUST_VISION:float = 999999.
    
    # Initialization
    def __init__( self,
                  visionSys:Vision,
                  driveSys:SwerveDrive
                ) -> None:
        # Command Attributes
        self.visionSys = visionSys
        self.driveSys = driveSys

        self.timer = Timer()

        self.setName( "AwaitVisionData" )

    def initialize(self) -> None:
        self.visionSys.set_stddev(self.TRUST_VISION)

    def execute(self) -> None:
        if self.visionSys.has_received_data:
            self.timer.start()

    def end(self, interrupted:bool) -> None:
        self.visionSys.set_stddev(self.NO_TRUST_VISION)
        self.driveSys.resetOdometry(self.driveSys.getPose())

    def isFinished(self) -> bool:
        return self.timer.get() > 5

    def runsWhenDisabled(self) -> bool:
        return True