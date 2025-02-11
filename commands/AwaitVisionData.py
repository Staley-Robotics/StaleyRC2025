import typing

from wpimath.geometry import Pose2d

from commands2 import Command, Subsystem
from subsystems import Vision
from util import FalconLogger

class AwaitVisionData(Command):
    # Variable Declaration
    # vision:Vision = None
    # m_getValue:typing.Callable[[],float] = lambda: 0.0
    
    # Initialization
    def __init__( self,
                  visionDataRecieved: typing.Callable[[], bool],
                  resetSwerveGyro: typing.Callable[[], None],
                  getVisionPose: typing.Callable[[], Pose2d],
                ) -> None:
        # Command Attributes
        self.visionDataRecieved = visionDataRecieved
        self.resetSwerveGyro = resetSwerveGyro
        self.getVisionPose = getVisionPose
        self.setName( "AwaitVisionData" )
        # self.addRequirements( visionSys )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        if self.visionDataRecieved(): 
            self.resetSwerveGyro( self.getVisionPose() )
            FalconLogger.logOutput('/thingy/has synced gyro', True)

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return self.visionDataRecieved()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return True