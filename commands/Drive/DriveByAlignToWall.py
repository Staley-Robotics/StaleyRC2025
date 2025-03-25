import typing
import math

from wpilib import SmartDashboard

from wpimath import applyDeadband
from wpimath.units import degrees
from commands2 import Command

from util import FalconLogger
from .DriveConstants import *
from subsystems import SwerveDrive

from wpimath.geometry import Rotation2d
from wpimath.controller import ProfiledPIDControllerRadians, PIDController
from wpimath.trajectory import TrapezoidProfileRadians

class DriveByAlignToWall:
    def __init__( self,
                mySubsystem:SwerveDrive,
                frcLeft: typing.Callable[[], float] = lambda: 0.0,
                frcRotation2d: typing.Callable[[], Rotation2d] = lambda: Rotation2d(0.0),
            ) -> None:

        # Command Attributes
        self.setName( "DriveByAlignToWall" )
        self.__getTarget = frcRotation2d
        FalconLogger.logOutput('thingy/targetAngle', Rotation2d())

        # self.turnPID = ProfiledPIDControllerRadians(
        #     3.5, 0, 0,
            # TrapezoidProfileRadians.Constraints(
                # DriveConstants.kMaxRotationSpeed,
            #     DriveConstants.kMaxRotationSpeed / 2
            # )
        # )
        self.turnPID = PIDController(1.05,0,0.0)
        self.turnPID.enableContinuousInput( -math.pi, math.pi )
        # self.turnPID.\
        SmartDashboard.putData( "DriveByStickRotation_PID", self.turnPID )

    def initialize(self) -> None:
        robotAngle = self.subsystem.getRobotAngle()
        # self.turnPID.reset()

    def execute(self) -> None:
        # self.turnPID = SmartDashboard.getData('DriveByStickRotation_PID')
        currentAngle = self.subsystem.getRobotAngle()
        targetAngle = self.__getTarget()
        self.turnPID.setSetpoint( targetAngle.radians())

        rotationValue = applyDeadband(self.turnPID.calculate( currentAngle.radians() ) / math.pi, 0.03)
        FalconLogger.logOutput('thingy/rotValue', rotationValue)

        self.subsystem.runPercentInputs( self.getX(), self.getY(), rotationValue / self.subsystem.getDriverMaxRotation() * DriveConstants.kMaxRotationSpeed)

        FalconLogger.logOutput('thingy/targetAngle', targetAngle)

    def isFinished(self) -> bool:
        return self.getR() != 0.0