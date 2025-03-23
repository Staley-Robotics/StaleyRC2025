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
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from commands.Drive.DriveByStick import DriveByStick

class DriveByStickRotate(DriveByStick):
    def __init__( self,
                mySubsystem:SwerveDrive,
                frcFwd: typing.Callable[[], float] = lambda: 0.0,
                frcLeft: typing.Callable[[], float] = lambda: 0.0,
                frcRotation: typing.Callable[[], float] = lambda: 0.0,
                frcRotation2d: typing.Callable[[], Rotation2d] = lambda: Rotation2d(0.0),
            ) -> None:

        # Command Attributes
        super().__init__( mySubsystem, frcFwd, frcLeft, frcRotation )
        self.setName( "DriveByStickRotation" )
        self.__getTarget = frcRotation2d

        self.turnPID = ProfiledPIDControllerRadians(
            3.5, 0, 0,
            TrapezoidProfileRadians.Constraints(
                DriveConstants.kMaxRotationSpeed,
                DriveConstants.kMaxRotationSpeed / 2
            )
        )
        self.turnPID.enableContinuousInput( -math.pi, math.pi )
        SmartDashboard.putData( "DriveByStickRotation_PID", self.turnPID )

    def initialize(self) -> None:
        robotAngle = self.subsystem.getRobotAngle()
        self.turnPID.reset( robotAngle.radians() )

    def execute(self) -> None:
        currentAngle = self.subsystem.getRobotAngle()
        targetAngle = self.__getTarget()
        rotationValue = self.turnPID.calculate( currentAngle.radians(), targetAngle.radians() )
        self.subsystem.runPercentInputs( self.getX(), self.getY(), rotationValue )

    def isFinished(self) -> bool:
        return self.getR() != 0.0