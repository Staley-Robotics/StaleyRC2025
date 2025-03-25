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
from wpimath.controller import ProfiledPIDControllerRadians, PIDController, HolonomicDriveController
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

        self.controller = HolonomicDriveController(
            PIDController(5,0,0),
            PIDController(5,0,0),
            ProfiledPIDControllerRadians(
                3.5, 0, 0,
                TrapezoidProfileRadians.Constraints(
                    DriveConstants.kMaxRotationSpeed,
                    DriveConstants.kMaxRotationSpeed / 2
                )
        )
        )
        self.controller.getThetaController().enableContinuousInput(-math.pi,math.pi)

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