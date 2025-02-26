import typing
import math

from wpilib import SmartDashboard

from wpimath import applyDeadband
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.units import degrees
from commands2 import Command

from util import FalconLogger
from .DriveConstants import *
from subsystems import SwerveDrive

class DriveByStickRotate(Command):
    # Deadband
    kDeadband = 0.04

    # Initialization
    # myX assumes forward is positive
    # myY assumes left is positive
    # myRotation assumes CCW is positive
    def __init__( self,
                  mySubsystem:SwerveDrive,
                  frcFwd: typing.Callable[[], float] = lambda: 0.0,
                  frcLeft: typing.Callable[[], float] = lambda: 0.0,
                  frcSnapAngle: typing.Callable[[], degrees] = lambda: -1
                ) -> None:
        # Command Attributes
        self.__subsystem:SwerveDrive = mySubsystem
        self.setName( "DriveByStick" )
        self.addRequirements( mySubsystem )

        self.turn_PID = ProfiledPIDControllerRadians(
            2, 0, 0,
            TrapezoidProfileRadians.Constraints(
                DriveConstants.kMaxRotationSpeed,
                DriveConstants.kMaxAcceleration
            )
        )
        self.turn_PID.enableContinuousInput(0, pi * 2)
        SmartDashboard.putData( '/Swerve/HTurnPID', self.turn_PID )

        self.desired_angle = mySubsystem.getRobotAngle().radians()

        self.__getX = frcFwd
        self.__getY = frcLeft
        self.__getSnapAngle = frcSnapAngle

    # On Start
    def initialize(self) -> None:
        self.turn_PID.reset( self.__subsystem.getRobotAngle().radians() )
        self.desired_angle = self.__subsystem.getRobotAngle().radians()

    # Periodic
    def execute(self) -> None:
        self.__subsystem.runPercentInputs( self.getX(), self.getY(), self.getR() )

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False

    # calls __getX lambda with Slew Rate Limiter Integration
    def getX(self, close:bool = False) -> float:
        x = self.__getX()
        x_normal = DriveConstants.Limiters.srl_tX.calculate( x )
        x_close = DriveConstants.Limiters.srl_tX_close.calculate( x )
        return x_normal if not close else x_close
        return self.__getX()

    # calls __getX lambda with Slew Rate Limiter Integration
    def getY(self, close:bool = False) -> float:
        y = self.__getY()
        y_normal = DriveConstants.Limiters.srl_tY.calculate( y )
        y_close = DriveConstants.Limiters.srl_tY_close.calculate( y )
        return y_normal if not close else y_close
        return self.__getY()

    # calls __getRotation lambda with Slew Rate Limiter Integration
    def getR(self, close:bool = False) -> float:
        # Check if D-PAD (POV) input is active.
        pov = self.__getSnapAngle()
        if pov != -1:
            self.desired_angle = math.radians(pov)
        
        # Get current angle
        current_angle = self.__subsystem.getRobotAngle().radians()

        # calculate
        rotation_speed = self.turn_PID.calculate( current_angle, self.desired_angle )

        # log
        FalconLogger.logOutput('/Swerve/currentAngle', current_angle)
        FalconLogger.logOutput('/Swerve/desiredAngle', self.desired_angle)

        return rotation_speed