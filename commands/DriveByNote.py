import typing
import math

from commands2 import Command

from ntcore.util import ntproperty

from wpilib import SmartDashboard, getTime

from wpimath import applyDeadband
from wpimath.controller import ProfiledPIDControllerRadians, PIDController
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.units import degreesToRadians
from wpimath.kinematics import ChassisSpeeds

from subsystems import SwerveDrive
from subsystems import VisionCameraDetector
from commands.DriveByStick import DriveByStick
from commands.DriveConstants import Constants

from util import FalconLogger

class DriveByNote(Command):
    def __init__(self,
                  swerve:SwerveDrive,
                  detectorCam:VisionCameraDetector,
                  frcFwd: typing.Callable[[], float] = lambda: 0.0,
                  frcLeft: typing.Callable[[], float] = lambda: 0.0,
                ) -> None:
        
        self.setName( "DriveByNote" )
        self.addRequirements( swerve, detectorCam )

        self.__swerve:SwerveDrive = swerve
        self.__cam = detectorCam
        
        self.__getX = frcFwd
        self.__getY = frcLeft
        
        self.desired_position = 0

        self.turnPID = ProfiledPIDControllerRadians(
            1.8,
            0.0,
            0.0,
            TrapezoidProfileRadians.Constraints(
                math.tau*2,
                math.pi*2
            )
        )
        # self.turnPID = PIDController(1.0,0.0,0.0)
        self.turnPID.enableContinuousInput(0, math.tau)
        SmartDashboard.putData("/NoteTracker/turnPIDstandard", self.turnPID)

        # self.has_seen_note = False

    
    def initialize(self):
        self.desired_position = self.__swerve.getRobotAngle().radians()

        self.turnPID.reset( self.__swerve.getRobotAngle().radians() )

        # self.has_seen_note = False
    
    def execute(self) -> None:
        ## Update vision data
        self.__cam.update_data()
        #logging
        FalconLogger.logInput("/DriveByNoteStats/Cam Larency", self.__cam.data['latency'])
        

        ## Evaluate action based on note position 
        if self.__cam.hasRecievedUpdate() and self.__cam.hasTarget():
            self.desired_position = self.__swerve.getRobotAngle( getTime() - self.__cam.data['latency'] ).radians() - degreesToRadians(self.__cam.data['hori_dist'])
            
        
        omega = applyDeadband(self.turnPID.calculate( self.__swerve.getRobotAngle().radians(), self.desired_position ), 0.01)

        # run drivetrain
        # self.__swerve.runPercentInputs( self.getX(), self.getY(), self.turnPID.calculate(self.__swerve.getRobotAngle().radians(), self.desired_position ))
        #utilizes code from runPercentInputs, then injects rotation data for some scenarios
        x = self.getX()
        y = self.getY()
        # omega = self.desired_position

        x = min( max( x, -1.0 ), 1.0 )
        y = min( max( y, -1.0 ), 1.0 )
        # omega = min( max( omega, -1.0 ), 1.0 )

        xSpeed = x * self.__swerve.getDriverMaxSpeed()
        ySpeed = y * self.__swerve.getDriverMaxSpeed()
        omegaSpeed = omega * self.__swerve.getDriverMaxRotation()

        cSpeed = ChassisSpeeds( xSpeed, ySpeed, omegaSpeed )

        self.__swerve.runChassisSpeeds( cSpeed )

        FalconLogger.logOutput('/DriveByNoteStats/desired position', self.desired_position)
        FalconLogger.logOutput('/DriveByNoteStats/pid output', omega)
        FalconLogger.logOutput('/DriveByNoteStats/final rotation speed', omegaSpeed)

    def end(self, interrupted):
        ...

    
    #  # Run By Percentage
    # def runPercentInputs(self, x:float, y:float, omega:float) -> None:
    #     # Range Tolerances
    #     x = min( max( x, -1.0 ), 1.0 )
    #     y = min( max( y, -1.0 ), 1.0 )
    #     omega = min( max( omega, -1.0 ), 1.0 )

    #     xSpeed = x * self.__DriveMaxSpeedPercent * SwerveDriveConstants.kMaxSpeed
    #     ySpeed = y * self.__DriveMaxSpeedPercent * SwerveDriveConstants.kMaxSpeed
    #     omegaSpeed = omega * self.__DriveMaxRotationPercent * SwerveDriveConstants.kRotationSpeed

    #     cSpeed = (
    #         ChassisSpeeds.fromFieldRelativeSpeeds( xSpeed, ySpeed, omegaSpeed, self.getRobotAngle() )
    #         if self.__DriveFieldRelative
    #         else ChassisSpeeds( xSpeed, ySpeed, omegaSpeed )
    #     )
    #     self.runChassisSpeeds( cSpeed )


    # calls __getX lambda with Slew Rate Limiter Integration
    def getX(self, close:bool = False) -> float:
        x = self.__getX()
        x_normal = Constants.Limiters.srl_tX.calculate( x )
        x_close = Constants.Limiters.srl_tX_close.calculate( x )
        return x_normal if not close else x_close
    
    # calls __getX lambda with Slew Rate Limiter Integration
    def getY(self, close:bool = False) -> float:
        y = self.__getY()
        y_normal = Constants.Limiters.srl_tY.calculate( y )
        y_close = Constants.Limiters.srl_tY_close.calculate( y )       
        return y_normal if not close else y_close
    
    # # calls __getRotation lambda with Slew Rate Limiter Integration
    # def getR(self, close:bool = False) -> float:
    #     r = self.__getRotation()
    #     r_normal = Constants.Limiters.srl_rO.calculate( r )
    #     r_close = Constants.Limiters.srl_rO_close.calculate( r )
    #     return r_normal if not close else r_close

    # def getR(self) -> float:
    #     return self.__cam.update_data()['hori_dist']