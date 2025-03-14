from wpilib import XboxController, SmartDashboard
from wpimath import applyDeadband
from commands2 import Subsystem, Command, cmd
from phoenix6 import hardware, controls

class SwerveTest(Subsystem):
    def __init__(self):
        self.flDrive = hardware.TalonFX( 1, 'canivore1' )
        self.flRotate = hardware.TalonFX( 2, 'canivore1' )
        self.flEncoder = hardware.CANcoder( 12, 'canivore1' )

        self.frDrive = hardware.TalonFX( 3, 'canivore1' )
        self.frRotate = hardware.TalonFX( 4, 'canivore1' )
        self.frEncoder = hardware.CANcoder( 14, 'canivore1' )

        self.blDrive = hardware.TalonFX( 5, 'canivore1' )
        self.blRotate = hardware.TalonFX( 6, 'canivore1' )
        self.blEncoder = hardware.CANcoder( 16, 'canivore1' )

        self.brDrive = hardware.TalonFX( 7, 'canivore1' )
        self.brRotate = hardware.TalonFX( 8, 'canivore1' )
        self.brEncoder = hardware.CANcoder( 18, 'canivore1' )

        SmartDashboard.putData( "SwerveTest", self )

    def periodic(self):
         if self.getDefaultCommand() == None:
            self.flDrive.set( 0 )
            self.flRotate.set( 0 )
            self.frDrive.set( 0 )
            self.frRotate.set( 0 )
            self.blDrive.set( 0 )
            self.blRotate.set( 0 )
            self.brDrive.set( 0 )
            self.brRotate.set( 0 )

class SwerveDefault(Command):
    driver1 = XboxController(0)
    pos:int = 0

    def __init__(self, sys:SwerveTest):
        self.sys = sys
        self.addRequirements( sys )

    def execute(self):
        x = applyDeadband( -self.driver1.getLeftY(), 0.04 )
        y = applyDeadband( -self.driver1.getLeftX(), 0.04 )
        print( x, y )

        if self.pos == 1:
            self.sys.flDrive.set( y )
            self.sys.flRotate.set( x )
        else:
            self.sys.flDrive.set( 0 )
            self.sys.flRotate.set( 0 )

        if self.pos == 2:
            self.sys.frDrive.set( y )
            self.sys.frRotate.set( x )
        else:
            self.sys.frDrive.set( 0 )
            self.sys.frRotate.set( 0 )

        if self.pos == 3:
            self.sys.blDrive.set( y )
            self.sys.blRotate.set( x )
        else:
            self.sys.blDrive.set( 0 )
            self.sys.blRotate.set( 0 )

        if self.pos == 4:
            self.sys.brDrive.set( y )
            self.sys.brRotate.set( x )
        else:
            self.sys.brDrive.set( 0 )
            self.sys.brRotate.set( 0 )

        if self.driver1.getXButton():
            self.pos = 1
        if self.driver1.getYButton():
            self.pos = 2
        if self.driver1.getAButton():
            self.pos = 3
        if self.driver1.getBButton():
            self.pos = 4
