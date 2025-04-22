# from wpilib import ...
from pyfrc.physics.core import PhysicsInterface, PhysicsEngine

from wpilib import Mechanism2d, Color8Bit, Color, SmartDashboard
from wpilib.simulation import SingleJointedArmSim
from wpimath.system.plant import DCMotor

from wpimath.units import *

from rev import SparkMaxSim

from subsystems import ElevatorPositions, AlgaeManipulatorPositions, AlgaeManipulatorConstants, CoralPivotPositions
from robot import MyRobot

class MechColors:
    background = Color8Bit(50,50,70)
    inert = Color8Bit(Color.kGray)
    measured = Color8Bit(Color.kGreen)
    setpoint = Color8Bit(Color.kYellow)

class PhysicsEngine:
    """
    will be used by robotpy for simulation

    Simulates Sharkbait, including the:
        - Drivetrain maybe? (currently handled in subsystem)
        - Algae Manipulator
        - Coral Manipulator
        - Elevator
        - Climber
    """
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot) -> None:
        self.phys_controller = physics_controller
        self.robot = robot

        ## Mechanism
        '''
        Mech Info:
        bottom left is (0,0)
        up is positive y, right is positive x
        rotation: counterclockwise positive & in degrees
        '''
        self.mech = Mechanism2d(50, 80, MechColors.background)
        frame_left_edge = self.mech.getRoot('frame_left_edge', 0, 1.5)
        frame_left = frame_left_edge.appendLigament('frame_left', 13, 0, color=MechColors.inert)
        elevator_base = frame_left.appendLigament('elevator_base', 1, 90, color=MechColors.inert)
        frame_right = frame_left.appendLigament('frame_right', 13, 0, color=MechColors.inert)

        self.elevator_mech_measured = elevator_base.appendLigament('elevator', ElevatorPositions.BOTTOM, 0, color=MechColors.measured )
        self.elevator_mech_setpoint = elevator_base.appendLigament('elevator_setpoint', ElevatorPositions.BOTTOM, 0, lineWidth=3, color=MechColors.setpoint )

        coral_mount = self.elevator_mech_measured.appendLigament('coral_mount', 3, -90, color=MechColors.inert)

        self.coral_mech_measured = coral_mount.appendLigament('coral', 19, CoralPivotPositions.START, color=MechColors.measured )
        self.coral_mech_setpoint = coral_mount.appendLigament('coral_setpoint', 19/2, CoralPivotPositions.START, lineWidth=3, color=MechColors.setpoint )
        
        self.algae_mech_measured = frame_right.appendLigament('algae', 18.5, AlgaeManipulatorPositions.START, color=MechColors.measured )
        self.algae_mech_setpoint = frame_right.appendLigament('algae_setpoint', 18.5/2, AlgaeManipulatorPositions.START, lineWidth=3, color=MechColors.setpoint )

        # might have to be on seperate mech
        # climber_mech_measured = elevator_base.appendLigament('climber', 19, CoralPivotPositions.START, color=MechColors.measured )
        # climber_mech_setpoint = elevator_base.appendLigament('climber_setpoint', 19/2, CoralPivotPositions.START, lineWidth=3, color=MechColors.setpoint )

        SmartDashboard.putData('FullMech', self.mech)

        ### Subsystems
        ## Algae
        self.algaeArmSim = SingleJointedArmSim(
            DCMotor.NEO550(2),
            AlgaeManipulatorConstants.pivot_kGearRatio,
            SingleJointedArmSim.estimateMOI( AlgaeManipulatorConstants.length, AlgaeManipulatorConstants.weight ), # NOTE: these are random numbers
            armLength=AlgaeManipulatorConstants.length,
            minAngle=degreesToRadians( -15.0 ),
            maxAngle=degreesToRadians( 100.0 ),
            simulateGravity=True, #Gravity
            startingAngle=degreesToRadians( 90.0 ),
        )

        self.algaePivotMotor = SparkMaxSim(robot.robotContainer.sysAlgae.leadMotor, DCMotor.NEO(2))
        self.algaeIntakeMotor = robot.robotContainer.sysAlgae.intakeMotor.getSimCollection()

    def update_sim(self, now:float, dt:float) -> None:
        """
        called when the simulation 'needs to be updated' (still typically ~0.02 secs, but does vary)
        """
        ## Battery & Rio
        ...

        ## Algae
        # Neo Periodic
        #driveRpm = AlgaeManipulatorConstants.NeoSim.kMaxRpm * self.__leadMotor.getAppliedOutput()
        driveRadps = self.algaeArmSim.getVelocity()
        driveRpm = radiansToRotations( driveRadps ) * 60

        self.algaePivotMotor.setMotorCurrent(0)
        self.algaePivotMotor.iterate( driveRpm , 12, 0.02)#dt)
        self.algaePivotMotor.getRelativeEncoderSim().iterate( driveRpm * AlgaeManipulatorConstants.pivot_kGearRatio, dt )
        #self.simPivot.getAbsoluteEncoderSim().iterate(driveRpm / AlgaeManipulatorConstants.pivot_kGearRatio, 0.02)

        # self.simPivot.getRelativeEncoderSim().setVelocity(driveRpm)
        # self.simPivot.getAbsoluteEncoderSim().setVelocity(driveRpm)
        # self.simPivot.getAbsoluteEncoderSim().setPosition(self.simPivot.getRelativeEncoderSim().getPosition() % 1)

        # 775pro Periodic
        velocity = AlgaeManipulatorConstants.Vex775Sim.kMaxRpm * 0.5#self.algaeIntakeMotor.getMotorOutputPercent()
        velocity_Tp100ms = int( velocity * 2048 / 60 / 10 ) # RPM * Ticks/Rot * 1 M/60sec * 1 sec / 10 (100ms)
        self.algaeIntakeMotor.setAnalogVelocity(velocity_Tp100ms)
        self.algaeIntakeMotor.addQuadraturePosition( int( velocity_Tp100ms * 0.02))#dt ) )

        self.algae_mech_measured.setAngle( self.algaeArmSim.getAngleDegrees() - 90.0 )
        self.algae_mech_setpoint.setAngle( rotationsToDegrees(self.algaePivotMotor.getSetpoint()) )

        ## Update MOI when you have Algae?
        # if self.hasAlgae():
        #     self.simPivotArm.setInput()
        # else:
        #     self.simPivotArm.estimateMOI()

        self.algaeArmSim.setInputVoltage( self.algaePivotMotor.getAppliedOutput() * 12.0 )
        self.algaeArmSim.update( 0.02)#dt )

        ## Coral Pivot
        ...

        ## Coral Manipulator
        ...

        ## Elevator
        ...

        ## Climber
        ...