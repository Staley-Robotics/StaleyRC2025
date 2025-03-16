from commands2 import Subsystem

from wpilib import SmartDashboard, RobotBase, RobotState, Mechanism2d, Color8Bit, RobotController, Color
from wpilib.shuffleboard import Shuffleboard
from wpilib.simulation import ElevatorSim, RoboRioSim, BatterySim
from wpimath.system.plant import DCMotor
from wpimath.units import *
rotations_per_inch = float
rotations_per_meter = float

from rev import SparkMax, SparkBase, SparkMaxConfig, ClosedLoopConfig, ClosedLoopSlot, SparkMaxSim, LimitSwitchConfig, EncoderConfig

from math import pi

from util import FalconLogger

class ElevatorConstants:
    _kP = 3.
    _kI = 0.0
    _kD = 0.1
    _kG = 0.0 # force to overcome gravity
    _kS = 0.0 # force to overcome friction
    _kV = 0.0 # Apply __ voltage for target velocity
    _kFF = 0.0 # Feed Forward

    _kOffset = 0.0
    _kAtSetpointTolerance:inches = 0.1

    _pulleyDiameter:inches = 2.256
    _pulleyRadius:inches = 2.256 / 2

    _gearRatio = 4
    _carriageMass:kilograms = 10.0 # Estimated

    _motorRotsPerHeightInches =  1 / _gearRatio * (_pulleyDiameter * pi)

class Elevator(Subsystem):

    class ElevatorPositions:
        '''
        measured in inches from ground to center of coral pivot axle
        '''
        BOTTOM:inches = 34.5 # Minimum height
        TOP:inches = 76.0 # maximum height
        MIDDLE:inches = (BOTTOM + TOP) / 2

        TROUGH:inches = 0.0
        LOW_CORAL:inches = 0.0
        MED_CORAL:inches = 0.0
        HIGH_CORAL:inches = 0.0

    def __init__(self, leadMotorId:int, followMotorId:int, controller):
        ## Motor Init
        self.openSpeed = controller
        # motors
        self.__leadMotor = SparkMax(leadMotorId, SparkBase.MotorType.kBrushless)
        self.__followMotor = SparkMax(followMotorId, SparkBase.MotorType.kBrushless)

        # configuration
        __motorLeadConfig = SparkMaxConfig()
        __motorLeadConfig = __motorLeadConfig.setIdleMode( SparkMaxConfig.IdleMode.kBrake )
        __motorLeadConfig = __motorLeadConfig.inverted( True )

        __motorLeadEncoderConfig = EncoderConfig()
        __motorLeadEncoderConfig = __motorLeadEncoderConfig.positionConversionFactor(ElevatorConstants._motorRotsPerHeightInches).velocityConversionFactor(ElevatorConstants._motorRotsPerHeightInches / 60)
        #__motorLeadEncoderConfig = __motorLeadEncoderConfig.inverted( True )

        __motorLeadClosedLoopConfig = ClosedLoopConfig()
        #__motorLeadClosedLoopConfig =  __motorLeadClosedLoopConfig.setFeedbackSensor( ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder )#makes not work lol?
        __motorLeadClosedLoopConfig = __motorLeadClosedLoopConfig.pidf(
            ElevatorConstants._kP,
            ElevatorConstants._kI,
            ElevatorConstants._kD,
            ElevatorConstants._kFF,
            ClosedLoopSlot.kSlot0
            )

        __motorLeadLimitSwitchConfig = LimitSwitchConfig()
        __motorLeadLimitSwitchConfig = __motorLeadLimitSwitchConfig.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        __motorLeadLimitSwitchConfig = __motorLeadLimitSwitchConfig.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
        __motorLeadLimitSwitchConfig = __motorLeadLimitSwitchConfig.forwardLimitSwitchEnabled(False)
        __motorLeadLimitSwitchConfig = __motorLeadLimitSwitchConfig.reverseLimitSwitchEnabled(False)

        __motorLeadConfig.apply(__motorLeadEncoderConfig)
        __motorLeadConfig.apply(__motorLeadClosedLoopConfig)
        __motorLeadConfig.apply(__motorLeadLimitSwitchConfig)

        self.__leadMotor.configure(__motorLeadConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        __motorFollowConfig = SparkMaxConfig().inverted(True).follow(leadMotorId, False).apply(__motorLeadEncoderConfig)
        self.__followMotor.configure(__motorFollowConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        # grab motor objects
        self.__pidController = self.__leadMotor.getClosedLoopController()

        self.__leadEncoder = self.__leadMotor.getEncoder()
        self.__leadEncoder.setPosition( self.ElevatorPositions.BOTTOM )
        self.__followEncoder = self.__followMotor.getEncoder()

        self.__topSwitch = self.__leadMotor.getReverseLimitSwitch() # TODO: test behavior of limit switch w/ lead+follow motors
        self.__bottomSwitch = self.__leadMotor.getForwardLimitSwitch()

        self.__setpoint = 0.0

        ## Simulation
        self.__gearbox = DCMotor.NEO(2)

        self.__simMotor = SparkMaxSim(self.__leadMotor, self.__gearbox)
        self.__simMotor.setPosition( self.ElevatorPositions.BOTTOM )
        self.__simEncoder = self.__simMotor.getAbsoluteEncoderSim()

        self.__elevatorSim = ElevatorSim(
            self.__gearbox,
            ElevatorConstants._gearRatio,
            ElevatorConstants._carriageMass,
            inchesToMeters(ElevatorConstants._pulleyRadius),
            inchesToMeters(Elevator.ElevatorPositions.BOTTOM),
            inchesToMeters(Elevator.ElevatorPositions.TOP),
            simulateGravity=False,
            startingHeight=inchesToMeters(Elevator.ElevatorPositions.BOTTOM),
            measurementStdDevs=[0.01, 0.00]
        )

        self.mech = Mechanism2d(
            30,
            50,
            Color8Bit(Color.kBlack)
        )
        mechRoot = self.mech.getRoot(
            "ElevatorRoot",
            15,
            10
        )

        outerFrame = mechRoot.appendLigament("outerFrame", 10, 90, lineWidth=12, color=Color8Bit(Color.kRed))
        # self.mechElevator = self.mechBase.appendLigament("ElevatorImmutable", 10, 90, color=Color8Bit(Color.kRed))
        self.mechElevatorMutable = outerFrame.appendLigament("ElevatorMutable", 0, 0, lineWidth=12, color=Color8Bit(Color.kLimeGreen))
        self.mechElevatorSetpoint = outerFrame.appendLigament("ElevatorSetpoint", 0, 0, lineWidth=6, color=Color8Bit(Color.kWhite))

        SmartDashboard.putData("Elevator", self)
        SmartDashboard.putData("ElevatorMech", self.mech)


    def periodic(self):
        # Lead motor
        FalconLogger.logInput("Elevator/Lead/MotorInput", self.__leadMotor.get()) # current speed of motor
        FalconLogger.logInput("Elevator/Lead/MotorOutput", self.__leadMotor.getAppliedOutput())
        FalconLogger.logInput("Elevator/Lead/MotorPosition_r", self.__leadEncoder.getPosition())
        FalconLogger.logInput("Elevator/Lead/MotorVelocity_rpm", self.__leadEncoder.getVelocity())
        FalconLogger.logInput("Elevator/Lead/MotorCurrent_a", self.__leadMotor.getOutputCurrent())
        FalconLogger.logInput("Elevator/Lead/MotorTemp_c", self.__leadMotor.getMotorTemperature())

        # Follow motor
        FalconLogger.logInput("Elevator/Follow/MotorInput", self.__followMotor.get())  # current speed of motor
        FalconLogger.logInput("Elevator/Follow/MotorOutput", self.__followMotor.getAppliedOutput())
        FalconLogger.logInput("Elevator/Follow/MotorPosition_r", self.__followEncoder.getPosition())
        FalconLogger.logInput("Elevator/Follow/MotorVelocity_rpm", self.__followEncoder.getVelocity())
        FalconLogger.logInput("Elevator/Follow/MotorCurrent_a", self.__followMotor.getOutputCurrent())
        FalconLogger.logInput("Elevator/Follow/MotorTemp_c", self.__followMotor.getMotorTemperature())

        # Limit Switches
        FalconLogger.logInput("Elevator/TopLimitSwitch", self.__topSwitch.get())
        FalconLogger.logInput("Elevator/BottomLimitSwitch", self.__bottomSwitch.get())

        if RobotState.isDisabled():
            self.stop()
        
        ### Open loop
        # self.__leadMotor.set( self.openSpeed() )
        
        ## Safety Measure:
        #TODO: measure standard applied output and velocities to establish safe zones
        #NOTE encoder velocities are in in per sec
        # if (abs(self.__leadMotor.getAppliedOutput()) > 0.2 and abs(self.__leadEncoder.getVelocity()) < 2) or (abs(self.__followMotor.getAppliedOutput()) > 0.2 and abs(self.__followEncoder.getVelocity()) < 2):
        #     self.warn_frames += 1
        # else:
        #     self.warn_frames = 0
        
        # if self.warn_frames > 5:
        #     print('Elevator Stall detected, stopping')
        #     self.stop()

        # self.mechElevatorMutable.setLength(
        #     self.__elevatorSim.getPosition() * 10
        # )
        # self.mechElevatorSetpoint.setLength(
        #     inchesToMeters(self.getSetpoint()) * 10
        # )

        FalconLogger.logOutput("Elevator/CurrentHeight_in", self.getHeight())
        FalconLogger.logOutput("Elevator/TargetHeight_in", self.getSetpoint())
        FalconLogger.logOutput("Elevator/SimHeight_in", self.__elevatorSim.getPositionInches())
    
    def simulationPeriodic(self):
        self.__elevatorSim.setInputVoltage( 12 * self.__simMotor.getAppliedOutput() )
        self.__elevatorSim.update( 0.02 )

        vel_ips = self.__elevatorSim.getVelocityFps() * 12

        self.__simMotor.setMotorCurrent( 0 )

        self.__simMotor.iterate(
            vel_ips,
            12,
            0.02
        )
    
    def stop(self) -> None:
        self.setSetpoint(self.getHeight())
    
    def getSetpoint(self) -> inches:
        return self.__setpoint
    
    def setSetpoint(self, setpoint: inches) -> None:
        self.__setpoint = (max(min(setpoint, Elevator.ElevatorPositions.TOP), Elevator.ElevatorPositions.BOTTOM))# * ElevatorConstants._rotsPerInch * ElevatorConstants._gearing
        
        # self.__pidController.setReference(
        #     self.__setpoint,
        #     SparkBase.ControlType.kPosition, 
        #     ClosedLoopSlot.kSlot0
        # )
    
    def setOpenControl(self, speedPercentage:float) -> None:
        self.__leadMotor.set( speedPercentage )

    def atSetpoint(self) -> bool:
        margin = self.getHeight() - self.getSetpoint()
        return abs(margin) <= ElevatorConstants._kAtSetpointTolerance

    def getHeight(self) -> inches:
        """
        Get the current height of the elevator (ground to coral pivot axle)
        """
        return self.__leadEncoder.getPosition()

    def getSwitchesState(self) -> int:
        '''
        :return int: -1 for at bottom, 1 for at top, 0 for neither triggered (upper limit switch - lower limit switch )
        '''
        return self.__topSwitch.get() - self.__bottomSwitch.get()

    def resetPosition(self) -> None:
        '''
        resets position of both motor encoders assuming the elevator is at the bottom
        '''
        self.__leadEncoder.setPosition( Elevator.ElevatorPositions.BOTTOM )
        self.__followEncoder.setPosition( Elevator.ElevatorPositions.BOTTOM )
