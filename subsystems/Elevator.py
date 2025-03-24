import math
from commands2 import Subsystem

from wpilib import SmartDashboard, RobotBase, RobotState, Mechanism2d, Color8Bit, RobotController, Color
from wpilib.shuffleboard import Shuffleboard
from wpilib.simulation import ElevatorSim, RoboRioSim, BatterySim
from wpimath.system.plant import DCMotor
from wpimath.units import *
rotations_per_inch = float
rotations_per_meter = float

from rev import SparkMax, SparkBase, SparkMaxConfig, ClosedLoopConfig, ClosedLoopSlot, SparkMaxSim, LimitSwitchConfig, EncoderConfig


from util import FalconLogger

class ElevatorConstants:
    _kP = 0.2#0.4
    _kI = 0.0
    _kD = 0.0
    _kG = 0.0 # force to overcome gravity
    _kS = 0.0 # force to overcome friction
    _kV = 0.0 # Apply __ voltage for target velocity
    _kFF = 0.0#0.001 # Feed Forward

    _kOffset = 0.0
    _kAtSetpointTolerance:inches = 0.25

    _pulleyDiameter:inches = 2.256
    _pulleyRadius:inches = 2.256 / 2

    _gearRatio = 4.0
    _carriageMass:kilograms = 10.0 # Estimated

    _motorRotsPerHeightInches =  1 / _gearRatio * (_pulleyDiameter * math.pi)

class ElevatorPositions:
    """
    measured in inches from ground to center of coral pivot axle
    """
    BOTTOM:inches = 34.75 # Minimum height
    TOP:inches = 76.0 # maximum height
    MIDDLE:inches = (BOTTOM + TOP) / 2

    SOURCE:inches = BOTTOM

    L1:inches = 35.0
    L2:inches = 52.6
    L25:inches = (L2 + 3.0)
    L3:inches = 68.6
    L35:inches = (L3 + 3.0)
    L4:inches = TOP

class Elevator(Subsystem):
    __setpoint = 0.0

    def __init__(self, leadMotorId:int, followMotorId:int):
        ## Motor Init
        # motors
        self.__leadMotor = SparkMax(leadMotorId, SparkBase.MotorType.kBrushless)
        self.__followMotor = SparkMax(followMotorId, SparkBase.MotorType.kBrushless)

        self.__leadEncoder = self.__leadMotor.getEncoder()
        self.__followEncoder = self.__followMotor.getEncoder()
        self.__pidController = self.__leadMotor.getClosedLoopController()
        
        self.__leadEncoder.setPosition(ElevatorPositions.BOTTOM)

        self.__topSwitch = self.__leadMotor.getForwardLimitSwitch()
        self.__bottomSwitch = self.__leadMotor.getReverseLimitSwitch()

        # configuration
        lMotorCfg = SparkMaxConfig()
        lMotorCfg = lMotorCfg.setIdleMode( SparkMaxConfig.IdleMode.kBrake )
        lMotorCfg = lMotorCfg.inverted( True )

        fMotorCfg = SparkMaxConfig()
        fMotorCfg = fMotorCfg.setIdleMode( SparkMaxConfig.IdleMode.kBrake )
        fMotorCfg = fMotorCfg.inverted( True )
        fMotorCfg = fMotorCfg.follow( self.__leadMotor.getDeviceId(), False )

        clCfg = ClosedLoopConfig()
        clCfg = clCfg.pidf(
            ElevatorConstants._kP,
            ElevatorConstants._kI,
            ElevatorConstants._kD,
            ElevatorConstants._kFF,
            ClosedLoopSlot.kSlot0
        )
        clCfg = clCfg.positionWrappingEnabled( False )

        convFactor = ElevatorConstants._motorRotsPerHeightInches
        encConfig = EncoderConfig()
        encConfig = encConfig.positionConversionFactor( convFactor ).velocityConversionFactor( convFactor / 60)

        lsConfig = LimitSwitchConfig()
        # lsConfig = lsConfig.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        lsConfig = lsConfig.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        # lsConfig = lsConfig.forwardLimitSwitchEnabled(False)
        lsConfig = lsConfig.reverseLimitSwitchEnabled(True)

        # Apply Configs
        lMotorCfg.apply(clCfg)
        lMotorCfg.apply(encConfig)
        lMotorCfg.apply(lsConfig)

        # Apply Configs
        fMotorCfg.apply(encConfig)

        self.__leadMotor.configure(lMotorCfg, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        self.__followMotor.configure(fMotorCfg, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        # Mechanism2d
        mech = Mechanism2d( 30, 90, Color8Bit(50,50,70) )
        mechRoot = mech.getRoot( "ElevatorRoot", 15, 10 )
        mechFrame = mechRoot.appendLigament("outerFrame", 10, 90, lineWidth=12, color=Color8Bit(Color.kGray))
        self.mechElevatorTarget = mechFrame.appendLigament("ElevatorTarget", 0, 0, lineWidth=6, color=Color8Bit(Color.kYellow))
        self.mechElevatorActual = mechFrame.appendLigament("ElevatorActual", 0, 0, lineWidth=12, color=Color8Bit(Color.kGreen))
        if RobotBase.isSimulation(): self.mechElevatorSim = mechFrame.appendLigament("ElevatorSSim", 0, 0, lineWidth=12, color=Color8Bit(Color.kRed))

        SmartDashboard.putData("Elevator", self)
        SmartDashboard.putData("ElevatorMech", mech)

        ## Simulation
        self.__simMotor = SparkMaxSim(self.__leadMotor, DCMotor.NEO() )
        self.__simMotor.setPosition( ElevatorPositions.BOTTOM )
        #self.__simEncoder = self.__simMotor.getAbsoluteEncoderSim()

        self.__elevatorSim = ElevatorSim(
            DCMotor.NEO(2),
            ElevatorConstants._gearRatio,
            ElevatorConstants._carriageMass,
            inchesToMeters(ElevatorConstants._pulleyRadius),
            inchesToMeters(ElevatorPositions.BOTTOM),
            inchesToMeters(ElevatorPositions.TOP),
            simulateGravity=False,
            startingHeight=inchesToMeters(ElevatorPositions.BOTTOM),
            measurementStdDevs=[0.01, 0.00]
        )
        self.__elevatorSim.setState( inchesToMeters( self.getHeight() ), 0.0 )

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

        if self.__bottomSwitch.get() and self.getHeight() != ElevatorPositions.BOTTOM:
            self.__leadEncoder.setPosition( ElevatorPositions.BOTTOM )
            self.setSetpoint( self.getHeight() )

        if RobotState.isDisabled():
            self.stop()
        else: 
            self.run()
        
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

        self.mechElevatorActual.setLength( self.getHeight() - 34.0 )
        self.mechElevatorTarget.setLength( self.getSetpoint() - 34.0 )

        FalconLogger.logOutput("Elevator/ActualHeight_in", self.getHeight())
        FalconLogger.logOutput("Elevator/TargetHeight_in", self.getSetpoint())
        
    
    def simulationPeriodic(self):
        FalconLogger.logOutput("Elevator/SimHeight_in", self.__elevatorSim.getPositionInches())

        velocity_ips = self.__elevatorSim.getVelocityFps() * 12

        self.__simMotor.setMotorCurrent( 0 )
        self.__simMotor.iterate( velocity_ips, 12, 0.02 )

        self.__elevatorSim.setInputVoltage( 12 * self.__simMotor.getAppliedOutput() )
        self.__elevatorSim.update( 0.02 )

        self.mechElevatorSim.setLength( self.__elevatorSim.getPositionInches() - 34.0 )
            
    def run(self) -> None:
        self.__pidController.setReference(
            self.__setpoint,
            SparkBase.ControlType.kPosition, 
            ClosedLoopSlot.kSlot0
        )

    def stop(self) -> None:
        self.setSetpoint(self.getHeight())

    def getSetpoint(self) -> inches:
        return self.__setpoint
    
    def setSetpoint(self, setpoint: inches, override: bool = False) -> None:
        if not override:
            setpoint = (max(min(setpoint, ElevatorPositions.TOP), ElevatorPositions.BOTTOM))
        self.__setpoint = setpoint
    
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
        self.__leadEncoder.setPosition( ElevatorPositions.BOTTOM )
        self.__followEncoder.setPosition( ElevatorPositions.BOTTOM )
