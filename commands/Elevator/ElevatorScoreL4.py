from commands2 import Command

from subsystems import Elevator, CoralManipulatorWheel, ElevatorPositions

class ElevatorScoreL4(Command):
    def __init__(self, elevatorSubsystem: Elevator, coralWheelSys:CoralManipulatorWheel):
        self.__elevator = elevatorSubsystem
        self.__coralWheelSys = coralWheelSys

        self.setName(f"{self.__class__.__name__}")
        self.addRequirements(self.__elevator, coralWheelSys)

    def initialize(self):
        if not self.__elevator.getHeight() > ElevatorPositions.L2:
            self.cancel()
        self.__elevator.setSetpoint( ElevatorPositions.L2 )

    def execute(self):
        # self.__elevator.setOpenControl( )
        ...
    
    def end(self, interrupted:bool):
        if not self.__coralWheelSys.ls.get():
            self.__coralWheelSys.set_has_coral(False)

    def isFinished(self):
        return self.__elevator.atSetpoint()
    
    def runsWhenDisabled(self):
        return False