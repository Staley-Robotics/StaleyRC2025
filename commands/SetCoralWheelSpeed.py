from typing import Callable

from commands2 import Command

from subsystems import CoralManipulatorWheel

class SetCoralWheelSpeed(Command):
    def __init__(self, coralWheelSys:CoralManipulatorWheel,
                 desiredSpeed:CoralManipulatorWheel.WheelSpeeds,
                 cmdID:str):
        self.wheelSys = coralWheelSys
        self.desiredSpeed = desiredSpeed

        self.addRequirements(coralWheelSys)
        self.setName( f'{self.__class__.__name__}:{cmdID}' )
    
    def initialize(self):
        self.wheelSys.setSpeed( self.desiredSpeed )
    
    def execute(self):
        pass
    
    def end(self, interrupted):
        self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.STOP )
    
    def isFinished(self):
        return False
