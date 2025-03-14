from typing import Callable

from commands2 import Command

from subsystems import CoralManipulatorWheel

class CoralWheelOpenControl(Command):
    def __init__(self, coralWheelSys:CoralManipulatorWheel,
                 getSpeedFunc:Callable[[], float] = lambda: 0.0):
        self.wheelSys = coralWheelSys
        self.getSetSpeed = getSpeedFunc

        self.addRequirements(coralWheelSys)
    
    def initialize(self):
        ...
    
    def execute(self):
        self.wheelSys.setSpeed( self.getSetSpeed() )
    
    def end(self, interrupted):
        self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.STOP )
    
    def isFinished(self):
        return False
