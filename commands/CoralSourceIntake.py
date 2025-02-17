from commands2 import Command

from subsystems import CoralManipulatorWheel

class CoralSourceIntake(Command):
    def __init__(self, coralWheelSys:CoralManipulatorWheel):
        self.wheelSys = coralWheelSys
    
    def initialize(self):
        self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.IN )
    
    def execute(self):
        ...
    
    def end(self, interrupted):
        self.wheelSys.stop()
    
    def isFinished(self):
        self.wheelSys.hasCoral()