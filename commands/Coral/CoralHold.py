from typing import Callable
from wpilib import Timer

from commands2 import Command

from subsystems import CoralManipulatorWheel

class CoralHold(Command):
    '''
    Checks for coral detected
    attempts intake on coral lost
    ends when coral is regained or has been lost for > 1 second
    '''
    def __init__(self, coralWheelSys:CoralManipulatorWheel) -> None:
        self.wheelSys = coralWheelSys

        self.timer = Timer()

        self.addRequirements(coralWheelSys)
        self.setName( f'{self.__class__.__name__}' )
    
    def initialize(self):
        # self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.OUT )
        ...
    
    def execute(self):
        if not self.wheelSys.hasCoral():
            self.timer.start()
            self.wheelSys.setSpeed(CoralManipulatorWheel.WheelSpeeds.SLIGHT_IN)

    def end(self, interrupted):
        self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.STOP )
    
    def isFinished(self):
        return (self.wheelSys.hasCoral() and self.timer.isRunning() ) or self.timer.get() >= 1
