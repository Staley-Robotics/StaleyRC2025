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
        self.timer.stop()
        self.timer.reset()
        self.setName( "CoralHold" )
    
    def execute(self):
        if not self.wheelSys.ls.get():
            if not self.timer.isRunning():
                self.timer.restart()
            self.wheelSys.setSpeed(CoralManipulatorWheel.WheelSpeeds.IN)
        else:
            self.timer.reset()
            self.wheelSys.setSpeed(CoralManipulatorWheel.WheelSpeeds.SLIGHT_IN)

    def end(self, interrupted):
        if not interrupted: # lost coral
            self.wheelSys.set_has_coral(False)
            self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.STOP )
        # else: # ejecting / failed and intaking
    
    def isFinished(self):
        return self.timer.hasElapsed( 1.5 )