from typing import Callable
from wpilib import Timer

from commands2 import Command, SelectCommand, cmd

from subsystems import CoralManipulatorWheel
from commands import CoralHold

class CoralDefault(SelectCommand):
    '''
    Checks for coral being detected/held
    if detected, run CoralHold()
    else, do nothing, restart check next frame
    '''
    def __init__(self, coralWheelSys:CoralManipulatorWheel) -> None:
        self.wheelSys = coralWheelSys

        super().__init__(
            {
                True: CoralHold( coralWheelSys ),
                False: cmd.none()
            },#coralWheelSys.runEnd(lambda:None, lambda:None)}
            coralWheelSys.hasCoral
        )

        self.addRequirements(coralWheelSys)
        self.setName( f'{self.__class__.__name__}' )
    
    # def initialize(self):
    #     # self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.OUT )
    #     ...
    
    # def execute(self):
    #     if not self.wheelSys.hasCoral():
    #         self.timer.start()
    #         self.wheelSys.setSpeed(CoralManipulatorWheel.WheelSpeeds.IN)

    # def end(self, interrupted):
    #     self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.STOP )
    
    def isFinished(self):
        return ...
