from typing import Callable
from wpilib import Timer

from commands2 import Command, SelectCommand, ConditionalCommand, cmd

from subsystems import CoralManipulatorWheel
from commands import CoralHold

class CoralDefault(ConditionalCommand):
    '''
    Checks for coral being detected/held
    if detected, run CoralHold()
    else, do nothing, restart check next frame
    '''
    def __init__(self, coralWheelSys:CoralManipulatorWheel) -> None:
        self.wheelSys = coralWheelSys

        a = cmd.none()
        a.setName( "DefaultCoral-None" )

        # super().__init__(
        #     {
        #         True: CoralHold( coralWheelSys ),
        #         False: a #cmd.none()
        #     },#coralWheelSys.runEnd(lambda:None, lambda:None)}
        #     coralWheelSys.hasCoral
        # )

        super().__init__(
            CoralHold( coralWheelSys ),
            a,
            coralWheelSys.hasCoral
        )

        self.addRequirements(coralWheelSys)
        self.setName( f'{self.__class__.__name__}' )
    
    # def initialize(self):
    #     super().initialize()

    def execute(self):
        super().execute()

        if self.selectedCommand != None:
            self.setName( self.selectedCommand.getName() )

    #     # self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.OUT )
    #     ...
    
    # def execute(self):
    #     if not self.wheelSys.hasCoral():
    #         self.timer.start()
    #         self.wheelSys.setSpeed(CoralManipulatorWheel.WheelSpeeds.IN)

    # def end(self, interrupted):
    #     self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.STOP )
    
    # def isFinished(self):
    #     return False #...
