import math, typing

from commands2 import SequentialCommandGroup, ParallelCommandGroup, SelectCommand

from subsystems import *
from commands import *
from util import ReefScape


class CoralIO(SelectCommand):
    def __init__(self, coralSys: CoralManipulatorWheel):
        super().__init__(
            {
                False:CoralWheelIn(coralSys),
                True:CoralWheelOut(coralSys),
            },
            coralSys.hasCoral
        )
        
        self.setName("CoralIO")
        self.addRequirements(coralSys)
