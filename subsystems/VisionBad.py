from commands2 import Subsystem

from wpimath.estimator import SwerveDrive4PoseEstimator

from ntcore import *

class CameraModes:
    #check this is right with limelight
    DETECTOR="detector"
    APRIL_TAG="apriltag"

class Limelight:

    id:str = ""
    poseSub:DoubleArraySubscriber = None
    table:NetworkTable


    def __init__(self, id:int, mode:CameraModes) -> None:
        self.id = f"limelight{id}"
        self.mode = mode

        self.table = NetworkTableInstance.getDefault().getTable(self.id)

        self.reset_mode()
    
    def reset_mode(self):
        if self.mode == CameraModes.APRIL_TAG:
            self.poseSub:DoubleArraySubscriber = self.table.getDoubleArrayTopic("botpose_wpiblue").subscribe([0.0])
        else:
            self.poseSub:DoubleArraySubscriber = self.table#...
    
    def set_pipeline(self, pipeline_id:int):
        self.table.getEntry("pipeline").setInteger(pipeline_id)
    
    def process_queue(self, queue:list[TimestampedDoubleArray]) -> None:
        ...

class Vision(Subsystem):

    poseEstimator:SwerveDrive4PoseEstimator = None

    def __init__(self,
                 poseEstimator:SwerveDrive4PoseEstimator,
                 ) -> None:
        self.poseEstimator = poseEstimator

        self.cameras = [
            Limelight(1, CameraModes.DETECTOR),
            Limelight(2, CameraModes.APRIL_TAG),
            Limelight(3, CameraModes.APRIL_TAG),
            Limelight(4, CameraModes.APRIL_TAG),
        ]

    def periodic(self):
        self.poseEstimator.addVisionMeasurement(self.cameras[1].getMeasurement)