from commands2 import Subsystem

import typing

from ntcore import NetworkTableInstance

from wpilib import getTime

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d
from wpimath.units import degreesToRadians

from util import FalconLogger

class CamTypes:
    APRIL_TAG_BOT_POSE = 0 # for getting the position of the robot on the field
    APRIL_TAG_REL_POS = 1 # for getting position of the robot relative to a particular tag
    DETECTOR = 2 # Nueral net object detection and location (single type of obj)
    CLASSIFIER = 3 # Nueral net object classification ( & location?) (multiple obj types)

class Camera:
    def __init__(self, id:str, mode:CamTypes, poseEstimator:typing.Callable[[], SwerveDrive4PoseEstimator]):
        self.id = id
        self.mode = mode
        self.estimatorRef = poseEstimator

        table = NetworkTableInstance.getDefault().getTable(id)
        self.poseSub = table.getDoubleArrayTopic('botpose_wpiblue').subscribe([])
        self.targetSub = table.getDoubleArrayTopic('t2d').subscribe([])

        self.last_pose:Pose2d = None
    
    def array2d_to_botpose(self, data:list[float]) -> Pose2d:
        return Pose2d( data[0], data[1], degreesToRadians(data[5]) )
    
    def get_last_pose(self) -> Pose2d:
        return self.last_pose
    
    def update_botpose(self) -> bool:
        '''
        assigns most recent vision data from this camera to the pose estimator
        :returns: whether or not there was any data to assign
        '''
        data = self.poseSub.readQueue()

        for pose_data in data:
            if pose_data.value[0] != 0:
                # FalconLogger.logOutput('thingy/thingy', self.array2d_to_botpose(pose_data.value))
                self.last_pose = self.array2d_to_botpose(pose_data.value)
                # FalconLogger.logOutput('thingy/time', pose_data.time)
                # FalconLogger.logOutput('thingy/latency', pose_data.value[6])
                self.estimatorRef().addVisionMeasurement( self.array2d_to_botpose(pose_data.value), pose_data.time/1000000.0 - (pose_data.value[6]/1000), [1.,1.,999999.] )
        
        return True
    
    def update_rel_data(self) -> bool:
        ...
    def update_detector(self) -> bool:
        ...
    def update_classifier(self) -> bool:
        ...
    

class Vision(Subsystem):
    '''
    creates and handles all limelights & limelight data
    '''

    def __init__(self, poseEstimator:typing.Callable[[], SwerveDrive4PoseEstimator]):
        self.estimatorRef = poseEstimator

        # self.cameras = [
        #     Camera('UpperCam', CamTypes.DETECTOR, poseEstimator), #on the coral arm or something
        #     Camera('FrontCam', CamTypes.APRIL_TAG_BOT_POSE, poseEstimator),
        #     Camera('BackCam', CamTypes.APRIL_TAG_BOT_POSE, poseEstimator),
        #     Camera('RightCam', CamTypes.APRIL_TAG_BOT_POSE, poseEstimator),
        #     Camera('LeftCam', CamTypes.APRIL_TAG_BOT_POSE, poseEstimator),
        # ]
        self.cameras = [
            Camera('limelight-four', CamTypes.APRIL_TAG_BOT_POSE, poseEstimator)
        ]

        self.frames_without_data = -1
    
    def get_last_pose(self) -> Pose2d:
        return self.cameras[0].get_last_pose()

    def periodic(self):
        if self.frames_without_data >= 0:
            self.frames_without_data += 1 #if we've recieved any data before, assume we wont recieve data, gets reset later

        for cam in self.cameras:
            match cam.mode:
                case CamTypes.APRIL_TAG_BOT_POSE:
                    if cam.update_botpose():
                        self.frames_without_data = 0
                case CamTypes.APRIL_TAG_REL_POS:
                    cam.update_rel_data()
                case CamTypes.DETECTOR:
                    cam.update_detector()
                case CamTypes.CLASSIFIER:    
                    cam.update_classifier()    

    def has_recieved_first_botpose_data(self) -> bool:
        return self.frames_without_data >= 0