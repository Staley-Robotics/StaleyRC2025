from commands2 import Subsystem

from ntcore import NetworkTableInstance

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d
from wpimath.units import degreesToRadians

class CamTypes:
    APRIL_TAG_BOT_POSE = 0 # for getting the position of the robot on the field
    APRIL_TAG_REL_POS = 1 # for getting position of the robot relative to a particular tag
    DETECTOR = 2 # Nueral net object detection and location (single type of obj)
    CLASSIFIER = 3 # Nueral net object classification ( & location?) (multiple obj types)

class Camera:
    def __init__(self, id:str, mode:CamTypes, poseEstimator:SwerveDrive4PoseEstimator):
        self.id = id
        self.mode = mode
        self.estimatorRef = poseEstimator

        table = NetworkTableInstance.getDefault().getTable(id)
        self.poseSub = table.getDoubleArrayTopic('botpose').subscribe([])
        self.targetSub = table.getDoubleArrayTopic('t2d').subscribe([])
    
    def array2d_to_botpose(self, data:list[float]) -> Pose2d:
        return Pose2d( data[0], data[1], degreesToRadians(data[5]) )
    
    def update_botpose(self) -> bool:
        '''
        assigns most recent vision data from this camera to the pose estimator
        :returns: whether or not there was any data to assign
        '''
        data = self.poseSub.readQueue()

        if data == []:
            return False

        for data in data:
            self.estimatorRef.addVisionMeasurement( self.array2d_to_botpose(data.value), data.time - data.value[6]/1000 )
        
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

    def __init__(self, poseEstimator:SwerveDrive4PoseEstimator):
        self.estimatorRef = poseEstimator

        self.cameras = [
            Camera('UpperCam', CamTypes.DETECTOR, poseEstimator), #on the coral arm or something
            Camera('FrontCam', CamTypes.APRIL_TAG_BOT_POSE, poseEstimator),
            Camera('BackCam', CamTypes.APRIL_TAG_BOT_POSE, poseEstimator),
            Camera('RightCam', CamTypes.APRIL_TAG_BOT_POSE, poseEstimator),
            Camera('LeftCam', CamTypes.APRIL_TAG_BOT_POSE, poseEstimator),
        ]

        self.frames_without_data = -1

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