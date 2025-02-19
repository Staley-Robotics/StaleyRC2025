import typing

from commands2 import Subsystem

from ntcore import NetworkTableInstance

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d
from wpimath.units import degreesToRadians

class VisionCamera(Subsystem):
    def __init__(self, sysID:str, odometryGetter:typing.Callable[[], SwerveDrive4PoseEstimator]):
        ## Subsystem setup
        self.sysID = sysID
        self.get_odometry = odometryGetter
    
        ## Networktable setup
        table = NetworkTableInstance.getDefault().getTable(id)
        self.poseSub = table.getDoubleArrayTopic('botpose_wpiblue').subscribe([])
        # self.targetSub = table.getDoubleArrayTopic('t2d').subscribe([])

        self.last_pose = Pose2d()

    def array2d_to_botpose(self, data:list[float]) -> Pose2d:
        '''
        converts the array2d object recieved from the limelight into a Pose2d object
        '''
        return Pose2d( data[0], data[1], degreesToRadians(data[5]) )
    
    def update_botpose(self) -> bool:
        '''
        adds all new vision data on this camera to SS's odometry class
        :returns bool: True if there was new data
        '''
        data = self.poseSub.readQueue()
        hadNewData = False

        for pose_data in data:
            if pose_data.value[0] != 0:
                hadNewData = True

                self.last_pose = self.array2d_to_botpose(pose_data.value)
                self.estimatorRef().addVisionMeasurement( self.last_pose, pose_data.time/1000000.0 - (pose_data.value[6]/1000), [1.,1.,999999.] )

        return hadNewData
    
    def periodic(self):
        
        self.update_botpose()

'''
limelight pose_data reference:
0: field relative x position (meters)
1: field relative y position (meters)
2:
3:
4:
5: field relative rotation (degrees)
6: total latency of most recent measurement (milliseconds)

time: global time when measurement was added to networktables (nanoseconds?)
'''