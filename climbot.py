#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from wpilib import SmartDashboard, TimedRobot
from rev import SparkMax

from ntcore.util import ntproperty

import math

from util import FalconXboxController, FalconLogger


class MyRobot(TimedRobot):
    """
    This sample program shows how to control a motor using a joystick. In the operator control part
    of the program, the joystick is read and the value is written to the motor.

    Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
    making it easy to work together.

    In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
    to the Dashboard.
    """
    controlMult = ntproperty('controlMult', 0.5)


    def robotInit(self):
        """Robot initialization function"""

        self.motor = SparkMax( 1, SparkMax.MotorType.kBrushless )

        self.driver1 = FalconXboxController( 0 )

    def robotPeriodic(self):
        """The RobotPeriodic function is called every control packet no matter the robot mode."""
        FalconLogger.logInput("MotorInput", self.motor.get())
        FalconLogger.logInput("MotorOutput", self.motor.getAppliedOutput())
        FalconLogger.logInput("MotorCurrent_a", self.motor.getOutputCurrent())
        FalconLogger.logInput("MotorPosition_r", self.motor.getEncoder().getPosition())
        FalconLogger.logInput("MotorVelocity_rpm", self.motor.getEncoder().getVelocity())
        FalconLogger.logInput("MotorTemp_c", self.motor.getMotorTemperature())

    def teleopPeriodic(self):
        self.motor.set(self.driver1.getLeftUpDown() * self.controlMult)