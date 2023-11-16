import wpilib

import ctre

from magicbot import MagicRobot

from networktables import NetworkTables
from networktables.util import ntproperty

from components import swervedrive, swervemodule

from collections import namedtuple

ModuleConfig = swervemodule.ModuleConfig

# Download and install stuff on the RoboRIO after imaging
'''
py -3 -m robotpy_installer download-python
py -3 -m robotpy_installer install-python
py -3 -m robotpy_installer download robotpy
py -3 -m robotpy_installer install robotpy
py -3 -m robotpy_installer download robotpy[ctre]
py -3 -m robotpy_installer install robotpy[ctre]
py -3 -m robotpy_installer download robotpy[rev]
py -3 -m robotpy_installer install robotpy[rev]
py -3 -m robotpy_installer download pynetworktables
py -3 -m robotpy_installer install pynetworktables
py -3 -m pip install -U robotpy[ctre]
py -3 -m pip install robotpy[ctre]
'''

# Push code to RoboRIO (only after imaging)
'''
python robot/robot.py deploy --skip-tests
py robot/robot.py deploy --skip-tests --no-version-check
'''


class MyRobot(MagicRobot):


    def createObjects(self):

        
        # self.frontLeftModule = swervemodule.SwerveModule()
        # self.frontRightModule = swervemodule.SwerveModule()
        # self.rearLeftModule = swervemodule.SwerveModule()
        # self.rearRightModule = swervemodule.SwerveModule()

        self.controller = wpilib.XboxController(0)

        self.frontLeftModule_driveMotor = ctre.WPI_TalonSRX(6)
        self.frontRightModule_driveMotor = ctre.WPI_TalonSRX(8)
        self.rearLeftModule_driveMotor = ctre.WPI_TalonSRX(3)
        self.rearRightModule_driveMotor = ctre.WPI_TalonSRX(1)

        self.frontLeftModule_rotateMotor = ctre.WPI_TalonSRX(5)
        self.frontRightModule_rotateMotor = ctre.WPI_TalonSRX(7)
        self.rearLeftModule_rotateMotor = ctre.WPI_TalonSRX(4)
        self.rearRightModule_rotateMotor = ctre.WPI_TalonSRX(2)

        self.frontLeftModule_encoder = self.frontLeftModule_rotateMotor
        self.frontRightModule_encoder = self.frontRightModule_rotateMotor
        self.rearLeftModule_encoder = self.rearLeftModule_driveMotor
        self.rearRightModule_encoder = self.rearRightModule_driveMotor

        self.frontLeftModule_cfg = {"sd_prefix":'frontLeft_Module', "inverted":True, "allow_reverse":True, "encoder":self.frontLeftModule_encoder}
        self.frontRightModule_cfg = {"sd_prefix":'frontRight_Module', "inverted":False, "allow_reverse":True, "encoder":self.frontRightModule_encoder}
        self.rearLeftModule_cfg = {"sd_prefix":'rearLeft_Module', "inverted":True, "allow_reverse":True, "encoder":self.rearLeftModule_encoder}
        self.rearRightModule_cfg = {"sd_prefix":'rearRight_Module', "inverted":False, "allow_reverse":True, "encoder":self.rearRightModule_encoder}

        self.frontLeftModule = swervemodule.SwerveModule(self.frontLeftModule_cfg, self.frontLeftModule_driveMotor, self.frontLeftModule_rotateMotor)
        self.frontRightModule = swervemodule.SwerveModule(self.frontRightModule_cfg, self.frontRightModule_driveMotor, self.frontRightModule_rotateMotor)
        self.rearLeftModule = swervemodule.SwerveModule(self.rearLeftModule_cfg, self.rearLeftModule_driveMotor, self.rearLeftModule_rotateMotor)
        self.rearRightModule = swervemodule.SwerveModule(self.rearRightModule_cfg, self.rearRightModule_driveMotor, self.rearRightModule_rotateMotor)

        self.drive = swervedrive.SwerveDrive(self.frontLeftModule, self.frontRightModule, self.rearLeftModule, self.rearRightModule)

    def autonomousInit(self):
        self.drive.flush()
    
    # def teleopInit(self):
    #     self.drive.flush()
    
    def move(self, x, y, rcw):
        self.drive.move(x, y, rcw)

    def teleopPeriodic(self):
        self.move(self.controller.getLeftY(), self.controller.getLeftX(), self.controller.getRightX())

        self.drive.execute()

if __name__ == "__main__":
    wpilib.run(MyRobot)