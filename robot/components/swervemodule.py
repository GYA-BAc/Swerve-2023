import math

import wpilib
import ctre

from networktables import NetworkTables
from wpimath.controller import PIDController
from collections import namedtuple

ModuleConfig = namedtuple('ModuleConfig', ['sd_prefix', 'zero', 'inverted', 'allow_reverse'])

ENCODER_SIZE = 4096

class SwerveModule:
    # driveMotor: ctre.WPI_TalonSRX
    # rotateMotor: ctre.WPI_TalonSRX
        
    # encoder: ctre.WPI_TalonSRX

    # cfg: ModuleConfig

    def __init__(self, config, drive, rotate):

        self.driveMotor = drive
        self.rotateMotor = rotate

        self.cfg = config

        self.encoder = self.cfg["encoder"]

        print(self.cfg)
        self.encoder_zero = self.encoder.getSelectedSensorPosition()

        self.inverted = self.cfg["inverted"] or False
        self.allow_reverse = self.cfg["allow_reverse"] or True

        self.driveMotor.setInverted(self.inverted)

        # self.requested_voltage = 0
        self.requested_ticks = 0
        self.requested_speed = 0

        self.pid_controller = PIDController(0.0000001, 0.0, 0.0)
        self.pid_controller.enableContinuousInput(0.0, 5.0) # Will set the 0 and 5 as the same point
        self.pid_controller.setTolerance(0.05, 0.05) # Tolerance where the PID will be accpeted aligned

    # def setup(self):
        # self.encoder_zero = self.cfg.zero or 0
        
        #NOTE: assumes when bot is turned on, wheels will be pointing forward
        
        
    # def get_voltage(self):
        # return self.encoder.getMotorOutputVoltage() - self.encoder_zero
        
    def get_encoder_ticks(self):
        return self.encoder.getSelectedSensorPosition()

    def flush(self):
        
        # self.requested_voltage = self.encoder_zero
        self.requested_ticks = self.encoder_zero
        self.requested_speed = 0
        self.pid_controller.reset()
    
    # @staticmethod
    # def voltage_to_degrees(voltage):
    #     deg = (voltage / 5) * 360

    #     if deg < 0:
    #         deg += 360
        
    #     return deg
    
    @staticmethod
    def ticks_to_degrees(ticks):
        deg = (ticks % ENCODER_SIZE)/ENCODER_SIZE
        deg *= 360

        return deg


    @staticmethod
    # def degree_to_voltage(degree):
    def degree_to_ticks(degree):
        return (degree / 360) * ENCODER_SIZE
    
    def set_deg(self, value):
        # self.requested_voltage = ((self.degree_to_voltage(value) + self.encoder_zero) % 5)
        self.requested_ticks = ((self.degree_to_ticks(value) + self.encoder_zero) % ENCODER_SIZE)

    def move(self, speed, deg):
        deg %= 360

        if self.allow_reverse:

            if abs(deg - self.ticks_to_degrees(self.get_encoder_ticks())) > 90:
                speed *= -1
                deg += 180
                deg %= 360
            
        self.requested_speed = speed
        self.set_deg(deg)
        
    def execute(self):
        error = self.pid_controller.calculate(self.encoder.getSelectedSensorPosition(), self.requested_ticks)

        output = 0

        if not self.pid_controller.atSetpoint():
            output = max(min(error,1), -1)

        self.rotateMotor.set(output)
        self.driveMotor.set(self.requested_speed)
            

        
        