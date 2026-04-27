import math

import commands2
import rev
from rev import SparkFlexConfig, SparkBase

from subsystems.drive_subsystem import DriveSubsystem
from wpimath.controller import PIDController

import ntcore


class DriveToEncoderPos(commands2.Command):

    def __init__(self, drive_sub: DriveSubsystem, x_speed, y_speed, target_angle, target_distance, target_threshold) -> None:
        super().__init__()

        self.x_speed = x_speed
        self.y_speed = y_speed

        self.target_distance = target_distance

        self.target_angle = target_angle

        self.target_threshold = target_threshold

        self.initial_reading = 0
        self.current_reading = 0

        self.real_distance = 0
        self.current_distance = 0

        self.drive_sub = drive_sub

        self.addRequirements(self.drive_sub)

        self.x_drive_pid_controller = PIDController(0.2, 0, 0)
        self.y_drive_pid_controller = PIDController(0.2, 0, 0)
        self.rot_pid_controller = PIDController(0.04, 0, 0)
        self.rot_pid_controller.enableContinuousInput(-180, 180)

        self.x_drive_pid_controller.setTolerance(target_threshold)
        self.x_drive_pid_controller.setTolerance(target_threshold)

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        drive_table = nt_instance.getTable("drive_table")

        self.target_distance_entry = drive_table.getDoubleTopic("target_distance").publish()

    def initialize(self):
        # self.drive_sub.reset_encoders()
        self.initial_reading = self.drive_sub.front_left.get_position().distance_ft

        self.rot_pid_controller.setSetpoint(self.target_angle)

    def execute(self) -> None:
        self.current_reading = self.drive_sub.front_left.get_position().distance_ft
        self.current_distance = abs(self.current_reading - self.initial_reading)

        rot_speed = -self.rot_pid_controller.calculate(self.drive_sub.get_heading())

        # x_speed = (-self.y_speed * math.cos(self.drive_sub.get_heading() * (math.pi / 180))) + (self.x_speed * math.sin(self.drive_sub.get_heading() * (math.pi / 180)))
        # y_speed = (self.y_speed * math.sin(self.drive_sub.get_heading() * (math.pi / 180))) + (self.x_speed * math.cos(self.drive_sub.get_heading() * (math.pi / 180)))

        gyro_degrees = self.drive_sub.get_heading()
        gyro_radians = gyro_degrees * math.pi / 180
        x_speed_adj = self.y_speed * math.cos(gyro_radians) + self.x_speed * math.sin(gyro_radians)
        y_speed_adj = -self.y_speed * math.sin(gyro_radians) + self.x_speed * math.cos(gyro_radians)

        self.drive_sub.drive(x_speed_adj, y_speed_adj, rot_speed, False, False)

        self.target_distance_entry.set(self.real_distance)

    def isFinished(self) -> bool:
        return self.current_distance >= self.target_distance

    def end(self, interrupted: bool) -> None:
        self.drive_sub.drive(0, 0, 0, False, False)