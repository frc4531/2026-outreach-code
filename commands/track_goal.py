import math

import commands2
import rev
from rev import SparkFlexConfig, SparkBase

from subsystems.turret_subsystem import TurretSubsystem
from wpimath.controller import PIDController

import ntcore
from wpilib import DriverStation

from subsystems.vision_subsystem import VisionSubsystem


class TrackGoal(commands2.Command):

    def __init__(self, turret_sub: TurretSubsystem, vision_sub: VisionSubsystem) -> None:
        super().__init__()

        self.target_x_coord = 0.0
        self.target_y_coord = 0.0
        self.vision_sub = vision_sub

        self.turret_sub = turret_sub
        self.addRequirements(self.turret_sub, self.vision_sub)

        self.turret_pid_controller = PIDController(0.05, 0, 0)
        self.hood_controller = self.turret_sub.hood_pid_controller

        self.close_distance = 1.4224
        self.far_distance = 5 #5.2832
        self.close_encoder = -0.1
        self.far_encoder = -17.5 #-18

        self.angle_velocity_adjustment_controller = PIDController(0, 0, 0)
        self.distance_velocity_adjustment_controller = PIDController(0, 0, 0)

        self.distance_range = abs(self.close_distance - self.far_distance)
        self.encoder_range = abs(self.far_encoder - self.close_encoder)
        self.cam_angle_ratio = self.encoder_range / self.distance_range

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        turret_table = nt_instance.getTable("turret_table")
        tracking_table = nt_instance.getTable("tracking_table")
        state_table = nt_instance.getTable("state_table")

        self.current_relative_output_position_entry = turret_table.getDoubleTopic("cur_rel_angle").publish()
        self.turret_pid_output_entry = turret_table.getDoubleTopic("turret_pid_output").publish()

        self.distance_entry = tracking_table.getDoubleTopic("distance_entry").publish()
        self.target_position_entry = tracking_table.getDoubleTopic("target_position_entry").publish()
        self.target_encoder_entry = tracking_table.getDoubleTopic("target_encoder_entry").publish()

        self.is_tracking_state = state_table.getDoubleTopic("is_tracking").publish()

    def initialize(self) -> None:
        ally = DriverStation.getAlliance() # DriverStation.getAlliance()
        if ally is not None:
            if ally == DriverStation.Alliance.kRed:
                self.target_x_coord = 11.875 #4.035
                self.target_y_coord = 4.035 #11.915
            elif ally == DriverStation.Alliance.kBlue:
                self.target_x_coord = 4.626 #4.035
                self.target_y_coord = 4.035 #4.626

    def execute(self) -> None:
        # Turret (angle) Control Block
        y_distance = self.vision_sub.corrected_turret_y_coord - self.target_y_coord
        x_distance = self.vision_sub.corrected_turret_x_coord - self.target_x_coord

        drive_velocity = 0

        target_angle = (math.atan(y_distance / x_distance)) * (180 / math.pi) + self.angle_velocity_adjustment_controller.calculate(drive_velocity)

        relative_angle = self.vision_sub.get_relative_angle(target_angle)
        self.current_relative_output_position_entry.set(relative_angle)

        if 0 <= relative_angle <= 270:
            target_encoder_position = (relative_angle / 270) * 140

            self.turret_sub.turret_pid_controller.setReference(target_encoder_position, rev.SparkBase.ControlType.kPosition)

        # Hood (distance) Control Block

        current_distance = math.sqrt(x_distance ** 2 + y_distance ** 2)
        distance_adjustment = self.distance_velocity_adjustment_controller.calculate(drive_velocity)

        if self.close_distance < current_distance < self.far_distance:
            target_encoder = -((((current_distance - self.close_distance) * self.encoder_range) / self.distance_range) + self.close_encoder)
            target_position = (max(self.far_encoder, min(self.close_encoder, target_encoder)))
            self.hood_controller.setReference(target_position, rev.SparkBase.ControlType.kPosition)
        elif current_distance < self.close_distance:
            target_position = self.close_encoder
            self.hood_controller.setReference(target_position, rev.SparkBase.ControlType.kPosition)
        elif current_distance > self.far_distance:
            target_position = self.far_encoder
            self.hood_controller.setReference(target_position, rev.SparkBase.ControlType.kPosition)
        else:
            self.turret_sub.set_hood_speed(0)

        self.distance_entry.set(current_distance)
        self.target_position_entry.set(target_position)
        self.is_tracking_state.set(True)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.turret_sub.set_turret_speed(0)
        self.turret_sub.set_hood_speed(0)
        self.is_tracking_state.set(False)