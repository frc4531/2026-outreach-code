import commands2
import rev
from rev import SparkFlexConfig, SparkBase

from subsystems.turret_subsystem import TurretSubsystem
from wpimath.controller import PIDController

import ntcore

from subsystems.vision_subsystem import VisionSubsystem


class HoodAndTurretToPositions(commands2.Command):

    def __init__(self, turret_sub: TurretSubsystem, vision_sub: VisionSubsystem, hood_target_position, turret_target_position) -> None:
        super().__init__()

        self.turret_target_position = turret_target_position
        self.hood_target_position = hood_target_position
        self.vision_sub = vision_sub

        self.turret_sub = turret_sub
        self.addRequirements(self.turret_sub, self.vision_sub)

        self.turret_pid_controller = PIDController(0.001, 0, 0)

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        turret_table = nt_instance.getTable("turret_table")
        mode_table = nt_instance.getTable("mode_table")

        self.current_relative_output_position_entry = turret_table.getDoubleTopic("cur_rel_angle").publish()
        self.turret_pid_output_entry = turret_table.getDoubleTopic("turret_pid_output").publish()
        self.passing = mode_table.getDoubleTopic("passing_mode").publish()

    def execute(self) -> None:
        relative_angle = self.vision_sub.get_relative_angle(self.turret_target_position)
        self.current_relative_output_position_entry.set(relative_angle)

        if 0 <= relative_angle <= 270:
            target_encoder_position = (relative_angle/270) * 140

            self.turret_sub.turret_pid_controller.setReference(target_encoder_position, rev.SparkBase.ControlType.kPosition)

        self.turret_sub.hood_pid_controller.setReference(self.hood_target_position, rev.SparkBase.ControlType.kPosition)

        if self.hood_target_position == -14 and self.turret_target_position == -180:
            self.passing.set(True)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.turret_sub.set_turret_speed(0)
        self.turret_sub.set_hood_speed(0)

        self.passing.set(False)