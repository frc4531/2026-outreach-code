import commands2
import rev
from rev import SparkFlexConfig, SparkBase

from subsystems.turret_subsystem import TurretSubsystem
from wpimath.controller import PIDController

import ntcore

from subsystems.vision_subsystem import VisionSubsystem


class TurretToPosition(commands2.Command):

    def __init__(self, turret_sub: TurretSubsystem, vision_sub: VisionSubsystem, target_position) -> None:
        super().__init__()

        self.target_position = target_position
        self.vision_sub = vision_sub

        self.turret_sub = turret_sub
        self.addRequirements(self.turret_sub, self.vision_sub)

        self.turret_pid_controller = PIDController(0.001, 0, 0)

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        turret_table = nt_instance.getTable("turret_table")

        self.current_relative_output_position_entry = turret_table.getDoubleTopic("cur_rel_angle").publish()
        self.turret_pid_output_entry = turret_table.getDoubleTopic("turret_pid_output").publish()

    def execute(self) -> None:
        relative_angle = self.vision_sub.get_relative_angle(self.target_position)
        self.current_relative_output_position_entry.set(relative_angle)

        if 0 <= relative_angle <= 270:
            target_encoder_position = (relative_angle/270) * 140

            self.turret_sub.turret_pid_controller.setReference(target_encoder_position, rev.SparkBase.ControlType.kPosition)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.turret_sub.set_turret_speed(0)