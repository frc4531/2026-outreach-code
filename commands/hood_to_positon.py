import commands2
import rev
from rev import SparkFlexConfig, SparkBase

from subsystems.turret_subsystem import TurretSubsystem


class HoodToPosition(commands2.Command):

    def __init__(self, turret_sub: TurretSubsystem, target_position) -> None:
        super().__init__()

        self.target_position = target_position

        self.turret_sub = turret_sub
        self.addRequirements(self.turret_sub)

    def execute(self) -> None:
        self.turret_sub.hood_pid_controller.setReference(self.target_position, rev.SparkBase.ControlType.kPosition)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.turret_sub.set_hood_speed(0)